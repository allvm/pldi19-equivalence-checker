// Copyright 2013-2019 Stanford University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "src/serialize/serialize.h"
#include "src/cfg/cfg.h"

using namespace cpputil;
using namespace std;
using namespace stoke;
using namespace x64asm;

//#define STOKE_CFG_ALL_INSTR_BLOCK
//#define STOKE_CFG_AFTER_LABEL_NEW_BLOACK
//#define STOKE_CFG_COMPARE_NEW_BLOCK
//#define DEBUG_CFG_RD

namespace stoke {

Assembler Cfg::assembler_ = Assembler();
Function Cfg::buffer_ = Function();

Cfg::loc_type Cfg::get_loc(size_t idx) const {
  assert(idx < get_code().size());
  for (auto i = get_exit() - 1; i > get_entry(); --i)
    if (idx >= blocks_[i]) {
      return loc_type(i, idx - blocks_[i]);
    }

  assert(false);
  return loc_type(0, 0);
}

bool Cfg::invariant_no_undef_reads() const {
  // NOTE: if this method changes, then which_undef_read must be adapted accordingly!

  // No need to check the entry; this will consider the exit, but it will be a nop.
  for (auto i = ++reachable_begin(), ie = reachable_end(); i != ie; ++i) {
    for (size_t j = 0, je = num_instrs(*i); j < je; ++j) {
      const auto idx = get_index({*i, j});
      const auto r = maybe_read_set(get_code()[idx]);
      const auto di = def_ins_[idx];

      if (!di.contains(r)) {
        return false;
      }
    }
  }

  return true;
}

bool Cfg::invariant_no_undef_live_outs() const {
  // NOTE: if this method changes, then which_undef_read must be adapted accordingly!
  const auto di_end = def_ins_[blocks_[get_exit()]];
  return di_end.contains(fxn_live_outs_);
}

bool Cfg::invariant_can_assemble() const {
  bool need_check = false;
  auto code = get_code();
  for (auto instr : code) {
    Opcode op = instr.get_opcode();
    if (label32_transform(op) != op) {
      need_check = true;
      break;
    }
  }

  if (!need_check)
    return true;

  buffer_.clear();
  buffer_.reserve(code.size()*32);
  return assembler_.assemble(buffer_, code);
}

string Cfg::which_undef_read() const {
  // NOTE: if this method changes, then performs_undef_read must be adapted accordingly!
  assert(performs_undef_read());

  ostringstream ss;
  bool empty = true;

  // No need to check the entry; this will consider the exit, but it will be a nop.
  for (auto i = ++reachable_begin(), ie = reachable_end(); i != ie; ++i) {
    for (size_t j = 0, je = num_instrs(*i); j < je; ++j) {
      const auto idx = get_index({*i, j});
      const auto r = maybe_read_set(get_code()[idx]);
      const auto di = def_ins_[idx];

      if (!di.contains(r)) {
        ss << (empty ? "" : ". ") << "Instruction '" << get_code()[idx] << "' reads " << r << " but only " << di << " are defined.";
        empty = false;
        return ss.str();
      }
    }
  }

  // Check that the live outs are all defined
  // i.e. every life_out is also def in at the end
  const auto di_end = def_ins_[blocks_[get_exit()]];
  if (!di_end.contains(fxn_live_outs_)) {
    ss << (empty ? "" : ". ") << "At the end, " << fxn_live_outs_ << " should be defined, but only " << di_end << " are.";
    empty = false;
  }

  assert(!empty);
  return ss.str();
}

void Cfg::recompute_blocks() {
  blocks_.clear();

  boundaries_.resize_for_bits(get_code().size() + 1);
  boundaries_.reset();

  // We know a-priori that these are boundaries
  boundaries_[0] = true;
  boundaries_[get_code().size()] = true;

  // Labels define the beginning of blocks; jumps and returns define the ends. */
  for (size_t i = 0, ie = get_code().size(); i < ie; ++i) {
    const auto& instr = get_code()[i];
#ifdef STOKE_CFG_ALL_INSTR_BLOCK
    boundaries_[i] = true;
    continue;
#endif
    if (instr.is_label_defn()) {
      boundaries_[i] = true;
#ifdef STOKE_CFG_AFTER_LABEL_NEW_BLOACK
      boundaries_[i+1] = true;
#endif
    } else if (instr.is_jump() || instr.is_return()) {
      boundaries_[i + 1] = true;
    }
#ifdef STOKE_CFG_COMPARE_NEW_BLOCK
    else {
      auto mws = instr.maybe_write_set();
      if (mws.flags_begin() != mws.flags_end()) {
        /* make compares start a new basic block... this simplifies things
           for the validator. */
        boundaries_[i] = true;
      }
    }
#endif
  }

  // Add sentinels for entry and exit blocks along with boundaries
  blocks_.push_back(0);
  for (auto i = boundaries_.set_bit_index_begin(), ie = boundaries_.set_bit_index_end(); i != ie;
       ++i) {
    blocks_.push_back(*i);
  }
  blocks_.push_back(get_code().size());
}

void Cfg::recompute_labels() {
  labels_.clear();
  for (auto i = get_entry() + 1, ie = get_exit(); i < ie; ++i) {
    if (num_instrs(i) > 0) {
      const auto& instr = get_code()[get_index({i, 0})];
      if (instr.is_label_defn()) {
        labels_[instr.get_operand<Label>(0)] = i;
      }
    }
  }
}

void Cfg::recompute_succs() {
  succs_.resize(num_blocks());
  for (auto& s : succs_) {
    s.clear();
  }

  for (auto i = get_entry(), ie = get_exit(); i < ie; ++i) {
    // Control passes from empty blocks to the next.
    if (num_instrs(i) == 0) {
      succs_[i].push_back(i + 1);
      continue;
    }
    // Control passes from return statements to the exit.
    const auto& instr = get_code()[get_index({i, num_instrs(i) - 1})];
    if (instr.is_return()) {
      succs_[i].push_back(get_exit());
      continue;
    }
    // Conditional jump targets are always listed second in succs_.
    const auto itr = labels_.find(instr.get_operand<Label>(0));
    const auto dest = itr == labels_.end() ? get_exit() : itr->second;
    if (instr.is_uncond_jump()) {
      succs_[i].push_back(dest);
    } else {
      succs_[i].push_back(i + 1);
      if (instr.is_cond_jump()) {
        succs_[i].push_back(dest);
      }
    }
  }
}

void Cfg::recompute_preds() {
  preds_.resize(num_blocks());
  for (auto& p : preds_) {
    p.clear();
  }

  for (auto i = get_entry(), ie = get_exit(); i < ie; ++i) {
    for (auto s = succ_begin(i), se = succ_end(i); s != se; ++s) {
      preds_[*s].push_back(i);
    }
  }
}

void Cfg::recompute_preds_instrs() {
  preds_instrs_.resize(get_code().size()+1);
  for (auto& p : preds_instrs_)
    p.clear();

  for (auto i = get_entry(), ie = get_exit(); i < ie; ++i) {
    for (size_t j = 0, je = num_instrs(i); j < je; ++j) {
      if (j == 0) {
        for (auto p = pred_begin(i), pe = pred_end(i); p != pe; ++p) {
          preds_instrs_[get_index({i, 0})].push_back({*p, num_instrs(*p)-1});
        }
      } else {
        preds_instrs_[get_index({i, j})].push_back({i, j-1});
      }
    }
  }
#ifdef DEBUG_CFG_RD
  std::cout << "\tTesting Predecessors Iterators:\n=====================\n\n";
  for (size_t k = 0 ; k < get_code().size(); k++) {
    std::cout << get_code()[k] << "\n";
    std::cout << "\tpredecessors:\n";
    for (auto p = pred_begin_instr(k), pe = pred_end_instr(k); p != pe; ++p) {
      std::cout << "\t\t" << (*p).first << " " << (*p).second << "\n";
      //std::cout << "\t\t" << get_code()[get_index(*p)] << "\n";
    }
  }
#endif
}

void Cfg::recompute_reachable() {
  reachable_.resize_for_bits(num_blocks());
  reachable_.reset();

  work_list_.clear();

  work_list_.push_back(get_entry());
  reachable_[get_entry()] = true;

  for (size_t i = 0; i < work_list_.size(); ++i) {
    const auto next = work_list_[i];
    for (auto s = succ_begin(next), se = succ_end(next); s != se; ++s) {
      if (!reachable_[*s]) {
        reachable_[*s] = true;
        work_list_.push_back(*s);
      }
    }
  }
}

/*
  The data flow information for each instruction is a vector of size get_code().size() + 1.
  Each element of that vector hols a set of registers.

  The gen set of an instruction i is reaching_defs_in_gen_i][i]

  The kill set of an instruction i is reaching_defs_in_gen_i][j], where j is all the istructions which are killed
  and the content of reaching_defs_in_gen_i][j] is the regs if j which are killed by i
*/
void Cfg::recompute_reaching_defs_in_gen_kill() {
  reaching_defs_in_gen_.resize(get_code().size() + 1, Dfv_RD(get_code().size() + 1, RegSet::empty()));
  reaching_defs_in_kill_.resize(get_code().size() + 1, Dfv_RD(get_code().size() + 1, RegSet::empty()));
  x64asm::RegSet gen, kill, aux_kill;

  // Find the gen set for all instructions
  for (auto i = ++reachable_begin(), ie = reachable_end(); i != ie; ++i) {
    for (size_t j = 0, je = num_instrs(*i); j < je; ++j) {
      const auto idx = get_index({*i, j});
      gen = RegSet::empty();
      gen |= must_write_set(get_code()[idx]);
      gen -= maybe_undef_set(get_code()[idx]);
      reaching_defs_in_gen_[idx][idx+1] = gen;
    }
  }

  // Find the kill set for all instructions
  for (auto i = ++reachable_begin(), ie = reachable_end(); i != ie; ++i) {
    for (size_t j = 0, je = num_instrs(*i); j < je; ++j) {
      const auto idx = get_index({*i, j});

      kill = RegSet::empty();
      kill |= must_write_set(get_code()[idx]);
      kill -= maybe_undef_set(get_code()[idx]);

      for (size_t k = 0 ; k < get_code().size() + 1; k++) {
        if (k == idx + 1) continue;

        if (k == 0) {
          reaching_defs_in_kill_[idx][k] = kill;
        } else {
          gen = reaching_defs_in_gen_[k-1][k];
          auto aux_kill = gen & kill;
          reaching_defs_in_kill_[idx][k] = aux_kill;
        }
      }
    }
  }
}

void Cfg::recompute_reaching_defs_in() {
  recompute_reaching_defs_in_gen_kill();

  reaching_defs_in_.resize(get_code().size() + 1, Dfv_RD(get_code().size() + 1, RegSet::empty()));
  reaching_defs_out_.resize(get_code().size() + 1,Dfv_RD(get_code().size() + 1, RegSet::empty()));

  // Boundary conditions
  //def_outs_[get_entry()] = fxn_def_ins_;
  reaching_defs_out_[get_entry()] = fxn_reaching_defs_ins_;

  // Initial conditions
  for (auto i = ++reachable_begin(), ie = reachable_end(); i != ie; ++i) {
    reaching_defs_out_[*i] = fxn_reaching_defs_ins_;
  }

  // Iterate until fixed point
  for (auto changed = true; changed;) {
    changed = false;

    for (auto i = ++reachable_begin(), ie = reachable_end(); i != ie; ++i) {
      for (size_t j = 0, je = num_instrs(*i); j < je; ++j) {
        const auto idx = get_index({*i, j});

        if (j == 0) { // Do Meet of the predecessors
          for (auto p = pred_begin_instr({*i, j}), pe = pred_end_instr({*i, j}); p != pe; ++p) {
            if ((*p).first == get_entry())
              reaching_defs_in_[idx] |= reaching_defs_out_[get_entry()];
            else
              reaching_defs_in_[idx] |= reaching_defs_out_[get_index(*p)];
          }
        } else {
          reaching_defs_in_[idx] = reaching_defs_out_[idx-1];
        }

        // Transfer function
        const auto new_out = (reaching_defs_in_[idx] - reaching_defs_in_kill_[idx]) | reaching_defs_in_gen_[idx];

        // Check for fixed point
        changed |= reaching_defs_out_[idx] != new_out;
        reaching_defs_out_[idx] = new_out;
      }
    }
  }

  // Find the reaching defintions which are actually used at the program points
  reaching_and_used_defs_in_.resize(get_code().size() + 1, Dfv_RD(get_code().size() + 1, RegSet::empty()));
  for (auto i = ++reachable_begin(), ie = reachable_end(); i != ie; ++i) {
    for (size_t j = 0, je = num_instrs(*i); j < je; ++j) {
      const auto idx = get_index({*i, j});
      auto rd_ins = reaching_defs_in({*i, j});
      auto read_set = maybe_read_set(get_code()[idx]);

      for (size_t k = 0 ; k < rd_ins.size(); k++) {
        if (rd_ins[k] == RegSet::empty()) {
          continue;
        }
        reaching_and_used_defs_in_[idx][k] = rd_ins[k] & read_set;
      }
    }
  }

#ifdef DEBUG_CFG_RD
  for (size_t k = 0 ; k < get_code().size(); k++) {
    std::cout << get_code()[k] << "\n";
    auto rd_def_in_ = reaching_defs_in_[k];
    auto rd_def_out_ = reaching_defs_out_[k];
    auto gen_rs = reaching_defs_in_gen_[k];
    auto kill_rs = reaching_defs_in_kill_[k];
    auto rd_def_in_used_ = reaching_and_used_defs_in_[k];

    auto must_write = must_write_set(get_code()[k]);
    auto may_write  = maybe_write_set(get_code()[k]);

    auto must_read = must_read_set(get_code()[k]);
    auto may_read  =  maybe_read_set(get_code()[k]);

    auto must_undef = must_undef_set(get_code()[k]);
    auto may_undef  = maybe_undef_set(get_code()[k]);

    std::cout << "\twrite-set: \n";
    std::cout << "\t\tMust: [" << must_write << "]\n";
    std::cout << "\t\tMay: [" << may_write <<  "]\n";
    std::cout << "\tread-set: \n";
    std::cout << "\t\tMust: [" << must_read << "]\n";
    std::cout << "\t\tMay: [" << may_read <<  "]\n";
    std::cout << "\tundef-set: \n";
    std::cout << "\t\tMust: [" << must_undef << "]\n";
    std::cout << "\t\tMay: [" << may_undef <<  "]\n";
    std::cout << "\n\n";

    std::cout << "\tgen-set: \n";
    for (size_t i = 0 ; i < gen_rs.size(); i++) {
      if (gen_rs[i] == RegSet::empty()) continue;

      std::cout << "\t\t[\n";
      if (i == 0)
        std::cout << "\t\t\t" << "E0" << "\n";
      else
        std::cout << "\t\t\t" << get_code()[i-1] << "\n";
      std::cout << "\t\t\t\t" <<  gen_rs[i] << "\n";
      std::cout << "\t\t]\n\n";
    }

    std::cout << "\tkill-set: \n";
    for (size_t i = 0 ; i < kill_rs.size(); i++) {
      if (kill_rs[i] != RegSet::empty()) {
        std::cout << "\t\t[\n";
        if (i == 0)
          std::cout << "\t\t\t" << "E0" << "\n";
        else
          std::cout << "\t\t\t" << get_code()[i-1] << "\n";
        std::cout << "\t\t\t\t" << kill_rs[i] << "\n";
        std::cout << "\t\t]\n\n";
      }
    }

    std::cout << "\treaching_defs_in_: \n";
    for (size_t i = 0 ; i < rd_def_in_.size(); i++) {
      if (rd_def_in_[i] != RegSet::empty()) {
        std::cout << "\t\t[\n";

        if (i == 0)
          std::cout << "\t\t\t" << "E0" << "\n";
        else
          std::cout << "\t\t\t" << get_code()[i-1] << "\n";

        std::cout << "\t\t\t\t" << rd_def_in_[i] << "\n";
        std::cout << "\t\t]\n\n";
      }
    }

    std::cout << "\treaching_defs_out_: \n";
    for (size_t i = 0 ; i < rd_def_out_.size(); i++) {
      if (rd_def_out_[i] != RegSet::empty()) {
        std::cout << "\t\t[\n";
        if (i == 0)
          std::cout << "\t\t\t" << "E0" << "\n";
        else
          std::cout << "\t\t\t" << get_code()[i-1] << "\n";

        std::cout << "\t\t\t\t" << rd_def_out_[i] << "\n";
        std::cout << "\t\t]\n\n";
      }
    }

    std::cout << "\treaching_defs_in_used_: \n";
    for (size_t i = 0 ; i < rd_def_in_used_.size(); i++) {
      if (rd_def_in_used_[i] != RegSet::empty()) {
        std::cout << "\t\t[\n";
        if (i == 0)
          std::cout << "\t\t\t" << "E0" << "\n";
        else
          std::cout << "\t\t\t" << get_code()[i-1] << "\n";

        std::cout << "\t\t\t\t" << rd_def_in_used_[i] << "\n";
        std::cout << "\t\t]\n\n";
      }
    }
  }

#endif

}

void Cfg::recompute_defs_gen_kill() {
  gen_.assign(num_blocks(), RegSet::empty());
  kill_.assign(num_blocks(), RegSet::empty());

  // No sense in checking the entry; we'll consider the exit, but it'll be a nop.
  for (auto i = ++reachable_begin(), ie = reachable_end(); i != ie; ++i) {
    for (auto j = instr_begin(*i), je = instr_end(*i); j != je; ++j) {
      gen_[*i] |= must_write_set(*j);
      gen_[*i] -= maybe_undef_set(*j);

      kill_[*i] |= maybe_undef_set(*j);
      kill_[*i] -= maybe_write_set(*j);
    }
  }
}
void Cfg::recompute_defs() {
  recompute_defs_gen_kill();

  // Need a little extra room for def_ins_[get_exit()]
  // You'll notice that this function uses blocks_[...] instead of get_index(...)
  // This is to subvert the assertion we'd blow for trying to call get_index(get_exit(),0)

  def_ins_.resize(get_code().size() + 1, RegSet::empty());
  def_outs_.resize(num_blocks(), RegSet::empty());

  // Boundary conditions
  def_outs_[get_entry()] = fxn_def_ins_;

  // Initial conditions
  for (auto i = ++reachable_begin(), ie = reachable_end(); i != ie; ++i) {
    def_outs_[*i] = RegSet::universe();
  }

  // Iterate until fixed point
  for (auto changed = true; changed;) {
    changed = false;

    for (auto i = ++reachable_begin(), ie = reachable_end(); i != ie; ++i) {
      // Meet operator
      def_ins_[blocks_[*i]] = RegSet::universe();
      for (auto p = pred_begin(*i), pe = pred_end(*i); p != pe; ++p) {
        if (is_reachable(*p)) {
          def_ins_[blocks_[*i]] &= def_outs_[*p];
        }
      }
      // Transfer function
      const auto new_out = (def_ins_[blocks_[*i]] - kill_[*i]) | gen_[*i];

      // Check for fixed point
      changed |= def_outs_[*i] != new_out;
      def_outs_[*i] = new_out;
    }
  }

  // Compute dataflow values for each instruction
  for (auto i = ++reachable_begin(), ie = reachable_end(); i != ie; ++i) {
    for (size_t j = 1, je = num_instrs(*i); j < je; ++j) {
      const auto idx = blocks_[*i] + j;
      def_ins_[idx] = def_ins_[idx - 1];

      const auto& instr = get_code()[idx - 1];
      def_ins_[idx] |= must_write_set(instr);
      def_ins_[idx] -= maybe_undef_set(instr);
    }
  }
}

void Cfg::recompute_liveness() {
  recompute_liveness_use_kill();

  // IMPORTANT NOTE: both vectors indexed by code size
  live_ins_.assign(get_code().size() + 1, RegSet::empty());
  live_outs_.assign(get_code().size() + 1, RegSet::empty());

  // If we ever encounter an indirect jump, we need to assume that everything
  // which we ever use (i.e. read) becomes live-out at that point.  So, let's
  // get the set of stuff we *ever* read.
  RegSet ever_read = fxn_live_outs_;

  // Note: maybe_read_set doesn't work for call, which
  // is why I need this loop.
  for (auto it : get_code()) {
    if (it.is_call()) {
      ever_read |= maybe_read_set(it);
    } else if (it.is_any_call()) {
      // we don't support this.
      // abort is a mean, evil way.
      std::cerr << "Instruction " << it << " not supported by liveness."
                << " @ " << __FILE__ << ":" << __LINE__ << endl;
      exit(1);
    } else {
      ever_read |= maybe_read_set(it);
    }

  }


  // Initial Conditions
  for (auto i = reachable_begin(), ie = reachable_end();
       i != ie; ++i) {

    if (num_instrs(*i) == 0) {
      continue;
    }

    // Set the live-in of each block to the empty set.
    live_ins_[blocks_[*i]] = RegSet::empty();

    // Set the live-out of each block to the empty set.  this requires
    // looking up the index of the last instruction in the block.  Except if
    // we have an indirect jump, in which case we need to add in everything
    // (see the note above)
    size_t last_instr_index = blocks_[*i] + num_instrs(*i) - 1;
    Instruction last_instr = get_code()[last_instr_index];
    if (last_instr.is_any_indirect_jump()) {
      live_outs_[last_instr_index] = ever_read;
    } else {
      live_outs_[last_instr_index] = RegSet::empty();
    }
  }
  live_ins_[blocks_[get_exit()]] = fxn_live_outs_;

  // Fixedpoint algorithm
  for (auto changed = true; changed;) {
    changed = false;

    for (auto i = reachable_begin(), ie = reachable_end();
         i != ie; ++i) {
      //iterate through all blocks except the exit
      if (num_instrs(*i) == 0) {
        continue;
      }

      // Meet operator.  Starting with the empty-set, union in the live-ins
      // from the first statement of every successor block.  Like before, we
      // need to check for indirect jumps here to see if we need to add in
      // all the registers ever read.
      size_t last_instr_index = blocks_[*i] + num_instrs(*i) - 1;
      Instruction last_instr = get_code()[last_instr_index];
      if (last_instr.is_any_indirect_jump()) {
        live_outs_[last_instr_index] = ever_read;
      } else {
        live_outs_[last_instr_index] = RegSet::empty();
      }

      for (auto s = succ_begin(*i), si = succ_end(*i); s != si; ++s) {
        if (is_reachable(*s)) {
          live_outs_[last_instr_index] |= live_ins_[blocks_[*s]];
        }
      }

      // Transfer function.
      // Take the live outs at the end of the block, remove the
      // kill set, and union in the use set.
      const auto new_in =
        (live_outs_[blocks_[*i] + num_instrs(*i) - 1] - liveness_kill_[*i]) |
        liveness_use_[*i];

      changed |= live_ins_[blocks_[*i]] != new_in;
#ifdef DEBUG_CFG_LIVENESS
      if (changed) {
        cout << "block " << *i << " from " << live_ins_[blocks_[*i]] << " --> " << new_in << endl;
        cout << "   " << "live out: " << live_outs_[blocks_[*i] + num_instrs(*i) - 1] << endl;
        cout << "   " << "kill: " << liveness_kill_[*i] << endl;
        cout << "   " << "use:  " << liveness_use_[*i] << endl;
      }
#endif
      live_ins_[blocks_[*i]] = new_in;
    }
  }

  // Compute dataflow values for each instruction
  for (auto i = reachable_begin(), ie = reachable_end(); i != ie; ++i) {
    //iterate through all blocks w/ at least 2 instructions
    if (num_instrs(*i) < 2) {
      continue;
    }

    // Go from the second-to-last instruction to the first
    // Update the live outs for each
    for (int j = num_instrs(*i) - 2; j >= 0; --j) {
      const auto idx = blocks_[*i] + j;
      live_outs_[idx] = live_outs_[idx + 1];

      const auto& instr = get_code()[idx + 1];
      live_outs_[idx] -= must_write_set(instr);
      live_outs_[idx] -= must_undef_set(instr);
      live_outs_[idx] |= maybe_read_set(instr);

      live_ins_[idx + 1] = live_outs_[idx];
    }
  }

}


void Cfg::recompute_liveness_use_kill() {
  liveness_use_.assign(num_blocks(), RegSet::empty());
  liveness_kill_.assign(num_blocks(), RegSet::empty());

  // No sense in checking the entry; we'll consider the exit, but it'll be a nop.
  for (auto i = reachable_begin(), ie = reachable_end(); i != ie; ++i) {
    for (auto j = instr_begin(*i), je = instr_end(*i); j != je; ++j) {

      /*      if(j->is_call()) {
              liveness_use_[*i] |= (RegSet::linux_call_parameters() - liveness_kill_[*i]);
              liveness_kill_[*i] |= RegSet::linux_call_scratch();

            } else {*/
      liveness_use_[*i] |= (maybe_read_set(*j) - liveness_kill_[*i]);

      liveness_kill_[*i] |= must_undef_set(*j);
      liveness_kill_[*i] |= must_write_set(*j);
      //      }
    }
  }
}

void Cfg::serialize(ostream& os) const {
  stoke::serialize<x64asm::RegSet>(os, fxn_def_ins_);
  stoke::serialize<x64asm::RegSet>(os, fxn_live_outs_);
  stoke::serialize<TUnit>(os, function_);
}

Cfg Cfg::deserialize(istream& is) {
  auto defins = stoke::deserialize<x64asm::RegSet>(is);
  auto liveouts = stoke::deserialize<x64asm::RegSet>(is);
  auto tunit = stoke::deserialize<TUnit>(is);
  return Cfg(tunit, defins, liveouts);
}


} // namespace x64
