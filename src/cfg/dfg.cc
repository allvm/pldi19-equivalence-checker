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

#include "src/cfg/dfg.h"
#include "src/serialize/serialize.h"

using namespace cpputil;
using namespace std;
using namespace stoke;
using namespace x64asm;

#define DEBUG_CFG_RD

namespace stoke {

/*
** Class DataValue
*/
bool DataValue::isRegOrMem() const {
  return type == REG || isMemory();
}

bool DataValue::isReg() const {
  return type == REG || isFlag();
}

bool DataValue::isMemory() const {
  return type == EXPLICIT_MEM || type == IMPLICIT_MEM;
}

bool DataValue::isFlag() const {
  return type == EFLAG;
}

//Operand DataValue::getReg() const {
//  assert((type == REG || type == MEM) && "Reg or Mem Absent!");
//  if (RegOrMemOrFlag.gp_begin() != RegOrMemOrFlag.gp_end())
//    return *RegOrMemOrFlag.gp_begin();
//  // if(RegOrMemOrFlag.sse_begin() != RegOrMemOrFlag.sse_begin())
//  return *RegOrMemOrFlag.sse_begin();
//}
//Eflags DataValue::getFlag() const {
//  assert(type == 2 && "Flag Absent!");
//  return *RegOrMemOrFlag.flags_begin();
//}

size_t DataValue::getInstrIndex() const {
  return instr_idx;
}

RegSet DataValue::getRegOrMemOrFlag() const {
  return RegOrMemOrFlag;
}

void DataValue::addFlag(const Eflags &rhs) {
  RegOrMemOrFlag += rhs;
  type = EFLAG;
}

bool DataValue::kills(DataValue *target) {
  if ((isReg() && target->isMemory()) || (isMemory() && target->isReg())) {
    return false;
  }

  if (isMemory() && target->isMemory()) return true;
  return RegOrMemOrFlag == target->RegOrMemOrFlag;
}

void DataValue::addRegOrMem(Operand rhs) {
  if (rhs.is_gp_register()) {
    Type t = rhs.type();
    switch (t) {
    case Type::RH: {
      Rh *r = static_cast<Rh *>(&rhs);
      RegOrMemOrFlag += *r;
      break;
    }
    case Type::AL:
    case Type::CL:
    case Type::R_8: {
      R8 *r = static_cast<R8 *>(&rhs);
      RegOrMemOrFlag += *r;
      break;
    }
    case Type::R_16:
    case Type::AX:
    case Type::DX: {
      R16 *r = static_cast<R16 *>(&rhs);
      RegOrMemOrFlag += *r;
      break;
    }
    case Type::R_32:
    case Type::EAX: {
      R32 *r = static_cast<R32 *>(&rhs);
      RegOrMemOrFlag += *r;
      break;
    }
    case Type::R_64:
    case Type::RAX: {
      R64 *r = static_cast<R64 *>(&rhs);
      RegOrMemOrFlag += *r;
      break;
    }
    default:
      assert(false && "addRegOrMem: Unknown register!!");
    }
    type = REG;
    return;
  }

  if (rhs.is_sse_register()) {
    Type t = rhs.type();
    switch (t) {
    case Type::XMM:
    case Type::XMM_0: {
      Xmm *r = static_cast<Xmm *>(&rhs);
      RegOrMemOrFlag += *r;
      break;
    }
    case Type::YMM: {
      Ymm *r = static_cast<Ymm *>(&rhs);
      RegOrMemOrFlag += *r;
      break;
    }
    default:
      assert(false && "addRegOrMem: Unknown register!!");
    }
    type = REG;
    return;
  }

  if (rhs.is_typical_memory()) {
    Type t = rhs.type();
    switch (t) {
    case Type::M_8: {
      M<M8> *r = static_cast<M<M8>*>(&rhs);
      //RegOrMemOrFlag += *r;
      mem = *r;
      break;
    }
    case Type::M_16: {
      M<M16> *r = static_cast<M<M16>*>(&rhs);
      //RegOrMemOrFlag += *r;
      mem = *r;
      break;
    }
    case Type::M_32: {
      M<M32> *r = static_cast<M<M32>*>(&rhs);
      //RegOrMemOrFlag += *r;
      mem = *r;
      break;
    }
    case Type::M_64: {
      M<M64> *r = static_cast<M<M64>*>(&rhs);
      //RegOrMemOrFlag += *r;
      mem = *r;
      break;
    }
    case Type::M_128: {
      M<M128> *r = static_cast<M<M128>*>(&rhs);
      //RegOrMemOrFlag += *r;
      mem = *r;
      break;
    }
    case Type::M_256: {
      M<M256> *r = static_cast<M<M256>*>(&rhs);
      //RegOrMemOrFlag += *r;
      mem = *r;
      break;
    }
    default:
      assert(false && "addRegOrMem: Unknown memory!!");
    }

    type = EXPLICIT_MEM;
    return;
  }

  assert(0 && "End of addRegOrMem!!");
}

stringstream &DataValue::print(stringstream &os, const Dfg *dfg) const {
  stringstream ss;
  if (type == EXPLICIT_MEM)
    ss << mem;
  else if (type == IMPLICIT_MEM)
    ss << "imp_mem";
  else
    ss << RegOrMemOrFlag;

  string regInfo = ss.str();

  if (type == EXPLICIT_MEM || type == IMPLICIT_MEM)
    regInfo = "MEM\\{" + regInfo + "\\}\\n";
  else
    regInfo = "\\" + regInfo.substr(0, regInfo.size() - 1) + "\\}\\n";

  os << regInfo;
  if (!dfg)
    return os;

  if (instr_idx == size_t(-1)) {
    os << "#" << -1;
    os << "::E0";
  } else {
    os << "#" << instr_idx;
    const auto &cfg = dfg->getCfg();
    assert(instr_idx < cfg.get_code().size() &&
           "DataValue::print: index OOB!!");
    os << "::" << cfg.get_code()[instr_idx];
  }

  return os;
}

/*
** Class DataFlowValue
*/
size_t DataFlowValue::size() const {
  return DFV.size();
}

const size_t &DataFlowValue::operator[](size_t index) const {
  assert(index < DFV.size() && "Array OOB!!");
  return DFV[index];
}

size_t &DataFlowValue::operator[](size_t index) {
  assert(index < DFV.size() && "Array OOB!!");
  return DFV[index];
}

/** Set union. */
DataFlowValue DataFlowValue::operator|(const DataFlowValue &rhs) const {
  size_t n = size();
  assert(n == rhs.size() && "Size mismatch on operator |\n");
  auto retval = DataFlowValue(n);
  for (size_t i = 0; i < n; i++) {
    retval[i] = DFV[i] | rhs[i];
  }

  return retval;
}

/** Set union. */
DataFlowValue DataFlowValue::operator|=(const DataFlowValue &rhs) {
  size_t n = size();
  assert(n == rhs.size() && "Size mismatch on operator |\n");

  for (size_t i = 0; i < n; i++) {
    DFV[i] |= rhs[i];
  }

  return *this;
}

/** Set intersection. */
DataFlowValue DataFlowValue::operator&(const DataFlowValue &rhs) const {
  size_t n = size();
  assert(n == rhs.size() && "Size mismatch on operator |\n");
  auto retval = DataFlowValue(n);
  for (size_t i = 0; i < n; i++) {
    retval[i] = DFV[i] & rhs[i];
  }

  return retval;
}

/** Set intersection. */
DataFlowValue DataFlowValue::operator&=(const DataFlowValue &rhs) {
  size_t n = size();
  assert(n == rhs.size() && "Size mismatch on operator |\n");

  for (size_t i = 0; i < n; i++) {
    DFV[i] &= rhs[i];
  }

  return *this;
}

/** Set inversion. */
DataFlowValue DataFlowValue::operator~() const {
  size_t n = size();
  auto retval = DataFlowValue(n);
  for (size_t i = 0; i < n; i++) {
    retval[i] = 1 - DFV[i];
  }

  return retval;
}

/** Set difference. */
DataFlowValue DataFlowValue::operator-(const DataFlowValue &rhs) const {
  return (*this) & ~rhs;
}

/** Set equality. */
bool DataFlowValue::operator==(const DataFlowValue &rhs) const {
  size_t n = size();
  assert(n == rhs.size() && "Size mismatch on operator |\n");
  for (size_t i = 0; i < n; i++) {
    if (DFV[i] != rhs[i])
      return false;
  }

  return true;
}

/** Set inequality. */
bool DataFlowValue::operator!=(const DataFlowValue &rhs) const {
  return !(*this == rhs);
}

/** Set containment. */
bool DataFlowValue::contains(const DataFlowValue &rhs) const {
  return (*this & rhs) == rhs;
}

stringstream &DataFlowValue::print(stringstream &os, const Dfg *dfg) const {
  size_t n = DFV.size();
  for (size_t i = 0; i < n; i++) {
    if (DFV[i]) {
      os << "\t";
      dfg->printNode(os, i);
      os << "\n";
    }
  }
  return os;
}

/*
** Class Dfg
*/
void Dfg::recompute_nodes() {

  // Add entry node
  size_t idx = -1;
  RegSet enty_rs = (RegSet::all_gps() | RegSet::all_ymms()) +
                   Constants::eflags_cf() + Constants::eflags_sf() +
                   Constants::eflags_zf() + Constants::eflags_of() +
                   Constants::eflags_pf() + Constants::eflags_af();
  // Add node for a enry gp node
  for (auto gp_it = enty_rs.gp_begin(); gp_it != enty_rs.gp_end(); ++gp_it) {
    DataValue *dfv = new DataValue(idx);
    dfv->addRegOrMem((*gp_it));

    setValueAtIndex(num_nodes, dfv);
    num_nodes++;
  }

  // Add node for a enry sse node
  for (auto sse_it = enty_rs.sse_begin(); sse_it != enty_rs.sse_end();
       ++sse_it) {
    DataValue *dfv = new DataValue(idx);
    dfv->addRegOrMem((*sse_it));

    setValueAtIndex(num_nodes, dfv);
    num_nodes++;
  }

  // Add node for a enry flag node
  for (auto flag_it = enty_rs.flags_begin(); flag_it != enty_rs.flags_end();
       ++flag_it) {
    DataValue *dfv = new DataValue(idx);
    dfv->addFlag((*flag_it));

    setValueAtIndex(num_nodes, dfv);
    num_nodes++;
  }

  // Add node for enry memory
  DataValue *dfv = new DataValue(idx);
  dfv->setType(DataValue::IMPLICIT_MEM);

  setValueAtIndex(num_nodes, dfv);
  num_nodes++;

  num_entry_nodes = num_nodes;

  // Add nodes corresponding to instrcutons
  for (auto i = ++getCfg().reachable_begin(), ie = getCfg().reachable_end();
       i != ie; ++i) {
    for (size_t j = 0, je = getCfg().num_instrs(*i); j < je; ++j) {

      auto idx = getCfg().get_index({*i, j});
      const auto instr = getCfg().get_code()[idx];
      auto must_write = instr.must_write_set();
      auto must_mem_write = instr.must_write_memory();

      // Add node for a dest gp node
      for (auto gp_it = must_write.gp_begin(); gp_it != must_write.gp_end();
           ++gp_it) {
        DataValue *dfv = new DataValue(idx);
        dfv->addRegOrMem((*gp_it));

        setValueAtIndex(num_nodes, dfv);
        num_nodes++;
      }

      // Add node for a dest sse node
      for (auto sse_it = must_write.sse_begin(); sse_it != must_write.sse_end();
           ++sse_it) {
        DataValue *dfv = new DataValue(idx);
        dfv->addRegOrMem((*sse_it));

        setValueAtIndex(num_nodes, dfv);
        num_nodes++;
      }

      // Add node for a dest flag node
      for (auto flag_it = must_write.flags_begin();
           flag_it != must_write.flags_end(); ++flag_it) {
        DataValue *dfv = new DataValue(idx);
        dfv->addFlag((*flag_it));

        setValueAtIndex(num_nodes, dfv);
        num_nodes++;
      }

      // Add edge to memory nodes
      if (!must_mem_write) continue;
      DataValue *dfv = new DataValue(idx);

      if (-1 != instr.mem_index()) {
        auto mem_op = instr.get_operand<M8>(instr.mem_index());
        dfv->addRegOrMem(mem_op);
        //cout << "DSAND" << instr << "\n" << mem_op << "\n" << dfv->getRegOrMemOrFlag() << "\n";
      } else {
        dfv->setType(DataValue::IMPLICIT_MEM);
      }

      setValueAtIndex(num_nodes, dfv);
      num_nodes++;
    }
  }
}

/*
  The data flow information for each instruction is a vector of size
  get_code().size() + 1.
  Each element of that vector hols a set of registers.

  The gen set of an instruction i is per_instr_reaching_defs_in_gen_i][i]

  The kill set of an instruction i is per_instr_reaching_defs_in_gen_i][j],
  where j is all
  the istructions which are killed
  and the content of per_instr_reaching_defs_in_gen_i][j] is the regs if j which
  are
  killed by i
*/
void Dfg::recompute_reaching_defs_in_gen_kill() {

  per_instr_reaching_defs_in_gen_.resize(code_size,
                                         DataFlowValue(num_nodes, 0));
  per_instr_reaching_defs_in_kill_.resize(code_size,
                                          DataFlowValue(num_nodes, 0));

  // Find the gen set for all instructions w.r.t data flow values.
  for (auto i = ++cfg->reachable_begin(), ie = cfg->reachable_end(); i != ie;
       ++i) {
    for (size_t j = 0, je = cfg->num_instrs(*i); j < je; ++j) {
      const auto idx = cfg->get_index({*i, j});
      vector<size_t> instr_data_outs = Instr2Indices[idx];
      for (auto instr_data_out : instr_data_outs) {
        per_instr_reaching_defs_in_gen_[idx][instr_data_out] = 1;
      }
    }
  }

  // Find the kill set for all instructions w.r.t data flow values.
  for (auto i = ++cfg->reachable_begin(), ie = cfg->reachable_end(); i != ie;
       ++i) {
    for (size_t j = 0, je = cfg->num_instrs(*i); j < je; ++j) {
      const auto idx = cfg->get_index({*i, j});
      // Find all the out defs for instr idx
      vector<size_t> instr_data_outs = Instr2Indices[idx];
      for (auto instr_data_out : instr_data_outs) {
        // For each such out defs, find the defined out set
        auto local_kill_set = ValueAtIndex(instr_data_out);

        // find all the data flow indices `local_kill_set` can kill
        for (size_t k = 0; k < num_nodes; k++) {
          auto kill_set = ValueAtIndex(k);
          if (local_kill_set->kills(kill_set)) {
            per_instr_reaching_defs_in_kill_[idx][k] = 1;
          }
        }
      }
    }
  }
}

void Dfg::recompute_reaching_defs_in() {
  recompute_reaching_defs_in_gen_kill();

  per_instr_reaching_defs_in_.resize(code_size, DataFlowValue(num_nodes));
  per_instr_reaching_defs_out_.resize(code_size, DataFlowValue(num_nodes));

  fxn_reaching_defs_ins_ = DataFlowValue(num_nodes, 0, num_entry_nodes);
  //for (size_t i = 0; i < code_size; i++) {
  //  per_instr_reaching_defs_out_[i] = fxn_reaching_defs_ins_;
  //}

  // Iterate until fixed point
  for (auto changed = true; changed;) {
    changed = false;

    for (auto block = ++cfg->reachable_begin(), ie = cfg->reachable_end();
         block != ie; ++block) {
      for (size_t instr = 0, je = cfg->num_instrs(*block); instr < je;
           ++instr) {
        const auto idx = cfg->get_index({*block, instr});

        if (instr == 0) { // First instr of every block
          for (auto p = cfg->pred_begin_instr({*block, instr}),
               pe = cfg->pred_end_instr({*block, instr});
               p != pe; ++p) {
            if ((*p).first == cfg->get_entry()) {
              per_instr_reaching_defs_in_[idx] = fxn_reaching_defs_ins_;
            } else {
              per_instr_reaching_defs_in_[idx] |=
                per_instr_reaching_defs_out_[cfg->get_index(*p)];
            }
          }
        } else {
          per_instr_reaching_defs_in_[idx] |= per_instr_reaching_defs_out_[idx - 1];
        }

        // Transfer function
        const auto new_out =
          (per_instr_reaching_defs_in_[idx] - per_instr_reaching_defs_in_kill_[idx]) |
          per_instr_reaching_defs_in_gen_[idx];

        // Check for fixed point
        changed |= per_instr_reaching_defs_out_[idx] != new_out;
        per_instr_reaching_defs_out_[idx] = new_out;
      }
    }
  }

  // Find the reaching defintions which are actually used at the program points
  per_instr_reaching_and_used_defs_in_.resize(code_size,
      DataFlowValue(num_nodes));
  for (auto i = ++cfg->reachable_begin(), ie = cfg->reachable_end(); i != ie;
       ++i) {
    for (size_t j = 0, je = cfg->num_instrs(*i); j < je; ++j) {
      const auto idx = cfg->get_index({*i, j});
      auto instr = cfg->get_code()[idx];
      auto rd_ins = per_instr_reaching_defs_in_[idx];
      auto read_set = cfg->maybe_read_set(instr);
      bool is_memory_read = instr.must_read_memory();

      for (size_t k = 0; k < rd_ins.size(); k++) {
        if (rd_ins[k] == 0)
          continue;
        auto dfv = ValueAtIndex(k);
        if (dfv->isReg()) {
          // If the reaching def is a regiter or flag
          // & if the current instr has a reg read set,
          // & the reaching def is in the read set.
          // This applies when the curr instr is memory
          if ((read_set & dfv->getRegOrMemOrFlag()) != RegSet::empty()) {
            per_instr_reaching_and_used_defs_in_[idx][k] = 1;
          }
        } else if (dfv->isMemory()) {
          // If the reaching def is a memory
          // & if the current instr has a reg read set,
          //    -- No action
          //
          // If the reaching def is a memory
          // & if the current instr has a mem def
          //    -- include that reaching def
          if (is_memory_read)
            per_instr_reaching_and_used_defs_in_[idx][k] = 1;
        } else {
          assert(0 && "dfg is neither reg or mem!!");
        }
      }
    }
  }

  // Find the per dfg node reaching defintions
  per_dfg_node_reaching_defs_in_.resize(num_nodes,
                                        DataFlowValue(num_nodes));
  for (auto i = ++cfg->reachable_begin(), ie = cfg->reachable_end(); i != ie;
       ++i) {
    for (size_t j = 0, je = cfg->num_instrs(*i); j < je; ++j) {
      const auto idx = cfg->get_index({*i, j});

      // Find the def outs for instr idx
      vector<size_t> instr_data_outs = Instr2Indices[idx];
      // Find the rd and used set of instr idx
      auto rd_used_in_set = per_instr_reaching_and_used_defs_in_[idx];

      for (auto instr_data_out : instr_data_outs) {
        per_dfg_node_reaching_defs_in_[instr_data_out] = rd_used_in_set;
      }
    }
  }



#ifdef DEBUG_CFG_RD
  stringstream ss;
  printDFG(ss);
  cout << ss.str();
#endif
}

void Dfg::serialize(ostream &os) const {}

// Dfg Dfg::deserialize(istream& is) {
//  return Cfg(tunit, defins, liveouts);
//}

} // namespace x64
