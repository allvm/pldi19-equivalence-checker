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

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "src/ext/cpputil/include/command_line/command_line.h"
#include "src/ext/cpputil/include/io/console.h"
#include "src/ext/cpputil/include/signal/debug_handler.h"
#include "src/ext/cpputil/include/system/terminal.h"
#include "src/ext/x64asm/include/x64asm.h"
#include "src/symstate/memory/trivial.h"
#include "src/symstate/pretty_visitor.h"
#include "src/symstate/print_visitor.h"
#include "src/symstate/search_visitor.h"
#include "src/symstate/simplify.h"
#include "src/validator/handlers/combo_handler.h"

#include "tools/gadgets/functions.h"
#include "tools/gadgets/target.h"

using namespace cpputil;
using namespace std;
using namespace stoke;
using namespace x64asm;

auto &content = Heading::create("Content Options:");
auto &use_smtlib_format_arg = FlagArg::create("smtlib_format")
                              .description("Show formula in smtlib format");
cpputil::ValueArg<std::string> &strata_path_arg =
  cpputil::ValueArg<std::string>::create("strata_path")
  .usage("<path/to/dir>")
  .description("The path to the directory with the strata circuits (a "
               "collection of .s files)")
  .default_val("");

template <typename T>
string out_padded(T t, size_t min_length, char pad = ' ') {
  stringstream ss;
  ss << (*t);
  size_t len = ss.str().size();
  for (size_t i = 0; i < (min_length - len); i++) {
    ss << pad;
  }
  return ss.str();
}

/** Returns bit pattern consisting of 0s and ending with 'ones' many 1s. */
uint64_t mask(uint16_t ones) {
  if (ones == 0)
    return 0;
  if (ones == 64)
    return -1;
  return (1ULL << ones) - 1;
}

uint64_t read_const(const SymBitVectorAbstract *const s) {
  auto c = (SymBitVectorConstant *)s;
  return c->constant_ & mask(c->size_);
}

bool is_zero(const SymBitVectorAbstract *const b) {
  return (b->type() == SymBitVector::CONSTANT) && read_const(b) == 0;
}

bool is_one(const SymBitVectorAbstract *const b) {
  return (b->type() == SymBitVector::CONSTANT) && read_const(b) == 1;
}

template <typename T> bool has_changed(T reg, SymBitVector &sym) {
  stringstream ss;
  ss << (*reg);
  if (sym.type() == SymBitVector::Type::VAR) {
    const SymBitVectorVar *const var =
      static_cast<const SymBitVectorVar *const>(sym.ptr);
    if (var->get_name() == ss.str())
      return false;
  }

  if (sym.type() == SymBitVector::Type::ITE) {
    const SymBitVectorIte *const ite =
      static_cast<const SymBitVectorIte *const>(sym.ptr);
    const SymBoolAbstract *const cond = ite->cond_;
    auto var = dynamic_cast<const SymBoolVar *const>(cond);
    if (NULL != var && is_one(ite->a_) && is_zero(ite->b_)) {
      // if ((var->get_name() == ss.str()) || (std::string::npos !=
      // ss.str().find("TMP_BOOL"))) {
      if ((var->get_name() == ss.str())) {
        return false;
      }
    }
  }

  return true;
}

template <typename T> bool has_changed(T reg, SymBool &sym) {
  stringstream ss;
  ss << (*reg);
  if (sym.type() == SymBool::Type::VAR) {
    const SymBoolVar *const var = static_cast<const SymBoolVar *const>(sym.ptr);
    if (var->get_name() == ss.str())
      return false;
  }
  return true;
}

SymPrettyVisitor pretty(Console::msg());
SymPrintVisitor smtlib(Console::msg());
SymSearchVisitor syms_search(Console::msg());

auto print = [&smtlib, &pretty](const auto cc) {
  auto c = SymSimplify().simplify(cc);
  if (use_smtlib_format_arg.value()) {
    smtlib((c));
  } else {
    pretty((c));
  }
};

void register_info(Instruction &instr, SymState &state) {

  auto must_write = instr.must_write_set();
  auto may_write = instr.maybe_write_set();
  Console::msg() << "\twrite-set: \n";
  Console::msg() << "\t\tMust: [" << must_write << "]\n";
  Console::msg() << "\t\tMay: [" << may_write << "]\n";

  auto must_read = instr.must_read_set();
  auto may_read = instr.maybe_read_set();
  Console::msg() << "\tread-set: \n";
  Console::msg() << "\t\tMust: [" << must_read << "]\n";
  Console::msg() << "\t\tMay: [" << may_read << "]\n";

  auto must_undef = instr.must_undef_set();
  auto may_undef = instr.maybe_undef_set();
  Console::msg() << "\tundef-set: \n";
  Console::msg() << "\t\tMust: [" << must_undef << "]\n";
  Console::msg() << "\t\tMay: [" << may_undef << "]\n";

  Console::msg() << "Formula:" << endl;
  Console::msg() << endl;

  bool printed = false;
  RegSet rs = must_write;

  for (auto gp_it = rs.gp_begin(); gp_it != rs.gp_end(); ++gp_it) {
    auto val = state.lookup(*gp_it);
    if (!has_changed(gp_it, val))
      continue;
    Console::msg() << out_padded(gp_it, 7) << ": ";
    print(val);
    Console::msg() << endl;

    auto actual_read_set = syms_search((val));
    Console::msg() << "\nActual read-set: " << actual_read_set;

    Console::msg() << endl;
    printed = true;
  }

  if (printed)
    cout << endl;
  printed = false;

  for (auto sse_it = rs.sse_begin(); sse_it != rs.sse_end(); ++sse_it) {
    auto val = state.lookup(*sse_it);
    if (!has_changed(sse_it, val))
      continue;
    Console::msg() << out_padded(sse_it, 7) << ": ";
    print(val);
    Console::msg() << endl;

    auto actual_read_set = syms_search((val));
    Console::msg() << "\nActual read-set: " << actual_read_set;

    Console::msg() << endl;
    printed = true;
  }

  if (printed)
    cout << endl;
  printed = false;

  for (auto flag_it = rs.flags_begin(); flag_it != rs.flags_end(); ++flag_it) {
    SymBool val = state[*flag_it];
    if (!has_changed(flag_it, val))
      continue;
    Console::msg() << out_padded(flag_it, 7) << ": ";
    print(val);
    Console::msg() << endl;

    auto actual_read_set = syms_search((val));
    Console::msg() << "\nActual ead Set: " << actual_read_set;

    Console::msg() << endl;
    printed = true;
  }
  Console::msg() << "\n\n";
}

void memory_info(Instruction &instr, SymState &state, TrivialMemory *mem) {

  auto must_mem_write = instr.must_write_memory();
  auto may_mem_write = instr.maybe_write_memory();
  Console::msg() << "\tmem-write-set: \n";
  Console::msg() << "\t\tMust: [" << must_mem_write << "]\n";
  Console::msg() << "\t\tMay: [" << may_mem_write << "]\n";

  auto must_mem_read = instr.must_read_memory();
  auto may_mem_read = instr.maybe_read_memory();
  Console::msg() << "\tmem-read-set: \n";
  Console::msg() << "\t\tMust: [" << must_mem_read << "]\n";
  Console::msg() << "\t\tMay: [" << may_mem_read << "]\n";

  // Check if there is a memory reference
  // bool check_mem_writes = false;
  // if (-1 != instr.mem_index()) {
  //  Console::msg() << "Memory accesses:" << endl;

  //  auto mem_op = instr.get_operand<M8>(instr.mem_index());
  //  Console::msg() << mem_op;
  //  Console::msg() << state.lookup(mem_op);
  //}

  auto reads = mem->get_reads();
  auto writes = mem->get_writes();
  bool assertCheck =
    ((must_mem_write && writes.size()) ||
     (!must_mem_write && !writes.size())) &&
    ((must_mem_read && reads.size()) || (!must_mem_read && !reads.size()));
  assert(assertCheck && "Mem assert check failed!!");

  bool printed = false;
  if (reads.size() > 0) {
    printed = true;
    cout << "Information about memory reads:" << endl;
    size_t sZ;
    SymBitVector sA;

    for (auto loc : reads) {
      cout << "  Value ";
      print(loc.value);
      cout << " (" << loc.size << " bytes)" << endl;
      cout << "    was read from address ";
      print(loc.address);
      cout << "." << endl;

      auto actual_read_set = syms_search((loc.value));
      actual_read_set |= syms_search((loc.address));
      Console::msg() << "\nActual read-set: " << actual_read_set;
    }
  }

  if (printed)
    cout << endl;
  printed = false;

  if (writes.size() > 0) {
    printed = true;
    cout << "Information about memory writes:" << endl;
    bool firstInstance = true;
    size_t sZ;
    SymBitVector sA;

    for (auto loc : writes) {
      cout << "  Value at Address ";
      print(loc.address);
      cout << " updated to" << endl;
      cout << "    Value ";
      print(loc.value);
      cout << " (" << loc.size << " bytes)." << endl;

      auto actual_read_set = syms_search((loc.value));
      actual_read_set |= syms_search((loc.address));
      Console::msg() << "\nActual read-set: " << actual_read_set;
    }
  }

  if (printed)
    cout << endl;
  Console::msg() << "\n\n";
}

void debug_instruction(Instruction &instr) {

  if (instr.get_opcode() == Opcode::LABEL_DEFN ||
      instr.get_opcode() == Opcode::RET || instr.is_any_jump())
    return;

  cout << instr << "\n";

  ComboHandler ch;
  SymState state = SymState("", true);
  TrivialMemory *mem = new TrivialMemory();
  state.memory = mem;

  if (ch.get_support(instr) == Handler::SupportLevel::NONE) {
    Console::error() << "Instruction unsupported: " << instr << endl;
    return;
  }

  ch.build_circuit(instr, state);
  if (ch.has_error()) {
    Console::error() << "Symbolic execution failed: " << ch.error() << endl;
    return;
  }

  register_info(instr, state);
  memory_info(instr, state, mem);
}

int main(int argc, char **argv) {
  CommandLineConfig::strict_with_convenience(argc, argv);
  DebugHandler::install_sigsegv();
  DebugHandler::install_sigill();
  target_arg.required(true);

  // Reading the target code
  Code code = target_arg.value().get_code();
  for (size_t i = 0; i < code.size(); i++) {
    debug_instruction(code[i]);
  }

  return 0;
}
