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

#include <iostream>

#include "src/ext/cpputil/include/command_line/command_line.h"
#include "src/ext/cpputil/include/signal/debug_handler.h"
#include "src/ext/cpputil/include/io/filterstream.h"
#include "src/ext/cpputil/include/io/column.h"
#include "src/ext/cpputil/include/io/console.h"

#include "src/symstate/pretty_visitor.h"
#include "src/symstate/print_visitor.h"
#include "src/symstate/memory/trivial.h"
#include "src/validator/handlers/combo_handler.h"

#include "tools/gadgets/functions.h"
#include "tools/gadgets/solver.h"
#include "tools/gadgets/target.h"
#include "tools/gadgets/testcases.h"
#include "tools/gadgets/seed.h"

using namespace cpputil;
using namespace std;
using namespace stoke;
using namespace x64asm;

struct CodeReader {
  void operator()(std::istream& is, Code& t) {
    is >> t;
  }
};

struct CodeWriter {
  void operator()(std::ostream& os, const Code& t) {
    os << t;
  }
};

auto& input_header = Heading::create("More Input Formats:");

auto& code_arg = ValueArg<Code, CodeReader, CodeWriter>::create("code")
                 .description("Input code directly");

auto& dbg = Heading::create("Formula Printing Options:");
auto& only_live_out_arg = FlagArg::create("only_live_outs")
                          .description("Only show live out registers");
auto& show_unchanged_arg = FlagArg::create("show_unchanged")
                           .description("Show the formula for unchanged registers");
auto& use_smtlib_format_arg = FlagArg::create("smtlib_format")
                              .description("Show formula in smtlib format");
auto& no_simplify_arg = FlagArg::create("no_simplify")
                        .description("Don't simplify formulas before printing them.");

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

template <typename T>
bool has_changed(T reg, SymBitVector& sym) {
  stringstream ss;
  ss << (*reg);
  if (sym.type() == SymBitVector::Type::VAR) {
    const SymBitVectorVar* const var = static_cast<const SymBitVectorVar* const>(sym.ptr);
    if (var->get_name() == ss.str()) return false;
  }
  return true;
}

template <typename T>
bool has_changed(T reg, SymBool& sym) {
  stringstream ss;
  ss << (*reg);
  if (sym.type() == SymBool::Type::VAR) {
    const SymBoolVar* const var = static_cast<const SymBoolVar* const>(sym.ptr);
    if (var->get_name() == ss.str()) return false;
  }
  return true;
}

int main(int argc, char** argv) {

  // not actually required here
  target_arg.required(false);

  CommandLineConfig::strict_with_convenience(argc, argv);
  DebugHandler::install_sigsegv();
  DebugHandler::install_sigill();

  if (!code_arg.has_been_provided() && !target_arg.has_been_provided()) {
    Console::error() << "Either --code or --target required." << endl;
  }

  if (code_arg.has_been_provided() && target_arg.has_been_provided()) {
    Console::error() << "At most one of --code and --target can be provided." << endl;
  }

  SeedGadget seed;
  TestcaseGadget tc(seed);

  FunctionsGadget aux_fxns;
  TargetGadget target(aux_fxns, false);

  Code code = target.get_code();
  if (code_arg.has_been_provided()) {
    code = code_arg.value();
  }

  Console::msg() << "Target" << endl << endl;
  Console::msg() << code << endl << endl;
  Console::msg() << "  maybe read:      " << code.maybe_read_set() << endl;
  Console::msg() << "  must read:       " << code.must_read_set() << endl;
  Console::msg() << "  maybe write:     " << code.maybe_write_set() << endl;
  Console::msg() << "  must write:      " << code.must_write_set() << endl;
  Console::msg() << "  maybe undef:     " << code.maybe_undef_set() << endl;
  Console::msg() << "  must undef:      " << code.must_undef_set() << endl;
  Console::msg() << "  required flags:  " << code.required_flags() << endl;

  Console::msg() << endl;

  // build initial state
  SymState state;
  TrivialMemory* mem = NULL;
  if (testcases_arg.value().empty()) {
    state = SymState("", true);
    mem = new TrivialMemory();
    state.memory = mem;
  } else {
    state = SymState(tc);
  }

  // test validator support
  ComboHandler ch;
  for (auto it : code) {
    if (it.get_opcode() == Opcode::LABEL_DEFN) continue;
    if (it.get_opcode() == Opcode::RET) break; // only go until first break
    if (ch.get_support(it) == Handler::SupportLevel::NONE) {
      Console::error() << "Instruction unsupported: " << it << endl;
    }
  }

  // compute formula
  size_t line = 0;
  for (auto it : code) {
    if (it.get_opcode() == Opcode::LABEL_DEFN) continue;
    if (it.get_opcode() == Opcode::RET) break;
    state.set_lineno(line);
    ch.build_circuit(it, state);
    line++;
  }

  if (ch.has_error()) {
    Console::error() << "Symbolic execution failed: " << ch.error() << endl;
  }

  Console::msg() << "Formula:" << endl;
  Console::msg() << endl;

  SymPrettyVisitor pretty(Console::msg());
  SymPrintVisitor smtlib(Console::msg());

  auto print = [&smtlib, &pretty](const auto cc) {
    auto c = SymSimplify().simplify(cc);
    if (no_simplify_arg.value()) {
      c = cc;
    }
    if (use_smtlib_format_arg.value()) {
      smtlib((c));
    } else {
      pretty((c));
    }
  };

  // print symbolic state
  bool printed = false;
  RegSet rs = (RegSet::all_gps() | RegSet::all_ymms()) +
              Constants::eflags_cf() +
              Constants::eflags_sf() +
              Constants::eflags_zf() +
              Constants::eflags_of() +
              Constants::eflags_pf() +
              Constants::eflags_af();
  if (only_live_out_arg.value()) {
    rs &= target.live_outs();
  }
  for (auto gp_it = rs.gp_begin(); gp_it != rs.gp_end(); ++gp_it) {
    auto val = state.lookup(*gp_it);
    if (!show_unchanged_arg.value() && !has_changed(gp_it, val)) continue;
    Console::msg() << out_padded(gp_it, 7) << ": ";
    print(val);
    Console::msg() << endl;
    printed = true;
  }
  if (printed) cout << endl;
  printed = false;
  for (auto sse_it = rs.sse_begin(); sse_it != rs.sse_end(); ++sse_it) {
    auto val = state.lookup(*sse_it);
    if (!show_unchanged_arg.value() && !has_changed(sse_it, val)) continue;
    Console::msg() << out_padded(sse_it, 7) << ": ";
    print(val);
    Console::msg() << endl;
    printed = true;
  }
  if (printed) cout << endl;
  printed = false;
  for (auto flag_it = rs.flags_begin(); flag_it != rs.flags_end(); ++flag_it) {
    SymBool val = state[*flag_it];
    if (!show_unchanged_arg.value() && !has_changed(flag_it, val)) continue;
    Console::msg() << out_padded(flag_it, 7) << ": ";
    print(val);
    Console::msg() << endl;
    printed = true;
  }
  if (printed) cout << endl;
  printed = false;

  // print memory reads and writes
  if (mem != NULL) {
    auto reads = mem->get_reads();
    auto writes = mem->get_writes();
    if (reads.size() > 0) {
      printed = true;
      cout << "Information about memory reads:" << endl;
      for (auto loc : reads) {
        cout << "  Value ";
        print(loc.value);
        cout << " (" << loc.size << " bytes)" << endl;
        cout << "    was read at address ";
        print(loc.address);
        cout << "." << endl;
      }
    }
    if (printed) cout << endl;
    printed = false;
    if (writes.size() > 0) {
      printed = true;
      cout << "Information about memory writes:" << endl;
      for (auto loc : writes) {
        cout << "  Address ";
        print(loc.address);
        cout << " was updated to" << endl;
        cout << "    ";
        print(loc.value);
        cout << " (" << loc.size << " bytes)." << endl;
      }
    }
    if (printed) cout << endl;
    printed = false;
  }

  Console::msg() << "sigfpe  : ";
  print(state.sigfpe);
  Console::msg() << endl;
  Console::msg() << "sigbus  : ";
  print(state.sigbus);
  Console::msg() << endl;
  Console::msg() << "sigsegv : ";
  print(state.sigsegv);
  Console::msg() << endl;

  return 0;
}
