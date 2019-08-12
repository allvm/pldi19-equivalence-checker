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

#include "src/cfg/dot_writer.h"
#include "src/cfg/dfg.h"
#include "tools/gadgets/functions.h"
#include "tools/gadgets/target.h"

using namespace cpputil;
using namespace std;
using namespace stoke;

auto& content = Heading::create("Content Options:");
auto& rd = FlagArg::create("rd")
           .alternate("reaching_defs_in")
           .description("Display reaching defintions in values for instructions");
auto& dfg = FlagArg::create("dfg")
            .alternate("data_flow_graph")
            .description("Display data flow graph");

auto& io = Heading::create("I/O Options:");
auto& dot_out = ValueArg<string>::create("do")
                .alternate("dot-out")
                .usage("<path/to/file.dot>")
                .description("Path to write dfg to")
                .default_val("");
auto& out = ValueArg<string>::create("o")
            .alternate("out")
            .usage("<path/to/file.pdf>")
            .description("Path to write cfg to")
            .default_val("./cfg.pdf");
auto& view = FlagArg::create("view")
             .alternate("v")
             .description("View cfg immediately");
auto& no_skip_checks = FlagArg::create("no_skip_checks")
                       .description("Run all the checks for creating a Cfg; don't skip.");



string tempfile(const string& temp) {
  vector<char> v(temp.begin(), temp.end());
  v.push_back('\0');

  const auto fd = mkstemp(v.data());
  return string(v.begin(), v.end()-1);
}

void to_dot(const string& dot_file) {
  ofstream ofs(dot_file);

  Dfg* target;
  Cfg* cfg;

  if (no_skip_checks)  {
    FunctionsGadget aux_fxns;
    cfg = static_cast<Cfg*>(new TargetGadget(aux_fxns, false));
  } {
    cfg = new Cfg(target_arg.value().get_code());
  }

  target = new Dfg(cfg);

  DotWriter dw;
  dw.set_dfg(dfg);
  dw(ofs, *target);
}

bool to_pdf(const string& dot_file, const string& pdf_file) {
  Terminal term;
  term << "cat " << dot_file << " | dot -Tpdf > " << pdf_file << " 2> /dev/null" << endl;
  return term.result() == 0;
}

bool view_pdf(const string& pdf_file) {
  Terminal term;
  term << "evince " << pdf_file << endl;
  return term.result() == 0;
}

int main(int argc, char** argv) {
  CommandLineConfig::strict_with_convenience(argc, argv);
  DebugHandler::install_sigsegv();
  DebugHandler::install_sigill();

  auto dot_file = dot_out.value();
  if (dot_out.value() == "")
    dot_file = tempfile("/tmp/stoke_debug_cfg.dot.XXXXXX");

  to_dot(dot_file);
  if (!to_pdf(dot_file, out.value())) {
    Console::error(1) << "Unable to save file!" << endl;
  } else if (view && !view_pdf(out.value())) {
    Console::error(1) << "Unable to open file for viewing!" << endl;
  }

  return 0;
}

