// Copyright 2013-2016 Stanford University
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

#include "src/validator/local_class_checker.h"

#include "tools/gadgets/learner.h"
#include "tools/gadgets/obligation_checker.h"
#include "tools/gadgets/seed.h"

using namespace cpputil;
using namespace std;
using namespace stoke;
using namespace x64asm;

auto& io_opt = Heading::create("I/O options:");

auto& filename_arg = ValueArg<string>::create("o")
            .alternate("out")
            .usage("<path/to/file.tc>")
            .description("File to write output to")
            .default_val("");

int main(int argc, char** argv) {

  /** Parse command line arguments. */
  CommandLineConfig::strict_with_convenience(argc, argv);

  ObligationChecker& obligation_checker = *(new ObligationCheckerGadget());

  /** Prepare the callback */
  ClassChecker::Callback callback = [] (ClassChecker::Result& result, void* optional) {
    /** On standard output, write the solution. */
    if(filename_arg.value().size() > 0) {
      ofstream of(filename_arg.value());
      of << result;
      of.close();
    } else {
      cout << result;
    }
    exit(0);
  };

  /** On standard input, read in the problem. */
  ClassChecker::Problem problem = ClassChecker::Problem::deserialize(cin);

  auto& target = problem.template_pod.get_target();
  auto& rewrite = problem.template_pod.get_rewrite();
  SeedGadget seed;
  InvariantLearnerGadget invariant_learner(seed, target, rewrite);

  auto data_collector = problem.data_collector;
  auto control_learner = problem.control_learner;
  LocalClassChecker class_checker(problem.data_collector, problem.control_learner,
                                  problem.target_bound, problem.rewrite_bound,
                                  obligation_checker, invariant_learner);
                                  
  ClassChecker& checker = *static_cast<ClassChecker*>(&class_checker);

  /** Solve the problem and output. */
  checker.check(problem, callback, NULL);
  checker.block_until_complete();

  return 0;
}