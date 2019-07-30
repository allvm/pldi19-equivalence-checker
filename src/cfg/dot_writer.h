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

#ifndef STOKE_SRC_CFG_DOT_WRITER_H
#define STOKE_SRC_CFG_DOT_WRITER_H

#include <iostream>

#include "src/ext/x64asm/include/x64asm.h"

#include "src/cfg/cfg.h"

namespace stoke {

class DotWriter {
public:
  /** Creates a new dot writer. By default, all extended printing is disabled. */
  DotWriter() {
    set_def_in(false, false);
    set_live_out(false);
    set_reaching_defs_in(false);
  }

  /** Toggle whether to display the defined-in relation for blocks and instructions. */
  DotWriter& set_def_in(bool block, bool instr) {
    def_in_block_ = block;
    def_in_instr_ = instr;
    return *this;
  }
  /** Toggle whether to display the live-out relation for blocks. */
  DotWriter& set_live_out(bool block) {
    live_out_block_ = block;
    return *this;
  }

  /** Toggle whether to display the reaching-defs-in relation for instructions. */
  DotWriter& set_reaching_defs_in(bool flag) {
    reaching_defs_in_instr_ = flag;
    return *this;
  }

  /** Toggle whether to display the reaching-defs-in relation for instructions. */
  DotWriter& set_dfg(bool flag) {
    dfg_ = flag;
    return *this;
  }



  /** Emits a control flow graph in .dot format. */
  void operator()(std::ostream& os, const Cfg& cfg) const {
    os << "digraph g {" << std::endl;
    os << "colorscheme = blues6" << std::endl;

    if (dfg_) {
      plot_dfg(os, cfg);
    } else {
      write_entry(os, cfg);
      write_exit(os, cfg);
      write_blocks(os, cfg);
      write_edges(os, cfg);

    }
    os << "}";
  }

private:
  /** Write the entry block for this graph. */
  void write_entry(std::ostream& os, const Cfg& cfg) const;
  /** Write the exit block for this graph. */
  void write_exit(std::ostream& os, const Cfg& cfg) const;
  /** Write a block. */
  void write_block(std::ostream& os, const Cfg& cfg, Cfg::id_type id) const;
  /** Write the basic blocks in this graph. */
  void write_blocks(std::ostream& os, const Cfg& cfg) const;
  /** Write the edges in this graph. */
  void write_edges(std::ostream& os, const Cfg& cfg) const;
  /** Write the contents of a register set. */
  void write_reg_set(std::ostream& os, const x64asm::RegSet& rs) const;
  void write_reaching_def(std::ostream& os, const Dfv_RD& rs) const;
  void plot_dfg(std::ostream& os, const Cfg& cfg) const;

  /** Write the defined-in relation for blocks? */
  bool def_in_block_;
  /** Write the defined-in relation for instructions? */
  bool def_in_instr_;
  /** Write the live-out relation for blocks? */
  bool live_out_block_;
  /** Write the reaching defs-in relation for instructions */
  bool reaching_defs_in_instr_;
  /** Write the data flow graph */
  bool dfg_;
};

} // namespace stoke

#endif
