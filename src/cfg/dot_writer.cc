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

#include <sstream>

#include "src/cfg/dot_writer.h"
#define DEBUG_CFG_RD

using namespace std;
using namespace x64asm;

namespace stoke {

void DotWriter::write_entry(ostream &os, const Cfg &cfg) const {
  os << "bb" << dec << cfg.get_entry() << " [";
  os << "shape=record  ";
  os << "label=\"{ENTRY";
  if (def_in_block_) {
    os << "|def-in: ";
    write_reg_set(os, cfg.def_ins());
  }
  if (reaching_defs_in_instr_) {
    os << "|reaching-def-in: \\n";
    write_reaching_def(os, cfg.reaching_defs_in());
  }

  os << "}\"];" << endl;
}

void DotWriter::write_exit(ostream &os, const Cfg &cfg) const {
  const auto id = cfg.get_exit();
  os << "bb" << dec << id << " [";
  os << "shape=record ";
  os << "label=\"{EXIT";
  if (def_in_block_) {
    os << "|def-in: ";
    write_reg_set(os, cfg.def_outs());
  }
  if (live_out_block_) {
    os << "|live-out: ";
    write_reg_set(os, cfg.live_outs());
  }
  os << "}\"];" << endl;
}

void DotWriter::write_block(ostream &os, const Cfg &cfg,
                            Cfg::id_type id) const {
  os << "bb" << dec << id << "[";
  os << "shape=record, style=filled, fillcolor=white, ";
  if (!cfg.is_reachable(id)) {
    os << "color = grey, ";
  }
  os << "label=\"{";
  os << "#" << id;
  if (def_in_block_ && cfg.is_reachable(id)) {
    os << "|def-in: ";
    write_reg_set(os, cfg.def_ins(id));
  }
  if (reaching_defs_in_instr_ && cfg.is_reachable(id)) {
    os << "|reaching-def-in: \\n";
    write_reaching_def(os, cfg.reaching_defs_in(id));
  }

  for (size_t j = 0, je = cfg.num_instrs(id); j < je; ++j) {
    if (def_in_instr_ && cfg.is_reachable(id)) {
      os << "|def-in: ";
      write_reg_set(os, cfg.def_ins({id, j}));
    }

    if (reaching_defs_in_instr_ && cfg.is_reachable(id)) {
      os << "|reaching-def-in: \\n";
      write_reaching_def(os, cfg.reaching_defs_in({id, j}));
    }

    auto instr = cfg.get_instr({id, j});
    if (!instr.is_nop()) {
      os << "|";
      os << "#" << j << " " << instr;
      os << "\\l";
    }
  }
  os << "}\"];" << endl;
}

void DotWriter::write_blocks(ostream &os, const Cfg &cfg) const {
  map<size_t, vector<Cfg::id_type>> nestings;
  for (size_t i = cfg.get_entry() + 1, ie = cfg.get_exit(); i < ie; ++i) {
    nestings[0].push_back(i);
  }

  for (const auto &n : nestings) {
    os << "subgraph cluster_" << n.first << " {" << endl;
    os << "style = filled" << endl;
    os << "color = " << (n.first + 1) << endl;

    for (const auto id : n.second) {
      write_block(os, cfg, id);
    }
  }

  for (size_t i = 0, ie = nestings.size(); i < ie; ++i) {
    os << "}" << endl;
  }
}

void DotWriter::write_edges(ostream &os, const Cfg &cfg) const {
  for (size_t i = cfg.get_entry(), ie = cfg.get_exit(); i < ie; ++i)
    for (auto s = cfg.succ_begin(i), se = cfg.succ_end(i); s != se; ++s) {
      os << "bb" << dec << i << "->bb" << dec << *s << " [";
      os << "style=";
      if (cfg.has_fallthrough_target(i) && cfg.fallthrough_target(i) == *s) {
        os << "bold";
      } else {
        os << "dashed";
      }
      os << " color=";
      if (cfg.is_reachable(i) || cfg.is_entry(i)) {
        os << "black";
      } else {
        os << "grey";
      }
      os << "];" << endl;
    }
}

void DotWriter::write_reg_set(ostream &os, const RegSet &rs) const {
  stringstream ss;
  ss << rs;
  string s = ss.str();
  os << "\\" << s.substr(0, s.size() - 1) << "\\}";
}

void DotWriter::write_reaching_def(ostream &os, const Dfv_RD &rs) const {
  // stringstream ss;

  for (size_t i = 0; i < rs.size(); i++) {
    if (rs[i] == RegSet::empty()) {
      continue;
    }
    stringstream ss;
    ss << rs[i];
    string s = ss.str();
    os << i << ": "
       << "\\" << s.substr(0, s.size() - 1) << "\\}\\n";
  }
}


static string plot_node(ostream &os, const Cfg &cfg, KeyCache &cache, std::string str) {
  auto p = cache.getKey(str);
  auto nodeid = p.first;
  os << nodeid << " [";
  os << "shape=record  ";
  os << "label=\"{" << str << "}\"];" << endl;
  return nodeid;
}

void DotWriter::write_dfg(ostream &os, const Dfg &dfg) const {
  KeyCache cache;
  for (size_t i = 0 ; i < dfg.num_nodes ; i++) {
    std::stringstream ss_dest;
    dfg.printNode(ss_dest, i);

    auto def_ins = dfg.per_dfg_node_reaching_defs_in_[i];
    for (size_t j = 0 ; j < dfg.num_nodes ; j++) {
      if (def_ins[j]) {
        std::stringstream ss_src;
        dfg.printNode(ss_src, j);

        auto dest_id = plot_node(os, dfg.getCfg(), cache, ss_dest.str());
        auto src_id = plot_node(os, dfg.getCfg(), cache, ss_src.str());
        os << src_id << "->" << dest_id << " [";
        os << "style=bold";
        os << " color=";
        os << "black];" << endl;
      }
    }
  }
}
} // namespace stoke
