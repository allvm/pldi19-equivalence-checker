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

// void DotWriter::plot_dfg(ostream& os, const Cfg& cfg) const {
//  // Add nodes
//  for (auto i = ++cfg.reachable_begin(), ie = cfg.reachable_end(); i != ie;
//  ++i) {
//    for (size_t j = 0, je = cfg.num_instrs(*i); j < je; ++j) {
//      const auto idx = cfg.get_index({*i, j});
//
//      os << "I" << dec << idx << " [";
//      os << "shape=record  ";
//      os << "label=\"{#" << idx << ":" << cfg.get_code()[idx];
//      os << "}\"];" << endl;
//    }
//  }
//
//  // Add edges
//  for (auto i = ++cfg.reachable_begin(), ie = cfg.reachable_end(); i != ie;
//  ++i) {
//    for (size_t j = 0, je = cfg.num_instrs(*i); j < je; ++j) {
//      const auto idx = cfg.get_index({*i, j});
//      auto rd_ins = cfg.reaching_and_used_defs_in({*i, j});
//
//      for (size_t k = 0 ; k < rd_ins.size(); k++) {
//        if (rd_ins[k] == RegSet::empty()) {
//          continue;
//        }
//
//        if(k == 0)
//          os << "I" << dec << 99 << "->I" << dec << idx << " [";
//        else
//          os << "I" << dec << k - 1 << "->I" << dec << idx << " [";
//        os << "style=bold";
//        os << " color=";
//        os << "black";
//
//        stringstream ss;
//        ss << rd_ins[k];
//        string s = ss.str();
//        os << " label=\"" << "\\" << s.substr(0, s.size() - 1) << "\\}\"";
//        os << "];" << endl;
//
//      }
//    }
//  }
//}

static void plot_node(ostream &os, const Cfg &cfg, KeyCache &cache, std::string str, size_t idx) {
  auto p = cache.getKey(str, idx);
  auto nodeid = p.first;
  os << nodeid << " [";
  os << "shape=record  ";
  if (idx == (size_t)-1) {
    os << "label=\"{" << str << " #" << idx << ":E0";
  } else {
    os << "label=\"{" << str << " #" << idx << ":" << cfg.get_code()[idx];
  }
  os << "}\"];" << endl;
}


static void plot_edge(ostream &os, const Cfg &cfg, KeyCache &cache,
                      std::string src_str, size_t src_idx,
                      std::string dest_str, size_t dest_idx) {
  auto dest_p = cache.getKey(dest_str, dest_idx);
  auto src_p = cache.getKey(src_str, src_idx);
  auto dest_nodeid = dest_p.first;
  auto src_nodeid = src_p.first;

  if (!dest_p.second) {
    std::cout << dest_str << ": " << cfg.get_code()[dest_idx] << endl;
    assert(dest_p.second == true && "Destination node missing!!");
  }

  if (!src_p.second) {
    if (src_idx == (size_t)-1) {
      plot_node(os, cfg, cache, src_str, src_idx);
    } else {
      std::cout << src_str << ": " << cfg.get_code()[src_idx] << endl;
      assert(src_p.second == true && "Unknown Node!!");
    }
  }

  os << src_nodeid << "->" << dest_nodeid << " [";
  os << "style=bold";
  os << " color=";
  os << "black];" << endl;
}

void DotWriter::plot_dfg_edge(ostream &os, const Cfg &cfg, KeyCache &cache) const {
  for (auto i = ++cfg.reachable_begin(), ie = cfg.reachable_end(); i != ie;
       ++i) {
    for (size_t j = 0, je = cfg.num_instrs(*i); j < je; ++j) {

      const auto idx = cfg.get_index({*i, j});
      const auto instr = cfg.get_code()[idx];
      auto rd_ins = cfg.reaching_and_used_defs_in({*i, j});
      auto must_write = instr.must_write_set();
#ifdef DEBUG_CFG_RD
      std::cout << idx << ":" << instr << "\n";
      std::cout << "\treaching_defs_in_used_: \n";
      for (size_t i = 0 ; i < rd_ins.size(); i++) {
        if (rd_ins[i] != RegSet::empty()) {
          std::cout << "\t\t[\n";
          if (i == 0)
            std::cout << "\t\t\t" << "E0" << "\n";
          else
            std::cout << "\t\t\t" << cfg.get_code()[i-1] << "\n";

          std::cout << "\t\t\t\t" << rd_ins[i] << "\n";
          std::cout << "\t\t]\n\n";
        }
      }
#endif

      // Add edge to a dest gp node
      for (auto gp_it_dest = must_write.gp_begin();
           gp_it_dest != must_write.gp_end(); ++gp_it_dest) {
        std::stringstream ss_dest;
        ss_dest << (*gp_it_dest);

        for (size_t k = 0; k < rd_ins.size(); k++) {
          auto rd_ins_reg_set = rd_ins[k];
          if (rd_ins_reg_set == RegSet::empty())
            continue;

          for (auto gp_it_src = rd_ins_reg_set.gp_begin();
               gp_it_src != rd_ins_reg_set.gp_end(); ++gp_it_src) {
            std::stringstream ss_src;
            ss_src << (*gp_it_src);
            plot_edge(os, cfg, cache, ss_src.str(), k-1, ss_dest.str(),
                      idx);
          }

          for (auto sse_it_src = rd_ins_reg_set.sse_begin();
               sse_it_src != rd_ins_reg_set.sse_end(); ++sse_it_src) {
            std::stringstream ss_src;
            ss_src << (*sse_it_src);
            plot_edge(os, cfg, cache, ss_src.str(), k-1, ss_dest.str(),
                      idx);
          }

          for (auto flag_it_src = rd_ins_reg_set.flags_begin();
               flag_it_src != rd_ins_reg_set.flags_end(); ++flag_it_src) {
            std::stringstream ss_src;
            ss_src << (*flag_it_src);
            plot_edge(os, cfg, cache, ss_src.str(), k-1, ss_dest.str(),
                      idx);
          }
        }
      }

      // Add edge to a dest sse node
      for (auto sse_it_dest = must_write.sse_begin();
           sse_it_dest != must_write.sse_end(); ++sse_it_dest) {
        std::stringstream ss_dest;
        ss_dest << (*sse_it_dest);

        for (size_t k = 0; k < rd_ins.size(); k++) {
          auto rd_ins_reg_set = rd_ins[k];
          if (rd_ins_reg_set == RegSet::empty())
            continue;

          for (auto gp_it_src = rd_ins_reg_set.gp_begin();
               gp_it_src != rd_ins_reg_set.gp_end(); ++gp_it_src) {
            std::stringstream ss_src;
            ss_src << (*gp_it_src);
            plot_edge(os, cfg, cache, ss_src.str(), k-1, ss_dest.str(),
                      idx);
          }

          for (auto sse_it_src = rd_ins_reg_set.sse_begin();
               sse_it_src != rd_ins_reg_set.sse_end(); ++sse_it_src) {
            std::stringstream ss_src;
            ss_src << (*sse_it_src);
            plot_edge(os, cfg, cache, ss_src.str(), k-1, ss_dest.str(),
                      idx);
          }

          for (auto flag_it_src = rd_ins_reg_set.flags_begin();
               flag_it_src != rd_ins_reg_set.flags_end(); ++flag_it_src) {
            std::stringstream ss_src;
            ss_src << (*flag_it_src);
            plot_edge(os, cfg, cache, ss_src.str(), k-1, ss_dest.str(),
                      idx);
          }
        }
      }

      // Add edge to a dest flag node
      for (auto flag_it_dest = must_write.flags_begin();
           flag_it_dest != must_write.flags_end(); ++flag_it_dest) {
        std::stringstream ss_dest;
        ss_dest << (*flag_it_dest);

        for (size_t k = 0; k < rd_ins.size(); k++) {
          auto rd_ins_reg_set = rd_ins[k];
          if (rd_ins_reg_set == RegSet::empty())
            continue;

          for (auto gp_it_src = rd_ins_reg_set.gp_begin();
               gp_it_src != rd_ins_reg_set.gp_end(); ++gp_it_src) {
            std::stringstream ss_src;
            ss_src << (*gp_it_src);
            plot_edge(os, cfg, cache, ss_src.str(), k-1, ss_dest.str(),
                      idx);
          }

          for (auto sse_it_src = rd_ins_reg_set.sse_begin();
               sse_it_src != rd_ins_reg_set.sse_end(); ++sse_it_src) {
            std::stringstream ss_src;
            ss_src << (*sse_it_src);
            plot_edge(os, cfg, cache, ss_src.str(), k-1, ss_dest.str(),
                      idx);
          }

          for (auto flag_it_src = rd_ins_reg_set.flags_begin();
               flag_it_src != rd_ins_reg_set.flags_end(); ++flag_it_src) {
            std::stringstream ss_src;
            ss_src << (*flag_it_src);
            plot_edge(os, cfg, cache, ss_src.str(), k-1, ss_dest.str(),
                      idx);
          }
        }
      }

      // Add edge to memory nodes
    }
  }
}

void DotWriter::plot_dfg_node(ostream &os, const Cfg &cfg, KeyCache &cache) const {

  for (auto i = ++cfg.reachable_begin(), ie = cfg.reachable_end(); i != ie;
       ++i) {
    for (size_t j = 0, je = cfg.num_instrs(*i); j < je; ++j) {

      auto idx = cfg.get_index({*i, j});
      const auto instr = cfg.get_code()[idx];
      std::cout << idx <<": " << instr << "\n";
      auto rd_ins = cfg.reaching_and_used_defs_in({*i, j});
      auto must_write = instr.must_write_set();

      // Add node for a dest gp node
      for (auto gp_it_dest = must_write.gp_begin();
           gp_it_dest != must_write.gp_end(); ++gp_it_dest) {
        std::stringstream ss_dest;
        ss_dest << (*gp_it_dest);

        plot_node(os, cfg, cache, ss_dest.str(), idx);
      }

      // Add node for a dest sse node
      for (auto sse_it_dest = must_write.sse_begin();
           sse_it_dest != must_write.sse_end(); ++sse_it_dest) {
        std::stringstream ss_dest;
        ss_dest << (*sse_it_dest);

        plot_node(os, cfg, cache, ss_dest.str(), idx);
      }

      // Add node for a dest flag node
      for (auto flag_it_dest = must_write.flags_begin();
           flag_it_dest != must_write.flags_end(); ++flag_it_dest) {
        std::stringstream ss_dest;
        ss_dest << (*flag_it_dest);

        plot_node(os, cfg, cache, ss_dest.str(), idx);
      }

      // Add edge to memory nodes
    }
  }

}


void DotWriter::plot_dfg(ostream &os, const Cfg &cfg) const {
  KeyCache cache;
  plot_dfg_node(os, cfg, cache);
  plot_dfg_edge(os, cfg, cache);
}
} // namespace stoke
