// Copyrhs 2013-2019 Stanford University
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

#ifndef STOKE_SRC_CFG_DFG_H
#define STOKE_SRC_CFG_DFG_H

#include <cassert>
#include <stdint.h>

#include <map>
#include <sstream>
#include <stack>
#include <unordered_map>
#include <vector>

#include "src/tunit/tunit.h"

#include "src/cfg/cfg.h"
#include "src/ext/cpputil/include/container/bit_vector.h"
#include "src/ext/cpputil/include/container/maputil.h"
#include "src/ext/x64asm/include/x64asm.h"

using namespace std;
using namespace x64asm;

namespace stoke {
class Dfg;

class DataValue {
public:
  enum DataValueType {
    UINIT = 0, // Uninitialized
    REG = 1,   // Reg
    EFLAG = 2, // Flag
    EXPLICIT_MEM = 3,   // Memory
    IMPLICIT_MEM = 4,   // Memory
    TOTAL = 5,
  };
private:
  RegSet RegOrMemOrFlag;
  int type;
  size_t instr_idx;
  Mem mem;

public:

  DataValue(size_t idx, Mem mem_ = M8(Constants::rax())) : instr_idx(idx), mem(mem_) {
    type = UINIT;
    RegOrMemOrFlag = RegSet::empty();
  }
  bool isRegOrMem() const;
  bool isReg() const;
  bool isFlag() const;
  bool isMemory() const;
  //Operand getReg() const;
  //Eflags getFlag() const;
  size_t getInstrIndex() const;
  RegSet getRegOrMemOrFlag() const;
  void addFlag(const Eflags &rhs);
  bool kills(DataValue *target);
  void addRegOrMem(Operand rhs);
  void setType(size_t t) {
    assert(t > 0 && t < TOTAL && "Type not a MEM or FLAG or REG!!");
    type = t;
  }
  stringstream &print(stringstream &os, const Dfg *dfg) const;
};

class DataFlowValue {
private:
  std::vector<size_t> DFV;

public:
  DataFlowValue(size_t num_nodes, size_t val = 0, size_t num_entry_nodes = 0) {
    DFV.resize(num_nodes, val);

    for (size_t i = 0; i < num_entry_nodes; i++)
      DFV[i] = 1;
  }

  size_t size() const;
  const size_t &operator[](size_t index) const;
  size_t &operator[](size_t index);
  /** Set union. */
  DataFlowValue operator|(const DataFlowValue &rhs) const;
  /** Set union. */
  DataFlowValue operator|=(const DataFlowValue &rhs);
  /** Set intersection. */
  DataFlowValue operator&(const DataFlowValue &rhs) const;
  /** Set intersection. */
  DataFlowValue operator&=(const DataFlowValue &rhs);
  /** Set inversion. */
  DataFlowValue operator~() const;
  /** Set difference. */
  DataFlowValue operator-(const DataFlowValue &rhs) const;
  /** Set equality. */
  bool operator==(const DataFlowValue &rhs) const;
  /** Set inequality. */
  bool operator!=(const DataFlowValue &rhs) const;
  /** Set containment. */
  bool contains(const DataFlowValue &rhs) const;
  stringstream &print(stringstream &os, const Dfg *dfg) const;
};

class Dfg {
  friend class DataFlowValue;
  friend class DataValue;

public:
  /** Node id type. */
  typedef size_t id_type;
  /** Edge type; points from source to target. */
  typedef std::pair<id_type, id_type> edge_type;
  /** Number of nodes */
  size_t num_nodes;
  /** */
  size_t num_entry_nodes;

  size_t code_size;
  /** Edges */
  map<id_type, vector<id_type>> in_edges;

private:
  const Cfg *cfg;
  std::map<DataValue *, size_t> Value2Index;
  std::map<size_t, DataValue *> Index2Value;
  std::map<size_t, vector<size_t>> Instr2Indices;

public:
  explicit Dfg(const Cfg *c) : cfg(c), fxn_reaching_defs_ins_(0) {
    assert(cfg && "NULL CFG!!");
    num_nodes = 0;
    code_size = cfg->get_code().size();
    recompute();
  }

  const Cfg &getCfg() const {
    return *cfg;
  }

  DataValue *ValueAtIndex(size_t idx) const {
    if (!Index2Value.count(idx)) {
      std::cerr << "Missing value for index " << idx << "\n";
      assert(0);
    }
    return Index2Value.at(idx);
  }

  void setValueAtIndex(size_t idx, DataValue *dfv) {
    if (Index2Value.count(idx)) {
      std::cerr << "Already existing value at index " << idx << "\n";
      assert(0);
    }
    Value2Index[dfv] = idx;
    Index2Value[idx] = dfv;
    Instr2Indices[dfv->getInstrIndex()].push_back(idx);
  }

  stringstream &printNode(stringstream &os, size_t idx) const {
    assert(idx < num_nodes && "printDataFlowValue: index OOB!!");

    DataValue *dfv = ValueAtIndex(idx);
    return dfv->print(os, this);
  }

  stringstream &printDFG(stringstream &os) const {
    os << "Printing info of each dfg node...\n";
    for (size_t k = 0; k < num_nodes; k++) {
      os << "#" << k << ": ";
      printNode(os, k);
      os << "\n";

      os << "Def-Ins: \n";
      per_dfg_node_reaching_defs_in_[k].print(os, this);
      os << "\n";

    }

    os << "Printing data flow values at each program point...\n";
    for (size_t k = 0; k < code_size; k++) {
      printDataFlowValue(os, k);
    }

    return os;
  }

  stringstream &printDataFlowValue(stringstream &os, size_t idx) const {
    assert(idx < code_size && "printDataFlowValue: index OOB!!");

    os << "#" << idx << ": " << cfg->get_code()[idx] << "\n";

    auto gen_set = per_instr_reaching_defs_in_gen_[idx];
    auto kill_set = per_instr_reaching_defs_in_kill_[idx];
    auto rd_in_set = per_instr_reaching_defs_in_[idx];
    auto rd_used_in_set = per_instr_reaching_and_used_defs_in_[idx];

    os << "Gen-Set: \n";
    gen_set.print(os, this);
    os << "\n";

    os << "Kill-Set: \n";
    kill_set.print(os, this);
    os << "\n";

    os << "RD-In-Set: \n";
    rd_in_set.print(os, this);
    os << "\n";

    os << "RD-n-Used-In-Set: \n";
    rd_used_in_set.print(os, this);

    os << "\n\n";
    return os;
  }

  /** The gen set for each instruction. */
  vector<DataFlowValue> per_instr_reaching_defs_in_gen_;
  /** The kill set for each instruction. */
  vector<DataFlowValue> per_instr_reaching_defs_in_kill_;
  ///** Recompute the indices in blocks_. */
  void recompute_nodes();
  ///** Recomputes the gen and kill sets used by recompute_reaching_defs(). */
  void recompute_reaching_defs_in_gen_kill();
  DataFlowValue fxn_reaching_defs_ins_;
  std::vector<DataFlowValue> per_instr_reaching_defs_in_;
  std::vector<DataFlowValue> per_instr_reaching_and_used_defs_in_;
  std::vector<DataFlowValue> per_instr_reaching_defs_out_;
  std::vector<DataFlowValue> per_dfg_node_reaching_defs_in_;

  /** Recompute internal state; recomputes basic block structure and data flow
   * values. */
  void recompute() {
    recompute_nodes();
    recompute_reaching_defs_in();
  }
  void recompute_reaching_defs_in();

  void serialize(std::ostream &os) const;
};

} // namespace stoke

#endif
