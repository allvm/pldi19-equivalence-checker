// Copyright 2013-2019 Stanford University
//
// Licensed under the Apache License, Version 2.0 (the License);
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an AS IS BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _STOKE_SRC_SYMSTATE_SEARCH_VISITOR
#define _STOKE_SRC_SYMSTATE_SEARCH_VISITOR

#include <sstream>

#include "src/ext/x64asm/include/x64asm.h"
#include "src/symstate/visitor.h"

namespace stoke {

/** A class to print symbolic formulas in a nicely readable way. */
class SymSearchVisitor
    : public SymVisitor<x64asm::RegSet, x64asm::RegSet, x64asm::RegSet> {

  // the implementation loosely follows https://gist.github.com/kputnam/5625856

public:
  SymSearchVisitor(std::ostream &os) : os_(os) {}

  /** Visit a bit-vector variable */
  x64asm::RegSet visit(const SymBitVectorVar *const bv) {
    x64asm::RegSet retval = x64asm::RegSet::empty();
    x64asm::RegSet gps_all = x64asm::RegSet::all_gps();
    x64asm::RegSet ymms_all = x64asm::RegSet::all_ymms();

    for (auto gp_it = gps_all.gp_begin(); gp_it != gps_all.gp_end(); ++gp_it) {
      // std::stringstream ss;
      //// auto &r = static_cast<x64asm::R64&>(*gp_it);
      // auto r = (x64asm::R64)(*gp_it);
      // ss << r;
      // if (ss.str() == bv->name_) {
      //  retval += r;
      //}

      std::stringstream ss(bv->name_);
      x64asm::R64 r64 = x64asm::Constants::rax();
      if (ss >> r64) {
        retval += r64;
      } else {
        std::cout << "\nNon R64: " << bv->name_;
      }
    }
    for (auto sse_it = ymms_all.sse_begin(); sse_it != ymms_all.sse_end();
         ++sse_it) {
      std::stringstream ss(bv->name_);
      x64asm::Ymm ymm = x64asm::Constants::ymm0();
      if (ss >> ymm) {
        retval += ymm;
      } else {
        std::cout << "\nNon Ymm: " << bv->name_;
      }
    }
    return retval;
  }

  /** Visit a boolean VAR */
  x64asm::RegSet visit(const SymBoolVar *const b) {
    x64asm::RegSet retval = x64asm::RegSet::empty();
    x64asm::RegSet flags_all =
        x64asm::RegSet::empty() + x64asm::Constants::eflags_cf() +
        x64asm::Constants::eflags_sf() + x64asm::Constants::eflags_zf() +
        x64asm::Constants::eflags_of() + x64asm::Constants::eflags_pf() +
        x64asm::Constants::eflags_af();

    for (auto flag_it = flags_all.flags_begin();
         flag_it != flags_all.flags_end(); ++flag_it) {
      std::stringstream ss;
      ss << *flag_it;
      if (ss.str() == b->name_) {
        retval += *flag_it;
      }
    }
    return retval;
  }

  x64asm::RegSet visit_binop(const SymBitVectorBinop *const e) {
    auto a = e->a_;
    auto b = e->b_;

    switch (e->type()) {
    case SymBitVector::AND:
    case SymBitVector::CONCAT:
    case SymBitVector::DIV:
    case SymBitVector::MINUS:
    case SymBitVector::MOD:
    case SymBitVector::MULT:
    case SymBitVector::OR:
    case SymBitVector::PLUS:
    case SymBitVector::ROTATE_LEFT:
    case SymBitVector::ROTATE_RIGHT:
    case SymBitVector::SHIFT_LEFT:
    case SymBitVector::SHIFT_RIGHT:
    case SymBitVector::SIGN_DIV:
    case SymBitVector::SIGN_MOD:
    case SymBitVector::SIGN_SHIFT_RIGHT:
    case SymBitVector::XOR:
      return (*this)(a) | (*this)(b);
      break;
    default:
      os_ << "(UNHANDLED_BINOP" << e->type() << " ";
      assert(false);
    }
  }

  /* Visit a binop on a bool */
  x64asm::RegSet visit_binop(const SymBoolBinop *const e) {

    switch (e->type()) {
    case SymBool::AND:
    case SymBool::IFF:
    case SymBool::IMPLIES:
    case SymBool::OR:
    case SymBool::XOR:
      return (*this)(e->a_) | (*this)(e->b_);
      break;
    default:
      os_ << "(UNHANDLED_BINOP" << e->type() << " ";
      assert(false);
    }
  }

  x64asm::RegSet visit_unop(const SymBitVectorUnop *const bv) {

    switch (bv->type()) {
    case SymBitVector::NOT:
    case SymBitVector::U_MINUS:
      return (*this)(bv->bv_);
      break;
    default:
      os_ << "UNHANDLED_UNOP" << bv->type() << " ";
      assert(false);
    }
  }

  x64asm::RegSet visit_compare(const SymBoolCompare *const e) {

    switch (e->type()) {
    case SymBool::EQ:
    case SymBool::GE:
    case SymBool::SIGN_GE:
    case SymBool::GT:
    case SymBool::SIGN_GT:
    case SymBool::LE:
    case SymBool::SIGN_LE:
    case SymBool::LT:
    case SymBool::SIGN_LT:
      return (*this)(e->a_) | (*this)(e->b_);
    default:
      os_ << "(UNHANDLED_COMPARE" << e->type() << " ";
      assert(false);
    }
  }

  /** Visit a bit-vector variable */
  x64asm::RegSet visit(const SymBitVectorArrayLookup *const bv) {
    return (*this)(bv->a_) | (*this)(bv->key_);
  }

  /** Visit a bit-vector constant */
  x64asm::RegSet visit(const SymBitVectorConstant *const bv) {
    return x64asm::RegSet::empty();
  }

  /** Visit a bit-vector extract */
  x64asm::RegSet visit(const SymBitVectorExtract *const bv) {
    return (*this)(bv->bv_);
  }

  /** Visit a bit-vector function */
  x64asm::RegSet visit(const SymBitVectorFunction *const bv) {
    x64asm::RegSet res = x64asm::RegSet::empty();
    for (size_t i = 0; i < bv->args_.size(); ++i) {
      res |= (*this)(bv->args_[i]);
    }

    return res;
  }

  /** Visit a bit-vector if-then-else */
  x64asm::RegSet visit(const SymBitVectorIte *const bv) {
    return (*this)(bv->a_) | (*this)(bv->b_);
  }

  /** Visit a bit-vector sign-extend */
  x64asm::RegSet visit(const SymBitVectorSignExtend *const bv) {
    return (*this)(bv->bv_);
  }

  /** Visit a boolean ARRAY_EQ */
  x64asm::RegSet visit(const SymBoolArrayEq *const b) {
    return (*this)(b->a_) | (*this)(b->b_);
  }

  /** Visit a boolean FALSE */
  x64asm::RegSet visit(const SymBoolFalse *const b) {
    return x64asm::RegSet::empty();
  }

  /** Visit a boolean FOR_ALL */
  x64asm::RegSet visit(const SymBoolForAll *const b) {
    assert(0 && "To DO: DSAND");
    return x64asm::RegSet::empty();
  }

  /** Visit a boolean NOT */
  x64asm::RegSet visit(const SymBoolNot *const b) {
    return (*this)(b->b_);
    ;
  }

  /** Visit a boolean TRUE */
  x64asm::RegSet visit(const SymBoolTrue *const b) {
    return x64asm::RegSet::empty();
  }

  /** Visit an array STORE */
  x64asm::RegSet visit(const SymArrayStore *const a) {
    return (*this)(a->a_) | (*this)(a->key_) | (*this)(a->value_);
  }

  /** Visit an array VAR */
  x64asm::RegSet visit(const SymArrayVar *const a) {
    return x64asm::RegSet::empty();
  }

private:
  std::ostream &os_;
};

} // namespace

#endif
