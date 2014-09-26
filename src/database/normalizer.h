
#ifndef STOKE_SRC_NORMALIZER_NORMALIZER_H
#define STOKE_SRC_NORMALIZER_NORMALIZER_H

#include <vector>
#include <functional>

#include "src/cfg/cfg.h"
#include "src/database/chunk.h"

namespace stoke {

class Normalizer {

  public:

    /* This type represents a function that takes code and does something
       meaningful to it, like uploads it to a database, and then resumes
       the computation.  */
    typedef std::function<void (x64asm::Code*)> CodeContinuation;

    /* This type represents a function that takes code and transforms
       it into something else.  An important convention I'm using is that the
       CodeOperators will allocate a new object without touching the old one.
       It's the job of the composition function to take care of freeing these
       objects after they've been used by the continuation. */
    typedef std::function<void (x64asm::Code*, CodeContinuation)> CodeOperator;

    Normalizer() {};

    /* This function is used to specify the action to take once a piece of code
       is fully normalized. */
    void reset_pipeline(CodeContinuation c) { continuation_ = c; }

    /* This function takes an operator to put into the chain of things done to
       each piece of code. */
    void operator<< (CodeOperator op) {
      continuation_ = compose(op, continuation_);
    }

    /* This operator renames the registers in order of appearance */
    CodeOperator normalize_registers();

    /* This operator renames the constants in order of appearance */
    CodeOperator normalize_constants();

    /* This operator splits a big piece of code into "chunks", which
       are call-free fragments of basic blocks.  It also filters by
       the depth of said blocks. */
    CodeOperator extract_chunks_of_depth(int depth);

    /* This operator takes the code and pulls out smaller portions of
       it and passes them down the pipeline */
    CodeOperator mangle_length();

    void run(x64asm::Code code);

  private:
    std::vector<x64asm::Code> chunk_list_;
    std::vector<int> nesting_depth_;

    CodeContinuation continuation_;
    CodeContinuation compose(CodeOperator op, CodeContinuation cont);


};

}




#endif
