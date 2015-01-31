// CombUtils.hpp ---
//
// Filename: CombUtils.hpp
// Author: Abhishek Udupa
// Created: Mon Jul 28 23:52:36 2014 (-0400)
//
//
// Copyright (c) 2013, Abhishek Udupa, University of Pennsylvania
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by The University of Pennsylvania
// 4. Neither the name of the University of Pennsylvania nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//

// Code:

#if !defined ESMC_COMB_UTILS_HPP_
#define ESMC_COMB_UTILS_HPP_

#include "../common/ESMCFwdDecls.hpp"

#include <functional>
#include <vector>
#include <iterator>

namespace ESMC {

template <typename E>
static inline void
CrossProdInt(vector<vector<E>>& Result,
             vector<E>& Scratch,
             typename vector<vector<E>>::const_iterator Me,
             typename vector<vector<E>>::const_iterator End)
{
    if (Me == End) {
        Result.push_back(Scratch);
        return;
    }

    const vector<E>& MyVec = *Me;
    for (auto it = MyVec.begin(); it != MyVec.end(); ++it) {
        Scratch.push_back(*it);
        CrossProdInt(Result, Scratch, Me + 1, End);
        Scratch.pop_back();
    }
}

template <typename E>
static inline vector<vector<E>>
CrossProduct(typename vector<vector<E>>::const_iterator Begin,
             typename vector<vector<E>>::const_iterator End)
{
    vector<vector<E>> Result;
    vector<E> Scratch;
    CrossProdInt(Result, Scratch, Begin, End);
    return Result;
}

template <typename T, class ForwardIterator>
static inline vector<T> Filter(const ForwardIterator& First,
                               const ForwardIterator& Last,
                               const function<bool(const T&)>& Pred)
{
    vector<T> Retval;
    for (auto it = First; it != Last; ++it) {
        if (Pred(*it)) {
            Retval.push_back(*it);
        }
    }
    return Retval;
}

static inline u64 Factorial(u32 Num)
{
    u64 Retval = 1;
    for (u32 i = 1; i <= Num; ++i) {
        Retval *= i;
    }
    return Retval;
}

} /* end namespace */

#endif /* ESMC_COMB_UTILS_HPP_ */

//
// CombUtils.hpp ends here
