// SizeUtils.hpp --- 
// 
// Filename: SizeUtils.hpp
// Author: Abhishek Udupa
// Created: Fri Aug  1 16:35:59 2014 (-0400)
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

#if !defined ESMC_SIZE_UTILS_HPP_
#define ESMC_SIZE_UTILS_HPP_

#include "../common/FwdDecls.hpp"
#include <math.h>

namespace ESMC {

    static inline u32 NextMultiple(u32 Value, u32 Size)
    {
        auto Rem = Value % Size;
        return (Rem == 0 ? Value : (Value + (Size - Rem)));
    }

    // Pads value by some number of bytes to get the best alignment
    static inline u32 Align(u32 Value, u32 Size)
    {
        if (Size == 1) {
            return Value;
        } else if (Size == 2) {
            return NextMultiple(Value, 2);
        } else {
            // An alignment of 4 is considered
            // sufficient for all types
            return NextMultiple(Value, 4);
        }
    }

    static inline u32 BytesForRange(u32 RangeWidth)
    {
        auto LogVal = (u32)ceil(log((double)RangeWidth) / log(2.0));
        if (LogVal <= 8) {
            return 1;
        } else if (LogVal <= 16) {
            return 2;
        } else if (LogVal <= 32) {
            return 4;
        } else {
            throw ESMCError((string)"Domain of variable too large");
        }
    }



} /* end namespace ESMC */

#endif /* ESMC_SIZE_UTILS_HPP_ */

// 
// SizeUtils.hpp ends here
