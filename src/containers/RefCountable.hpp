// RefCountable.hpp ---
//
// Filename: RefCountable.hpp
// Author: Abhishek Udupa
// Created: Sun Jun 29 13:47:16 2014 (-0400)
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

// Basic ref countable type

#if !defined ESMC_REF_COUNTABLE_HPP_
#define ESMC_REF_COUNTABLE_HPP_

#include "../common/ESMCFwdDecls.hpp"

namespace ESMC {

    class RefCountable
    {
    private:
        mutable i64 RefCount_;

    public:
        inline RefCountable()
            : RefCount_((i64)0)
        {
            // Nothing here
        }

        inline RefCountable(const RefCountable& Other)
            : RefCount_((i64)0)
        {
            // Nothing here
        }

        virtual ~RefCountable()
        {
            // Nothing here
        }

        inline void IncRef_() const
        {
            RefCount_++;
        }

        inline void DecRef_() const
        {
            RefCount_--;
            if (RefCount_ <= (i64)0) {
                delete this;
            }
        }

        inline i64 GetRefCnt_() const
        {
            return RefCount_;
        }
    };

} /* end namespace */

#endif /* ESMC_REF_COUNTABLE_HPP_ */

//
// RefCountable.hpp ends here
