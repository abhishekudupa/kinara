// LTSExtensions.hpp ---
//
// Filename: LTSExtensions.hpp
// Author: Abhishek Udupa
// Created: Tue Jul 29 08:53:58 2014 (-0400)
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

#if !defined ESMC_LTS_EXTENSIONS_HPP_
#define ESMC_LTS_EXTENSIONS_HPP_

#include "../common/ESMCFwdDecls.hpp"
#include "LTSSemTypes.hpp"

// moved this single definition into a separate
// header file to eliminate circular deps.

namespace ESMC {
    namespace LTS {

        using ESMC::LTS::TypeExtensionBase;
        using ESMC::MC::RValueInterpreter;
        using ESMC::MC::LValueInterpreter;

        class LTSExtensionT
        {
        public:
            // Offset != -1 ==> Fixed offset
            i32 Offset;
            // info about constants
            bool ConstCompiled;
            i64 ConstVal;
            // Info for record accesses
            u32 FieldOffset;
            // Interpreter for whatever
            // cannot be compiled in
            RValueInterpreter* Interp;

            LTSExtensionT();
            ~LTSExtensionT();
        };

        class LTSTypeExtensionT : public TypeExtensionBase
        {
        public:
            // Offset in the permutation vector
            i32 TypeOffset;
            // ID of the type
            i32 TypeID;

            LTSTypeExtensionT();
            virtual ~LTSTypeExtensionT();
        };

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LTS_EXTENSIONS_HPP_ */

//
// LTSExtensions.hpp ends here
