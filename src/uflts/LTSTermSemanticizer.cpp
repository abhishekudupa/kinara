// LTSTermSemanticizer.cpp --- 
// 
// Filename: LTSTermSemanticizer.cpp
// Author: Abhishek Udupa
// Created: Sat Jul 26 15:58:15 2014 (-0400)
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

#include "LTSTermSemanticizer.hpp"

namespace ESMC {
    namespace LTS {
        
        const i64 LTSOps::OpEQ;
        const i64 LTSOps::OpNOT;
        const i64 LTSOps::OpITE;
        const i64 LTSOps::OpOR;
        const i64 LTSOps::OpAND;
        const i64 LTSOps::OpIMPLIES;
        const i64 LTSOps::OpIFF;
        const i64 LTSOps::OpXOR;

        // Arithmetic
        const i64 LTSOps::OpADD;
        const i64 LTSOps::OpSUB;
        const i64 LTSOps::OpMINUS;
        const i64 LTSOps::OpMUL;
        const i64 LTSOps::OpDIV;
        const i64 LTSOps::OpMOD;
        const i64 LTSOps::OpGT;
        const i64 LTSOps::OpGE;
        const i64 LTSOps::OpLT;
        const i64 LTSOps::OpLE;

        // Syntactic operators for symmetry, etc
        const i64 LTSOps::OpIndex;
        const i64 LTSOps::OpField;
        
        // Nondet
        const i64 LTSOps::OpNonDet;

        const i64 LTSOps::UFOffset;
        
        // Temporal operators
        const i64 LTSOps::OpTemporalX;
        const i64 LTSOps::OpTemporalU;
        const i64 LTSOps::OpTemporalF;
        const i64 LTSOps::OpTemporalG;

        // Some special types
        // for creating expressions for field access
        const i64 LTSOps::FieldAccType;
        const i64 LTSOps::UndefType;


        namespace Detail {

            const Exprs::ExprTypeRef InvalidType = Exprs::ExprTypeRef::NullPtr;

            const string BoundVarPrefix = "dbvar";

            const unordered_map<i64, string> OpCodeToNameMap = 
                { { LTSOps::OpEQ, "=" },
                  { LTSOps::OpNOT, "not" },
                  { LTSOps::OpITE, "ite" },
                  { LTSOps::OpOR, "or" },
                  { LTSOps::OpAND, "and" },
                  { LTSOps::OpIMPLIES, "implies" },
                  { LTSOps::OpIFF, "iff" },
                  { LTSOps::OpXOR, "xor" },
                  { LTSOps::OpADD, "+" },
                  { LTSOps::OpSUB, "-" },
                  { LTSOps::OpMINUS, "-" },
                  { LTSOps::OpMUL, "*" },
                  { LTSOps::OpDIV, "div" },
                  { LTSOps::OpMOD, "mod" },
                  { LTSOps::OpGT, ">" },
                  { LTSOps::OpGE, ">=" },
                  { LTSOps::OpLT, "<" },
                  { LTSOps::OpLE, "<=" },
                  { LTSOps::OpNonDet, "nondet"} };

        } /* end namespace Detail */
    } /* end namespace LTS */
} /* end namespace ESMC */


// 
// LTSTermSemanticizer.cpp ends here
