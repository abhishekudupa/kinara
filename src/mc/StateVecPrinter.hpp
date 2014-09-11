// StateVecPrinter.hpp --- 
// 
// Filename: StateVecPrinter.hpp
// Author: Abhishek Udupa
// Created: Wed Aug 20 16:20:55 2014 (-0400)
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

#if !defined ESMC_STATE_VEC_PRINTER_HPP_
#define ESMC_STATE_VEC_PRINTER_HPP_

#include "../common/FwdDecls.hpp"

namespace ESMC {
    namespace MC {

        using namespace ESMC::LTS;

        class ScalarPrinter
        {
        private:
            u32 Offset;
            u32 Size;
            ExprTypeRef Type;
            i64 Low;
            i64 High;

        public:
            ScalarPrinter();
            ScalarPrinter(u32 Offset, const ExprTypeRef& Type);
            ScalarPrinter(const ScalarPrinter& Other);
            ~ScalarPrinter();

            ScalarPrinter& operator = (const ScalarPrinter& Other);
            bool operator == (const ScalarPrinter& Other) const;

            string Print(const StateVec* StateVector) const;
        };

        class StateVecPrinter
        {
        private:
            vector<pair<ExpT, ScalarPrinter>> ExpsToPrint;
            LTSCompiler* Compiler;

            void MakePrinters(const ExpT& Exp, LabelledTS* TheLTS);
            
        public:
            StateVecPrinter(LabelledTS* TheLTS, LTSCompiler* Compiler);
            ~StateVecPrinter();

            vector<string> PrintState(const StateVec* StateVector) const;

            // Prints delta
            vector<string> PrintState(const StateVec* StateVector,
                                      const StateVec* PrevStateVector) const;

            void PrintState(const StateVec* StateVector, ostream& Out) const;
            void PrintState(const StateVec* StateVector,
                            const StateVec* PrevStateVector,
                            ostream& Out) const;

            void PrintState(const ProductState* State, ostream& Out) const;
            void PrintState(const ProductState* State, 
                            const ProductState* Prev, ostream& Out) const;
        };

    } /* end namespace MC */
} /* end namespace ESMC */

#endif /* ESMC_STATE_VEC_PRINTER_HPP_ */

// 
// StateVecPrinter.hpp ends here










