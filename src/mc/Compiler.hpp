// Compiler.hpp --- 
// 
// Filename: Compiler.hpp
// Author: Abhishek Udupa
// Created: Mon Aug 18 01:53:54 2014 (-0400)
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

#if !defined ESMC_COMPILER_HPP_
#define ESMC_COMPILER_HPP_

#include "../common/FwdDecls.hpp"
#include "../uflts/LTSTypes.hpp"


namespace ESMC {
    namespace MC {

        using namespace ESMC::LTS;

        class LValueInterpreter;

        class RValueInterpreter
        {
        protected:
            ExprTypeRef Type;
            u32 Size;
            bool IsScalar;
            
        public:
            RValueInterpreter(const ExprTypeRef& Type);
            virtual ~RValueInterpreter();
            
            virtual i64 GetScalarValue(const StateVec* StateVector) const = 0;
            virtual string GetScalarString(const StateVec* StateVector) const = 0;
            virtual const u08* GetPtrValue(const StateVec* StateVector) const = 0;
            virtual void WriteToLValue(const StateVec* InStateVector,
                                       StateVec* OutStateVector,
                                       const LValueInterpreter* LValInterp) const = 0;

            static void MakeInterpreter(const ExpT& Exp, const LabelledTS* TheLTS);

            template <typename T> 
            inline T* As()
            {
                return dynamic_cast<T*>(this);
            }

            template <typename T>
            inline const T* As() const
            {
                return dynamic_cast<const T*>(this);
            }
            
            template <typename T>
            inline bool Is() const
            {
                return (dynamic_cast<const T*>(this) != nullptr);
            }
        };

        class LValueInterpreter : public RValueInterpreter
        {
            friend class RValueInterpreter;

        protected:
            bool IsMsg;
            i32 FixedOffset;

        public:
            LValueInterpreter(const ExprTypeRef& Type, bool IsMsg, i32 FixedOffset = -1);
            virtual ~LValueInterpreter();

            virtual void Write(StateVec* StateVector, u08* SrcPtr) const = 0;
        };

        class ConstantInterpreter : public RValueInterpreter
        {
        private:
            i64 ScalarValue;
            u08* PtrValue;

        public:
            ConstantInterpreter(const ExpT& Exp);
            virtual ~ConstantInterpreter();

            virtual i64 GetScalarValue(const StateVec* StateVector) const override;
            virtual string GetScalarString(const StateVec* StateVector) const override;
            virtual const u08* GetPtrValue(const StateVec* StateVector) const override;
            virtual void WriteToLValue(const StateVec* InStateVector,
                                       StateVec* OutStateVector,
                                       const LValueInterpreter* LValInterp) const override;
        };

        class OpInterpreter : public RValueInterpreter
        {
        protected:
            vector<RValueInterpreter*> SubInterps;

        public:

        };

    } /* end namespace MC */
} /* end namespace ESMC */

#endif /* ESMC_COMPILER_HPP_ */

// 
// Compiler.hpp ends here
