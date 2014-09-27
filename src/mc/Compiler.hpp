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

        extern const i64 UndefValue;

        class OffsetCompiler : public VisitorBaseT
        {
        private:
            const LabelledTS* TheLTS;
        public:
            OffsetCompiler(const LabelledTS* TheLTS);
            virtual ~OffsetCompiler();

            virtual void VisitVarExpression(const VarExpT* Exp) override;
            virtual void VisitBoundVarExpression(const BoundVarExpT* Exp) override;
            virtual void VisitConstExpression(const ConstExpT* Exp) override;
            virtual void VisitOpExpression(const OpExpT* Exp) override;
            virtual void VisitAQuantifiedExpression(const AQExpT* Exp) override;
            virtual void VisitEQuantifiedExpression(const EQExpT* Exp) override;

            static void Do(const ExpT& Exp, const LabelledTS* TheLTS);
        };

        class LTSCompiler;

        class RValueInterpreter
        {
        protected:
            bool Scalar;
            u32 Size;

        public:
            RValueInterpreter(bool Scalar, u32 Size);
            virtual ~RValueInterpreter();
            
            bool IsScalar() const;
            u32 GetSize() const;

            virtual i64 EvaluateScalar(const StateVec* StateVector) const = 0;
            virtual const u08* Evaluate(const StateVec* StateVector) const = 0;
            // Non-exception versions
            virtual i64 EvaluateScalarNE(const StateVec* StateVector) const = 0;
            virtual const u08* EvaluateNE(const StateVec* StateVector) const = 0;

            static void MakeInterpreter(const ExpT& Exp, LTSCompiler* Compiler);

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
            inline T* SAs()
            {
                return static_cast<T*>(this);
            }
            
            template <typename T>
            inline const T* SAs() const
            {
                return static_cast<const T*>(this);
            }

            template <typename T>
            inline bool Is() const
            {
                return (dynamic_cast<const T*>(this) != nullptr);
            }
        };

        class LValueInterpreter : public RValueInterpreter
        {
        protected:
            // Should we refer to the message
            // and not the state vector itself?
            bool Msg;
            i64 Low;
            i64 High;

        public:
            LValueInterpreter(u32 Size, bool Msg, bool IsScalar, 
                              i64 Low, i64 High);
            virtual ~LValueInterpreter();

            i64 GetLow() const;
            i64 GetHigh() const;
            bool IsMsg() const;

            virtual void WriteScalar(i64 Value, StateVec* StateVector) const = 0;
            virtual void Write(const u08* Ptr, StateVec* StateVector) const = 0;

            void Update(const RValueInterpreter* RHS, const StateVec* InStateVector,
                        StateVec* OutStateVector) const;
        };

        class CompiledConstInterpreter : public RValueInterpreter
        {
        private:
            i64 Value;
            u08* Ptr;
            
        public:
            CompiledConstInterpreter(u32 Size, i64 Value);
            CompiledConstInterpreter(u32 Size, u08* Ptr);
            virtual ~CompiledConstInterpreter();

            virtual i64 EvaluateScalar(const StateVec* StateVector) const override;
            virtual const u08* Evaluate(const StateVec* StateVector) const override;

            virtual i64 EvaluateScalarNE(const StateVec* StateVector) const override;
            virtual const u08* EvaluateNE(const StateVec* StateVector) const override;
        };

        class CompiledLValueInterpreter : public LValueInterpreter
        {
        private:
            u32 Offset;
            
        public:
            CompiledLValueInterpreter(u32 Size, bool Msg, bool Scalar, 
                                      u32 Offset, i64 Low, 
                                      i64 High);
            virtual ~CompiledLValueInterpreter();

            u32 GetOffset() const;

            virtual i64 EvaluateScalar(const StateVec* StateVector) const override;
            virtual const u08* Evaluate(const StateVec* StateVector) const override;
            
            virtual i64 EvaluateScalarNE(const StateVec* StateVector) const override;
            virtual const u08* EvaluateNE(const StateVec* StateVector) const override;

            virtual void WriteScalar(i64 Value, StateVec* StateVector) const override;
            virtual void Write(const u08* Ptr, StateVec* StateVector) const override;
        };

        class OpInterpreter : public RValueInterpreter
        {
        protected:
            vector<RValueInterpreter*> SubInterps;
            mutable vector<i64> SubEvals;
            const u32 NumSubInterps;

            inline void EvaluateSubInterps(const StateVec* StateVector) const;
            
        public:
            OpInterpreter(bool Scalar, u32 Size, 
                          const vector<RValueInterpreter*>& SubInterps);
            virtual ~OpInterpreter();
        };

        class EQInterpreter : public OpInterpreter
        {
        public:
            EQInterpreter(const vector<RValueInterpreter*>& SubInterps);
            virtual ~EQInterpreter();

            virtual i64 EvaluateScalar(const StateVec* StateVector) const override;
            virtual const u08* Evaluate(const StateVec* StateVector) const override;

            virtual i64 EvaluateScalarNE(const StateVec* StateVector) const override;
            virtual const u08* EvaluateNE(const StateVec* StateVector) const override;
        };

        class NOTInterpreter : public OpInterpreter
        {
        public:
            NOTInterpreter(const vector<RValueInterpreter*>& SubInterps);
            virtual ~NOTInterpreter();

            virtual i64 EvaluateScalar(const StateVec* StateVector) const override;
            virtual const u08* Evaluate(const StateVec* StateVector) const override;

            virtual i64 EvaluateScalarNE(const StateVec* StateVector) const override;
            virtual const u08* EvaluateNE(const StateVec* StateVector) const override;
        };

        class ITEInterpreter : public OpInterpreter
        {
        public:
            ITEInterpreter(const vector<RValueInterpreter*>& SubInterps);
            virtual ~ITEInterpreter();
            
            virtual i64 EvaluateScalar(const StateVec* StateVector) const override;
            virtual const u08* Evaluate(const StateVec* StateVector) const override;

            virtual i64 EvaluateScalarNE(const StateVec* StateVector) const override;
            virtual const u08* EvaluateNE(const StateVec* StateVector) const override;
        };

        class ORInterpreter : public OpInterpreter
        {
        public:
            ORInterpreter(const vector<RValueInterpreter*>& SubInterps);
            virtual ~ORInterpreter();
            
            virtual i64 EvaluateScalar(const StateVec* StateVector) const override;
            virtual const u08* Evaluate(const StateVec* StateVector) const override;

            virtual i64 EvaluateScalarNE(const StateVec* StateVector) const override;
            virtual const u08* EvaluateNE(const StateVec* StateVector) const override;
        };


        class ANDInterpreter : public OpInterpreter
        {
        public:
            ANDInterpreter(const vector<RValueInterpreter*>& SubInterps);
            virtual ~ANDInterpreter();
            
            virtual i64 EvaluateScalar(const StateVec* StateVector) const override;
            virtual const u08* Evaluate(const StateVec* StateVector) const override;

            virtual i64 EvaluateScalarNE(const StateVec* StateVector) const override;
            virtual const u08* EvaluateNE(const StateVec* StateVector) const override;
        };

        class IMPLIESInterpreter : public OpInterpreter
        {
        public:
            IMPLIESInterpreter(const vector<RValueInterpreter*>& SubInterps);
            virtual ~IMPLIESInterpreter();
            
            virtual i64 EvaluateScalar(const StateVec* StateVector) const override;
            virtual const u08* Evaluate(const StateVec* StateVector) const override;

            virtual i64 EvaluateScalarNE(const StateVec* StateVector) const override;
            virtual const u08* EvaluateNE(const StateVec* StateVector) const override;
        };

        class IFFInterpreter : public OpInterpreter
        {
        public:
            IFFInterpreter(const vector<RValueInterpreter*>& SubInterps);
            virtual ~IFFInterpreter();
            
            virtual i64 EvaluateScalar(const StateVec* StateVector) const override;
            virtual const u08* Evaluate(const StateVec* StateVector) const override;

            virtual i64 EvaluateScalarNE(const StateVec* StateVector) const override;
            virtual const u08* EvaluateNE(const StateVec* StateVector) const override;
        };

        class XORInterpreter : public OpInterpreter
        {
        public:
            XORInterpreter(const vector<RValueInterpreter*>& SubInterps);
            virtual ~XORInterpreter();
            
            virtual i64 EvaluateScalar(const StateVec* StateVector) const override;
            virtual const u08* Evaluate(const StateVec* StateVector) const override;

            virtual i64 EvaluateScalarNE(const StateVec* StateVector) const override;
            virtual const u08* EvaluateNE(const StateVec* StateVector) const override;
        };

        class ADDInterpreter : public OpInterpreter
        {
        public:
            ADDInterpreter(const vector<RValueInterpreter*>& SubInterps);
            virtual ~ADDInterpreter();
            
            virtual i64 EvaluateScalar(const StateVec* StateVector) const override;
            virtual const u08* Evaluate(const StateVec* StateVector) const override;

            virtual i64 EvaluateScalarNE(const StateVec* StateVector) const override;
            virtual const u08* EvaluateNE(const StateVec* StateVector) const override;
        };

        class SUBInterpreter : public OpInterpreter
        {
        public:
            SUBInterpreter(const vector<RValueInterpreter*>& SubInterps);
            virtual ~SUBInterpreter();
            
            virtual i64 EvaluateScalar(const StateVec* StateVector) const override;
            virtual const u08* Evaluate(const StateVec* StateVector) const override;

            virtual i64 EvaluateScalarNE(const StateVec* StateVector) const override;
            virtual const u08* EvaluateNE(const StateVec* StateVector) const override;
        };
        
        class MINUSInterpreter : public OpInterpreter
        {
        public:
            MINUSInterpreter(const vector<RValueInterpreter*>& SubInterps);
            virtual ~MINUSInterpreter();
            
            virtual i64 EvaluateScalar(const StateVec* StateVector) const override;
            virtual const u08* Evaluate(const StateVec* StateVector) const override;

            virtual i64 EvaluateScalarNE(const StateVec* StateVector) const override;
            virtual const u08* EvaluateNE(const StateVec* StateVector) const override;
        };
        
        class MULInterpreter : public OpInterpreter
        {
        public:
            MULInterpreter(const vector<RValueInterpreter*>& SubInterps);
            virtual ~MULInterpreter();
            
            virtual i64 EvaluateScalar(const StateVec* StateVector) const override;
            virtual const u08* Evaluate(const StateVec* StateVector) const override;

            virtual i64 EvaluateScalarNE(const StateVec* StateVector) const override;
            virtual const u08* EvaluateNE(const StateVec* StateVector) const override;
        };

        class DIVInterpreter : public OpInterpreter
        {
        public:
            DIVInterpreter(const vector<RValueInterpreter*>& SubInterps);
            virtual ~DIVInterpreter();
            
            virtual i64 EvaluateScalar(const StateVec* StateVector) const override;
            virtual const u08* Evaluate(const StateVec* StateVector) const override;

            virtual i64 EvaluateScalarNE(const StateVec* StateVector) const override;
            virtual const u08* EvaluateNE(const StateVec* StateVector) const override;
        };

        class MODInterpreter : public OpInterpreter
        {
        public:
            MODInterpreter(const vector<RValueInterpreter*>& SubInterps);
            virtual ~MODInterpreter();
            
            virtual i64 EvaluateScalar(const StateVec* StateVector) const override;
            virtual const u08* Evaluate(const StateVec* StateVector) const override;

            virtual i64 EvaluateScalarNE(const StateVec* StateVector) const override;
            virtual const u08* EvaluateNE(const StateVec* StateVector) const override;
        };

        class GTInterpreter : public OpInterpreter
        {
        public:
            GTInterpreter(const vector<RValueInterpreter*>& SubInterps);
            virtual ~GTInterpreter();
            
            virtual i64 EvaluateScalar(const StateVec* StateVector) const override;
            virtual const u08* Evaluate(const StateVec* StateVector) const override;

            virtual i64 EvaluateScalarNE(const StateVec* StateVector) const override;
            virtual const u08* EvaluateNE(const StateVec* StateVector) const override;
        };
        

        class GEInterpreter : public OpInterpreter
        {
        public:
            GEInterpreter(const vector<RValueInterpreter*>& SubInterps);
            virtual ~GEInterpreter();
            
            virtual i64 EvaluateScalar(const StateVec* StateVector) const override;
            virtual const u08* Evaluate(const StateVec* StateVector) const override;

            virtual i64 EvaluateScalarNE(const StateVec* StateVector) const override;
            virtual const u08* EvaluateNE(const StateVec* StateVector) const override;
        };

        class LTInterpreter : public OpInterpreter
        {
        public:
            LTInterpreter(const vector<RValueInterpreter*>& SubInterps);
            virtual ~LTInterpreter();
            
            virtual i64 EvaluateScalar(const StateVec* StateVector) const override;
            virtual const u08* Evaluate(const StateVec* StateVector) const override;

            virtual i64 EvaluateScalarNE(const StateVec* StateVector) const override;
            virtual const u08* EvaluateNE(const StateVec* StateVector) const override;
        };
        
        class LEInterpreter : public OpInterpreter
        {
        public:
            LEInterpreter(const vector<RValueInterpreter*>& SubInterps);
            virtual ~LEInterpreter();
            
            virtual i64 EvaluateScalar(const StateVec* StateVector) const override;
            virtual const u08* Evaluate(const StateVec* StateVector) const override;

            virtual i64 EvaluateScalarNE(const StateVec* StateVector) const override;
            virtual const u08* EvaluateNE(const StateVec* StateVector) const override;
        };

        class IndexInterpreter : public LValueInterpreter
        {
        private:
            LValueInterpreter* ArrayInterp;
            RValueInterpreter* IndexInterp;
            u32 ElemSize;

        public:
            IndexInterpreter(u32 Size, bool Msg, bool IsScalar,
                             LValueInterpreter* ArrayInterp,
                             RValueInterpreter* IndexInterp,
                             u32 ElemSize,
                             i64 Low, i64 High);
            virtual ~IndexInterpreter();

            virtual i64 EvaluateScalar(const StateVec* StateVector) const override;
            virtual const u08* Evaluate(const StateVec* StateVector) const override;

            virtual i64 EvaluateScalarNE(const StateVec* StateVector) const override;
            virtual const u08* EvaluateNE(const StateVec* StateVector) const override;

            virtual void WriteScalar(i64 Value, StateVec* StateVector) const override;
            virtual void Write(const u08* Ptr, StateVec* StateVector) const override;
        };

        class FieldInterpreter : public LValueInterpreter
        {
        private:
            LValueInterpreter* RecInterp;
            u32 FieldOffset;

        public:
            FieldInterpreter(u32 Size, bool Msg, bool IsScalar,
                             LValueInterpreter* RecInterp,
                             u32 FieldOffset,
                             i64 Low, i64 High);
            virtual ~FieldInterpreter();

            virtual i64 EvaluateScalar(const StateVec* StateVector) const override;
            virtual const u08* Evaluate(const StateVec* StateVector) const override;

            virtual i64 EvaluateScalarNE(const StateVec* StateVector) const override;
            virtual const u08* EvaluateNE(const StateVec* StateVector) const override;

            virtual void WriteScalar(i64 Value, StateVec* StateVector) const override;
            virtual void Write(const u08* Ptr, StateVec* StateVector) const override;
        };

        class LTSCompiler
        {
        private:
            vector<RValueInterpreter*> InterpsToFree;

        public:
            LTSCompiler();
            ~LTSCompiler();

            void RegisterInterp(RValueInterpreter* Interp);
            void CompileExp(const ExpT& Exp, LabelledTS* TheLTS);
            void CompileLTS(LabelledTS* TheLTS);
        };
        
    } /* end namespace MC */
} /* end namespace ESMC */

#endif /* ESMC_COMPILER_HPP_ */

// 
// Compiler.hpp ends here
