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

#include "../tpinterface/TheoremProver.hpp"
#include "../common/ESMCFwdDecls.hpp"
#include "../uflts/LTSDecls.hpp"

namespace ESMC {
    namespace MC {

        using namespace ESMC::LTS;

        extern const i64 ExceptionValue;

        enum class UpdateStatusT {
            UpdateOK, EvalException, BoundsViolation
        };

        typedef const ExpressionBase<LTSExtensionT, LTSTermSemanticizer>* ExpPtrT;

        class OffsetCompiler : public VisitorBaseT
        {
        public:
            OffsetCompiler();
            virtual ~OffsetCompiler();

            virtual void VisitVarExpression(const VarExpT* Exp) override;
            virtual void VisitBoundVarExpression(const BoundVarExpT* Exp) override;
            virtual void VisitConstExpression(const ConstExpT* Exp) override;
            virtual void VisitOpExpression(const OpExpT* Exp) override;
            virtual void VisitAQuantifiedExpression(const AQExpT* Exp) override;
            virtual void VisitEQuantifiedExpression(const EQExpT* Exp) override;

            static void Do(const ExpT& Exp);
        };

        class LTSCompiler;

        class RValueInterpreter
        {
        protected:
            ExpPtrT Exp;
            ExpT NoExceptionPredicate;
            ExpT TrueExp;

            inline void SetNoExceptionPred(const ExpT& NewPred);

        public:
            RValueInterpreter(ExpPtrT Exp, const ExpT& NoExceptionPredicate);
            virtual ~RValueInterpreter();

            ExpPtrT GetExp() const;

            virtual i64 Evaluate(const StateVec* StateVector) const = 0;
            virtual void UpdateModel(const Z3Model& Model,
                                     const unordered_set<i64>& InterpretedOps,
                                     const unordered_map<i64, ExpT>& IndicatorExps) const;
            const ExpT& GetNoExceptionPredicate() const;

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
            i64 Low;
            i64 High;
            u32 Size;
            bool Scalar;

        public:
            LValueInterpreter(ExpPtrT Exp, const ExpT& NoExceptionPredicate);
            virtual ~LValueInterpreter();

            i64 GetLow() const;
            i64 GetHigh() const;
            bool IsScalar() const;
            u32 GetSize() const;

            // Return success or failure
            // Failure could be due to undef values
            // or out of bounds values
            virtual UpdateStatusT Write(i64 Value, const StateVec* InStateVector,
                                        StateVec* OutStateVector, ExpT& NEPred) const
                __attribute__ ((warn_unused_result)) = 0;

            virtual i64 GetOffset(const StateVec* StateVector) const = 0;

            // Return success or failure, same as Write
            UpdateStatusT Update(const RValueInterpreter* RHS, const StateVec* InStateVector,
                                 StateVec* OutStateVector, ExpT& NEPred) const
                __attribute__ ((warn_unused_result));
        };

        class CompiledConstInterpreter : public RValueInterpreter
        {
        private:
            i64 Value;

        public:
            CompiledConstInterpreter(i64 Value, ExpPtrT Exp);
            virtual ~CompiledConstInterpreter();

            virtual i64 Evaluate(const StateVec* StateVector) const override;
        };

        class CompiledLValueInterpreter : public LValueInterpreter
        {
        private:
            u32 Offset;

        public:
            CompiledLValueInterpreter(u32 Offset, ExpPtrT Exp);
            virtual ~CompiledLValueInterpreter();

            u32 GetOffset() const;

            virtual i64 GetOffset(const StateVec* StateVector) const override;
            virtual i64 Evaluate(const StateVec* StateVector) const override;
            virtual UpdateStatusT Write(i64 Value, const StateVec* InStateVector,
                                        StateVec* OutStateVector, ExpT& NEPred) const override;
        };

        namespace Detail {

            class ValueVecHasher
            {
            public:
                inline u64 operator () (const vector<i64>& Vec) const
                {
                    u64 Retval = 0;
                    boost::hash_combine(Retval, Vec.size());
                    for (auto const& Elem : Vec) {
                        boost::hash_combine(Retval, Elem);
                    }
                    return Retval;
                }
            };

        } /* end namespace Detail */

        class UFInterpreter : public RValueInterpreter
        {
        public:
            typedef unordered_map<vector<i64>, i64,
                                  Detail::ValueVecHasher> EvalMapT;

        private:
            vector<RValueInterpreter*> ArgInterps;
            mutable vector<i64> SubEvals;
            const u32 NumArgInterps;
            mutable EvalMapT EvalMap;
            mutable Z3Model Model;
            mutable bool Enabled;
            i64 MyOpCode;
            i64 Low;
            i64 High;

            inline i64 DoEval() const;

        public:
            UFInterpreter(const vector<RValueInterpreter*>& ArgInterps,
                          ExpPtrT Exp);
            virtual ~UFInterpreter();

            virtual i64 Evaluate(const StateVec* StateVector) const override;
            virtual void UpdateModel(const Z3Model& Model,
                                     const unordered_set<i64>& InterpretedOps,
                                     const unordered_map<i64, ExpT>& IndicatorExps) const override;
            const EvalMapT& GetEvalMap() const;
            i64 GetOpCode() const;
            bool IsEnabled() const;
        };

        class OpInterpreter : public RValueInterpreter
        {
        protected:
            vector<RValueInterpreter*> SubInterps;
            mutable vector<i64> SubEvals;
            const u32 NumSubInterps;

            inline void EvaluateSubInterps(const StateVec* StateVector) const;
            inline void SetNEPredAllDef();

        public:
            OpInterpreter(const vector<RValueInterpreter*>& SubInterps,
                          ExpPtrT Exp);
            virtual ~OpInterpreter();
        };

        class EQInterpreter : public OpInterpreter
        {
        public:
            EQInterpreter(const vector<RValueInterpreter*>& SubInterps, ExpPtrT Exp);
            virtual ~EQInterpreter();

            virtual i64 Evaluate(const StateVec* StateVector) const override;
        };

        class NOTInterpreter : public OpInterpreter
        {
        public:
            NOTInterpreter(const vector<RValueInterpreter*>& SubInterps, ExpPtrT Exp);
            virtual ~NOTInterpreter();

            virtual i64 Evaluate(const StateVec* StateVector) const override;
        };

        class ITEInterpreter : public OpInterpreter
        {
        public:
            ITEInterpreter(const vector<RValueInterpreter*>& SubInterps, ExpPtrT Exp);
            virtual ~ITEInterpreter();

            virtual i64 Evaluate(const StateVec* StateVector) const override;
        };

        class ORInterpreter : public OpInterpreter
        {
        public:
            ORInterpreter(const vector<RValueInterpreter*>& SubInterps, ExpPtrT Exp);
            virtual ~ORInterpreter();

            virtual i64 Evaluate(const StateVec* StateVector) const override;
        };


        class ANDInterpreter : public OpInterpreter
        {
        public:
            ANDInterpreter(const vector<RValueInterpreter*>& SubInterps, ExpPtrT Exp);
            virtual ~ANDInterpreter();

            virtual i64 Evaluate(const StateVec* StateVector) const override;
        };

        class IMPLIESInterpreter : public OpInterpreter
        {
        public:
            IMPLIESInterpreter(const vector<RValueInterpreter*>& SubInterps, ExpPtrT Exp);
            virtual ~IMPLIESInterpreter();

            virtual i64 Evaluate(const StateVec* StateVector) const override;
        };

        class IFFInterpreter : public OpInterpreter
        {
        public:
            IFFInterpreter(const vector<RValueInterpreter*>& SubInterps, ExpPtrT Exp);
            virtual ~IFFInterpreter();

            virtual i64 Evaluate(const StateVec* StateVector) const override;
        };

        class XORInterpreter : public OpInterpreter
        {
        public:
            XORInterpreter(const vector<RValueInterpreter*>& SubInterps, ExpPtrT Exp);
            virtual ~XORInterpreter();

            virtual i64 Evaluate(const StateVec* StateVector) const override;
        };

        class ADDInterpreter : public OpInterpreter
        {
        public:
            ADDInterpreter(const vector<RValueInterpreter*>& SubInterps, ExpPtrT Exp);
            virtual ~ADDInterpreter();

            virtual i64 Evaluate(const StateVec* StateVector) const override;
        };

        class SUBInterpreter : public OpInterpreter
        {
        public:
            SUBInterpreter(const vector<RValueInterpreter*>& SubInterps, ExpPtrT Exp);
            virtual ~SUBInterpreter();

            virtual i64 Evaluate(const StateVec* StateVector) const override;
        };

        class MINUSInterpreter : public OpInterpreter
        {
        public:
            MINUSInterpreter(const vector<RValueInterpreter*>& SubInterps, ExpPtrT Exp);
            virtual ~MINUSInterpreter();

            virtual i64 Evaluate(const StateVec* StateVector) const override;
        };

        class MULInterpreter : public OpInterpreter
        {
        public:
            MULInterpreter(const vector<RValueInterpreter*>& SubInterps, ExpPtrT Exp);
            virtual ~MULInterpreter();

            virtual i64 Evaluate(const StateVec* StateVector) const override;
        };

        class DIVInterpreter : public OpInterpreter
        {
        public:
            DIVInterpreter(const vector<RValueInterpreter*>& SubInterps, ExpPtrT Exp);
            virtual ~DIVInterpreter();

            virtual i64 Evaluate(const StateVec* StateVector) const override;
        };

        class MODInterpreter : public OpInterpreter
        {
        public:
            MODInterpreter(const vector<RValueInterpreter*>& SubInterps, ExpPtrT Exp);
            virtual ~MODInterpreter();

            virtual i64 Evaluate(const StateVec* StateVector) const override;
        };

        class GTInterpreter : public OpInterpreter
        {
        public:
            GTInterpreter(const vector<RValueInterpreter*>& SubInterps, ExpPtrT Exp);
            virtual ~GTInterpreter();

            virtual i64 Evaluate(const StateVec* StateVector) const override;
        };


        class GEInterpreter : public OpInterpreter
        {
        public:
            GEInterpreter(const vector<RValueInterpreter*>& SubInterps, ExpPtrT Exp);
            virtual ~GEInterpreter();

            virtual i64 Evaluate(const StateVec* StateVector) const override;
        };

        class LTInterpreter : public OpInterpreter
        {
        public:
            LTInterpreter(const vector<RValueInterpreter*>& SubInterps, ExpPtrT Exp);
            virtual ~LTInterpreter();

            virtual i64 Evaluate(const StateVec* StateVector) const override;
        };

        class LEInterpreter : public OpInterpreter
        {
        public:
            LEInterpreter(const vector<RValueInterpreter*>& SubInterps, ExpPtrT Exp);
            virtual ~LEInterpreter();

            virtual i64 Evaluate(const StateVec* StateVector) const override;
        };

        class IndexInterpreter : public LValueInterpreter
        {
        private:
            LValueInterpreter* ArrayInterp;
            RValueInterpreter* IndexInterp;
            u32 ElemSize;
            bool IndexSymmetric;
            bool IndexRange;
            i64 IndexRangeLow;
            i64 IndexRangeHigh;

        public:
            IndexInterpreter(LValueInterpreter* ArrayInterp,
                             RValueInterpreter* IndexInterp, ExpPtrT Exp);
            virtual ~IndexInterpreter();

            bool IsScalar() const;

            virtual i64 Evaluate(const StateVec* StateVector) const override;
            virtual UpdateStatusT Write(i64 Value, const StateVec* InStateVector,
                                        StateVec* StateVector, ExpT& NEPred) const override;
            virtual i64 GetOffset(const StateVec* StateVector) const override;
        };

        class FieldInterpreter : public LValueInterpreter
        {
        private:
            LValueInterpreter* RecInterp;
            u32 FieldOffset;

        public:
            FieldInterpreter(LValueInterpreter* RecInterp,
                             u32 FieldOffset, ExpPtrT Exp);
            virtual ~FieldInterpreter();

            virtual i64 Evaluate(const StateVec* StateVector) const override;
            virtual UpdateStatusT Write(i64 Value, const StateVec* InStateVector,
                                        StateVec* OutStateVector, ExpT& NEPred) const override;
            virtual i64 GetOffset(const StateVec* StateVector) const override;
        };

        class LTSCompiler
        {
        private:
            vector<RValueInterpreter*> RegisteredInterps;
            unordered_map<i64, vector<const UFInterpreter*>> UFInterpreters;

            inline bool HasMsgLValue(const ExpT& Exp, LabelledTS* TheLTS);

            inline vector<LTSAssignRef>
            ArrayTransformAssignments(const vector<LTSAssignRef>& Updates, MgrT* Mgr) const;
            LTSAssignRef ArrayTransformAssignment(const ExpT& LHS,
                                                  const ExpT& RHS,
                                                  const MgrT::SubstMapT& AccumMap) const;


        public:
            LTSCompiler();
            ~LTSCompiler();

            void RegisterInterp(RValueInterpreter* Interp);
            void CompileExp(const ExpT& Exp, LabelledTS* TheLTS);
            void CompileLTS(LabelledTS* TheLTS);
            void UpdateModel(const Z3Model& Model,
                             const unordered_set<i64>& InterpretedOps,
                             const unordered_map<i64, ExpT>& IndicatorExps);
            const unordered_map<i64, vector<const UFInterpreter*>>& GetUFInterpreters() const;
        };

    } /* end namespace MC */
} /* end namespace ESMC */

#endif /* ESMC_COMPILER_HPP_ */

//
// Compiler.hpp ends here
