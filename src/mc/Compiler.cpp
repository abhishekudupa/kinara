// Compiler.cpp ---
//
// Filename: Compiler.cpp
// Author: Abhishek Udupa
// Created: Mon Aug 18 12:02:24 2014 (-0400)
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
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLD ''AS IS'' AND ANY
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

#include <unordered_map>
#include <boost/lexical_cast.hpp>

#include "../uflts/LabelledTS.hpp"
#include "../utils/SizeUtils.hpp"
#include "../uflts/LTSTransitions.hpp"
#include "../uflts/LTSAssign.hpp"
#include "../tpinterface/TheoremProver.hpp"
#include "../uflts/LTSFairnessSet.hpp"
#include "../uflts/LTSUtils.hpp"
#include "../uflts/LTSTransformers.hpp"

#include "Compiler.hpp"
#include "StateVec.hpp"
#include "LTSChecker.hpp"

namespace ESMC {
    namespace MC {

        using namespace ESMC::LTS;
        using ESMC::LTS::Detail::ArrayRValueTransformer;

        const i64 ExceptionValue = INT64_MAX;

        OffsetCompiler::OffsetCompiler()
            : VisitorBaseT("OffsetCompiler")
        {
            // Nothing here
        }

        OffsetCompiler::~OffsetCompiler()
        {
            // Nothing here
        }

        void OffsetCompiler::VisitVarExpression(const VarExpT* Exp)
        {
            if (Exp->ExtensionData.Offset != -1) {
                return;
            }
            if (Exp->GetVarType()->Is<FieldAccessType>()) {
                return;
            }
            throw InternalError((string)"Variable without offset encountered: " +
                                Exp->ToString() + "\n.At: " + __FILE__ + ":" +
                                to_string(__LINE__));
        }

        void OffsetCompiler::VisitBoundVarExpression(const BoundVarExpT* Exp)
        {
            throw InternalError((string)"Bound var was not expected in compilation. " +
                                "This should have been unrolled. Expression:\n" +
                                Exp->ToString() + "\nAt: " + __FILE__ + ":" +
                                to_string(__LINE__));
        }

        void OffsetCompiler::VisitConstExpression(const ConstExpT* Exp)
        {
            if (Exp->ExtensionData.ConstCompiled) {
                return;
            }

            auto Type = Exp->GetConstType();
            auto const& ConstVal = Exp->GetConstValue();
            auto Val = Type->As<ScalarType>()->ConstToVal(ConstVal);
            Exp->ExtensionData.ConstVal = Val;
            Exp->ExtensionData.ConstCompiled = true;
        }

        void OffsetCompiler::VisitEQuantifiedExpression(const EQExpT* Exp)
        {
            throw InternalError((string)"Quantified expr was not expected in compilation. " +
                                "This should have been unrolled. Expression:\n" +
                                Exp->ToString() + "\nAt: " + __FILE__ + ":" +
                                to_string(__LINE__));
        }

        void OffsetCompiler::VisitAQuantifiedExpression(const AQExpT* Exp)
        {
            throw InternalError((string)"Quantified expr was not expected in compilation. " +
                                "This should have been unrolled. Expression:\n" +
                                Exp->ToString() + "\nAt: " + __FILE__ + ":" +
                                to_string(__LINE__));
        }

        void OffsetCompiler::VisitOpExpression(const OpExpT* Exp)
        {
            if (Exp->ExtensionData.Offset != -1) {
                return;
            }

            VisitorBaseT::VisitOpExpression(Exp);
            auto OpCode = Exp->GetOpCode();
            auto const& Children = Exp->GetChildren();

            if (OpCode == LTSOps::OpIndex) {
                if (Children[0]->ExtensionData.Offset != -1 &&
                    Children[1]->Is<ConstExpression>()) {
                    auto ArrType = Children[0]->GetType()->As<ArrayType>();
                    auto IndexType = ArrType->GetIndexType();
                    auto ValueType = ArrType->GetValueType();
                    auto ValueSize = ValueType->GetByteSize();
                    ValueSize = Align(ValueSize, ValueSize);

                    if (IndexType->Is<SymmetricType>()) {
                        Exp->ExtensionData.Offset =
                            Children[0]->ExtensionData.Offset +
                            (ValueSize * (Children[1]->ExtensionData.ConstVal - 1));
                    } else if (IndexType->Is<RangeType>()) {
                        auto IndexTypeAsRange = IndexType->SAs<RangeType>();
                        auto IndexRangeLow = IndexTypeAsRange->GetLow();
                        Exp->ExtensionData.Offset =
                            Children[0]->ExtensionData.Offset +
                            (ValueSize * (Children[1]->ExtensionData.ConstVal - IndexRangeLow));
                    } else {
                        Exp->ExtensionData.Offset =
                            Children[0]->ExtensionData.Offset +
                            (ValueSize * (Children[1]->ExtensionData.ConstVal));
                    }
                    return;
                }
            }

            if (OpCode == LTSOps::OpField) {
                auto ChildAsVar = Children[1]->As<VarExpression>();
                auto const& FieldName = ChildAsVar->GetVarName();
                auto RecType = Children[0]->GetType()->As<RecordType>();
                Exp->ExtensionData.FieldOffset = RecType->GetFieldOffset(FieldName);

                if (Children[0]->ExtensionData.Offset != -1) {
                    Exp->ExtensionData.Offset = Children[0]->ExtensionData.Offset +
                        Exp->ExtensionData.FieldOffset;
                }
            }
        }

        void OffsetCompiler::Do(const ExpT& Exp)
        {
            OffsetCompiler TheCompiler;
            Exp->Accept(&TheCompiler);
        }

        // Interpreters
        RValueInterpreter::RValueInterpreter(ExpPtrT Exp, const ExpT& NoExceptionPredicate)
            : Exp(Exp), NoExceptionPredicate(NoExceptionPredicate),
              TrueExp(Exp->GetMgr()->MakeTrue())
        {
            // Nothing here
        }

        RValueInterpreter::~RValueInterpreter()
        {
            // Nothing here
        }

        inline void RValueInterpreter::SetNoExceptionPred(const ExpT& NewPred)
        {
            if (NewPred != ExpT::NullPtr) {
                auto Mgr = NewPred->GetMgr();
                NoExceptionPredicate = ArrayRValueTransformer::Do(Mgr, NewPred);
                NoExceptionPredicate = Mgr->SimplifyFP(NoExceptionPredicate);
            } else {
                this->NoExceptionPredicate = NewPred;
            }
        }

        ExpPtrT RValueInterpreter::GetExp() const
        {
            return Exp;
        }

        const ExpT& RValueInterpreter::GetNoExceptionPredicate() const
        {
            if (NoExceptionPredicate != ExpT::NullPtr) {
                return NoExceptionPredicate;
            } else {
                return TrueExp;
            }
        }

        void RValueInterpreter::MakeInterpreter(const ExpT& Exp, LTSCompiler* Compiler)
        {
            auto& Ext = Exp->ExtensionData;
            if (Ext.Interp != nullptr) {
                return;
            }

            // Constant?
            if (Ext.ConstCompiled) {
                Ext.Interp = new CompiledConstInterpreter(Ext.ConstVal, Exp);
                Compiler->RegisterInterp(Ext.Interp);
                return;
            }

            // Compiled LValue
            if (Ext.Offset != -1) {
                Ext.Interp = new CompiledLValueInterpreter(Ext.Offset, Exp);
                Compiler->RegisterInterp(Ext.Interp);
                return;
            }

            if (Exp->GetType()->Is<FieldAccessType>()) {
                return;
            }

            // We need to build an interpreter for these
            auto ExpAsOp = Exp->As<Exprs::OpExpression>();
            if (ExpAsOp == nullptr) {
                throw InternalError((string)"Unexpected expression type in compilation:\n" +
                                    Exp->ToString() + "\nAt: " + __FILE__ + ":" +
                                    to_string(__LINE__));
            }

            auto OpCode = ExpAsOp->GetOpCode();
            auto const& Children = ExpAsOp->GetChildren();

            vector<RValueInterpreter*> SubInterps;
            for (auto const& Child : Children) {
                MakeInterpreter(Child, Compiler);
                SubInterps.push_back(Child->ExtensionData.Interp);
            }

            switch (OpCode) {
            case LTSOps::OpEQ:
                Ext.Interp = new EQInterpreter(SubInterps, Exp);
                break;
            case LTSOps::OpNOT:
                Ext.Interp = new NOTInterpreter(SubInterps, Exp);
                break;
            case LTSOps::OpITE:
                Ext.Interp = new ITEInterpreter(SubInterps, Exp);
                break;
            case LTSOps::OpOR:
                Ext.Interp = new ORInterpreter(SubInterps, Exp);
                break;
            case LTSOps::OpAND:
                Ext.Interp = new ANDInterpreter(SubInterps, Exp);
                break;
            case LTSOps::OpIMPLIES:
                Ext.Interp = new IMPLIESInterpreter(SubInterps, Exp);
                break;
            case LTSOps::OpIFF:
                Ext.Interp = new IFFInterpreter(SubInterps, Exp);
                break;
            case LTSOps::OpXOR:
                Ext.Interp = new XORInterpreter(SubInterps, Exp);
                break;
            case LTSOps::OpADD:
                Ext.Interp = new ADDInterpreter(SubInterps, Exp);
                break;
            case LTSOps::OpSUB:
                Ext.Interp = new SUBInterpreter(SubInterps, Exp);
                break;
            case LTSOps::OpMINUS:
                Ext.Interp = new MINUSInterpreter(SubInterps, Exp);
                break;
            case LTSOps::OpMUL:
                Ext.Interp = new MULInterpreter(SubInterps, Exp);
                break;
            case LTSOps::OpDIV:
                Ext.Interp = new DIVInterpreter(SubInterps, Exp);
                break;
            case LTSOps::OpMOD:
                Ext.Interp = new MODInterpreter(SubInterps, Exp);
                break;
            case LTSOps::OpGT:
                Ext.Interp = new GTInterpreter(SubInterps, Exp);
                break;
            case LTSOps::OpGE:
                Ext.Interp = new GEInterpreter(SubInterps, Exp);
                break;
            case LTSOps::OpLT:
                Ext.Interp = new LTInterpreter(SubInterps, Exp);
                break;
            case LTSOps::OpLE:
                Ext.Interp = new LEInterpreter(SubInterps, Exp);
                break;
            case LTSOps::OpIndex:
                if (!SubInterps[0]->Is<LValueInterpreter>()) {
                    throw InternalError((string)"Expected an LValueInterpreter.\nAt: " +
                                        __FILE__ + ":" + to_string(__LINE__));
                }
                Ext.Interp = new IndexInterpreter(SubInterps[0]->SAs<LValueInterpreter>(),
                                                  SubInterps[1], Exp);
                break;

            case LTSOps::OpField: {
                if (!SubInterps[0]->Is<LValueInterpreter>()) {
                    throw InternalError((string)"Expected an LValueInterpreter.\nAt: " +
                                        __FILE__ + ":" + to_string(__LINE__));
                }
                auto const& RecType = Children[0]->GetType()->SAs<RecordType>();
                auto const& FieldName = Children[1]->SAs<VarExpression>()->GetVarName();
                auto FieldOffset = RecType->GetFieldOffset(FieldName);

                Ext.Interp = new FieldInterpreter(SubInterps[0]->SAs<LValueInterpreter>(),
                                                  FieldOffset, Exp);

            }
                break;

            default: {
                // must be an uninterpreted function
                Ext.Interp = new UFInterpreter(SubInterps, Exp);
            }
            }

            Compiler->RegisterInterp(Ext.Interp);
        }

        // default action: do nothing
        void RValueInterpreter::UpdateModel(const Z3Model& Model,
                                            const unordered_set<i64>& InterpretedOps,
                                            const unordered_map<i64, ExpT>& IndicatorExps) const
        {
            return;
        }

        LValueInterpreter::LValueInterpreter(ExpPtrT Exp, const ExpT& NoExceptionPredicate)
            : RValueInterpreter(Exp, NoExceptionPredicate)
        {
            auto const& Type  = Exp->GetType();
            Scalar = Type->Is<ScalarType>();
            if (!Scalar) {
                Size = 0;
                High = INT64_MAX;
                Low = INT64_MIN;
            } else {
                Size = Type->GetByteSize();
                if (Type->Is<RangeType>()) {
                    auto TypeAsRange = Type->SAs<RangeType>();
                    Low = TypeAsRange->GetLow();
                    High = TypeAsRange->GetHigh();
                } else {
                    Low = 0;
                    High = Type->GetCardinality() - 1;
                }
            }
        }

        LValueInterpreter::~LValueInterpreter()
        {
            // Nothing here
        }

        i64 LValueInterpreter::GetLow() const
        {
            return Low;
        }

        i64 LValueInterpreter::GetHigh() const
        {
            return High;
        }

        bool LValueInterpreter::IsScalar() const
        {
            return Scalar;
        }

        u32 LValueInterpreter::GetSize() const
        {
            return Size;
        }

        UpdateStatusT LValueInterpreter::Update(const RValueInterpreter* RHS,
                                                const StateVec *InStateVector,
                                                StateVec *OutStateVector,
                                                ExpT& NEPred) const
        {
            auto NewVal = RHS->Evaluate(InStateVector);
            if (NewVal == ExceptionValue) {
                NEPred = RHS->GetNoExceptionPredicate();
                return UpdateStatusT::EvalException;
            }
            return Write(NewVal, InStateVector, OutStateVector, NEPred);
        }

        CompiledConstInterpreter::CompiledConstInterpreter(i64 Value, ExpPtrT Exp)
            : RValueInterpreter(Exp, ExpT::NullPtr), Value(Value)
        {
            // Nothing here
        }

        CompiledConstInterpreter::~CompiledConstInterpreter()
        {
            // Nothing here
        }

        i64 CompiledConstInterpreter::Evaluate(const StateVec* StateVector) const
        {
            return Value;
        }

        CompiledLValueInterpreter::CompiledLValueInterpreter(u32 Offset, ExpPtrT Exp)
            : LValueInterpreter(Exp, ExpT::NullPtr), Offset(Offset)
        {
            // Nothing here
        }

        CompiledLValueInterpreter::~CompiledLValueInterpreter()
        {
            // Nothing here
        }

        i64 CompiledLValueInterpreter::Evaluate(const StateVec* StateVector) const
        {
            if (!Scalar) {
                throw InternalError((string)"Evaluate() called on non-scalar lvalue" +
                                    "\nAt: " + __FILE__ + ":" + to_string(__LINE__));
            }
            i64 RawVal = 0;
            if (Size == 1) {
                RawVal = StateVector->ReadByte(Offset);
            } else if (Size == 2) {
                RawVal = StateVector->ReadShort(Offset);
            } else if (Size == 4) {
                RawVal = StateVector->ReadWord(Offset);
            }
            return RawVal + Low;
        }

        UpdateStatusT CompiledLValueInterpreter::Write(i64 Value, const StateVec* InStateVector,
                                                       StateVec* StateVector, ExpT& NEPred) const
        {
            i64 RawValue = 0;
            if (!Scalar) {
                throw InternalError((string)"Write() called on non-scalar type");
            }

            if (Value < Low || Value > High) {
                return UpdateStatusT::BoundsViolation;
            } else {
                RawValue = Value - Low;
            }
            if (Size == 1) {
                StateVector->WriteByte(Offset, (u08)RawValue);
            } else if (Size == 2) {
                StateVector->WriteShort(Offset, (u16)RawValue);
            } else if (Size == 4) {
                StateVector->WriteWord(Offset, (u32)RawValue);
            }

            return UpdateStatusT::UpdateOK;
        }

        i64 CompiledLValueInterpreter::GetOffset(const StateVec* StateVector) const
        {
            return Offset;
        }

        u32 CompiledLValueInterpreter::GetOffset() const
        {
            return Offset;
        }

        // Uninterpreted function interpreter implementation
        UFInterpreter::UFInterpreter(const vector<RValueInterpreter*>& ArgInterps,
                                     ExpPtrT Exp)
            : RValueInterpreter(Exp, ExpT::NullPtr),
              ArgInterps(ArgInterps), SubEvals(ArgInterps.size()),
              NumArgInterps(ArgInterps.size()), Model(Z3Model::NullModel),
              Enabled(false), MyOpCode(Exp->As<OpExpression>()->GetOpCode())
        {
            auto const& ExpType = Exp->GetType();
            if (ExpType->Is<RangeType>()) {
                auto ExpAsRange = ExpType->SAs<RangeType>();
                Low = ExpAsRange->GetLow();
                High = ExpAsRange->GetHigh();
            } else {
                Low = 0;
                High = ExpType->GetCardinality() - 1;
            }
        }

        UFInterpreter::~UFInterpreter()
        {
            // Nothing here
        }

        inline i64 UFInterpreter::DoEval() const
        {
            auto it = EvalMap.find(SubEvals);
            if (it != EvalMap.end()) {
                return it->second;
            }

            // We need to create a new entry
            auto Mgr = Exp->GetMgr();
            auto ExpAsOp = Exp->SAs<Exprs::OpExpression>();
            auto OpCode = ExpAsOp->GetOpCode();
            vector<ExpT> AppArgExps;

            for (u32 i = 0; i < NumArgInterps; ++i) {
                auto CurInterp = ArgInterps[i];
                auto ArgType = CurInterp->GetExp()->GetType()->As<ScalarType>();
                auto AppArgExp = Mgr->MakeVal(ArgType->ValToConst(SubEvals[i]),
                                              CurInterp->GetExp()->GetType());
                AppArgExps.push_back(AppArgExp);
            }

            auto ConcAppExp = Mgr->MakeExpr(OpCode, AppArgExps);

            // Evaluate the expression in the model
            Z3TPRef TP = Model.GetTPPtr();
            auto EvalExp = TP->Evaluate(ConcAppExp);
            auto EvalAsConst = EvalExp->As<Exprs::ConstExpression>();
            if (EvalAsConst == nullptr) {
                throw ESMCError((string)"Evaluating a term on the model did not " +
                                "result in a constant valued interpretation.\nTerm:\n" +
                                ConcAppExp->ToString() + "\nEvaluation:\n" + EvalExp->ToString());
            }
            auto RangeType = EvalExp->GetType()->As<ScalarType>();
            auto Val = RangeType->ConstToVal(EvalAsConst->GetConstValue());

            // cache the result
            EvalMap[SubEvals] = Val;
            return Val;
        }

        i64 UFInterpreter::Evaluate(const StateVec* StateVector) const
        {
            if (!Enabled) {
                return Low;
            }

            // We are guaranteed that range and domain are scalars
            for (u32 i = 0; i < NumArgInterps; ++i) {
                SubEvals[i] = ArgInterps[i]->Evaluate(StateVector);
            }
            return DoEval();
        }

        void UFInterpreter::UpdateModel(const Z3Model& Model,
                                        const unordered_set<i64>& InterpretedOps,
                                        const unordered_map<i64, ExpT>& AllFalsePreds) const
        {
            EvalMap.clear();
            if (InterpretedOps.find(MyOpCode) == InterpretedOps.end()) {
                Enabled = false;
            } else {
                auto it = AllFalsePreds.find(MyOpCode);
                if (it != AllFalsePreds.end()) {
                    // check if my all false pred is true
                    Z3TPRef TP = Model.GetTPPtr();
                    auto Res = TP->Evaluate(it->second);
                    auto ResAsConst = Res->As<Exprs::ConstExpression>();
                    if (ResAsConst == nullptr) {
                        throw ESMCError((string)"Evaluating an indicator variable on the model " +
                                        "did not result in a constant valued interpretation.\n" +
                                        "Term:\n" + it->second->ToString() +
                                        "\nEvaluation:\n" + Res->ToString());
                    }
                    if (ResAsConst->GetConstValue() == "false") {
                        Enabled = true;
                        this->Model = Model;
                    } else {
                        Enabled = false;
                    }
                } else {
                    this->Model = Model;
                    Enabled = true;
                }
            }
        }

        const UFInterpreter::EvalMapT&
        UFInterpreter::GetEvalMap() const
        {
            return EvalMap;
        }

        i64 UFInterpreter::GetOpCode() const
        {
            return MyOpCode;
        }

        bool UFInterpreter::IsEnabled() const
        {
            return Enabled;
        }

        OpInterpreter::OpInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                     ExpPtrT Exp)
            : RValueInterpreter(Exp, ExpT::NullPtr), SubInterps(SubInterps),
              SubEvals(SubInterps.size()), NumSubInterps(SubInterps.size())
        {
            // Nothing here
        }

        OpInterpreter::~OpInterpreter()
        {
            // Nothing here
        }

        inline void OpInterpreter::EvaluateSubInterps(const StateVec *StateVector) const
        {
            for (u32 i = 0; i < NumSubInterps; ++i) {
                SubEvals[i] = SubInterps[i]->Evaluate(StateVector);
            }
        }

        inline void OpInterpreter::SetNEPredAllDef()
        {
            auto Mgr = Exp->GetMgr();
            vector<ExpT> NEConjuncts(NumSubInterps);

            if (NumSubInterps == 0) {
                SetNoExceptionPred(ExpT::NullPtr);
            } else if (NumSubInterps == 1) {
                SetNoExceptionPred(SubInterps[0]->GetNoExceptionPredicate());
            } else {
                for (u32 i = 0; i < NumSubInterps; ++i) {
                    NEConjuncts[i] = SubInterps[i]->GetNoExceptionPredicate();
                }
                auto NEPred = Mgr->MakeExpr(LTSOps::OpAND, NEConjuncts);
                SetNoExceptionPred(NEPred);
            }
        }

        EQInterpreter::EQInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                     ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            SetNEPredAllDef();
        }

        EQInterpreter::~EQInterpreter()
        {
            // Nothing here
        }

        // All operands to eq must be scalar.
        // This is enforced by the Semanticizer
        i64 EQInterpreter::Evaluate(const StateVec* StateVector) const
        {
            return (SubInterps[0]->Evaluate(StateVector) ==
                    SubInterps[1]->Evaluate(StateVector));
        }

        NOTInterpreter::NOTInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                       ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            SetNEPredAllDef();
        }

        NOTInterpreter::~NOTInterpreter()
        {
            // Nothing here
        }

        i64 NOTInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            if (SubEvals[0] == ExceptionValue) {
                return ExceptionValue;
            } else if (SubEvals[0] == 0) {
                return 1;
            } else {
                return 0;
            }
        }

        ITEInterpreter::ITEInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                       ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            auto Mgr = Exp->GetMgr();
            auto NEPred1 = SubInterps[0]->GetNoExceptionPredicate();
            auto NEPred2 = Mgr->MakeExpr(LTSOps::OpIMPLIES, SubInterps[0]->GetExp(),
                                         SubInterps[1]->GetNoExceptionPredicate());
            auto NegCondition = Mgr->MakeExpr(LTSOps::OpNOT, SubInterps[0]->GetExp());
            auto NEPred3 = Mgr->MakeExpr(LTSOps::OpIMPLIES, NegCondition,
                                         SubInterps[2]->GetNoExceptionPredicate());
            auto NEPred = Mgr->MakeExpr(LTSOps::OpAND, NEPred1, NEPred2, NEPred3);
            SetNoExceptionPred(NEPred);
        }

        ITEInterpreter::~ITEInterpreter()
        {
            // Nothing here
        }

        // both branches are guaranteed to be scalar types
        // by the Semanticizer
        i64 ITEInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            if (SubEvals[0] == ExceptionValue) {
                return ExceptionValue;
            } else if (SubEvals[0] != 0) {
                return SubEvals[1];
            } else {
                return SubEvals[2];
            }
        }

        ORInterpreter::ORInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                     ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            auto Mgr = Exp->GetMgr();
            const u32 NumDisjuncts = SubInterps.size();

            vector<ExpT> NEDisjuncts(NumDisjuncts + 1);
            vector<ExpT> AllDefConjuncts(NumDisjuncts);

            for (u32 i = 0; i < NumDisjuncts; ++i) {
                NEDisjuncts[i] = Mgr->MakeExpr(LTSOps::OpAND,
                                               SubInterps[i]->GetNoExceptionPredicate(),
                                               SubInterps[i]->GetExp());
                AllDefConjuncts[i] = SubInterps[i]->GetNoExceptionPredicate();
            }

            NEDisjuncts[NumDisjuncts] = Mgr->MakeExpr(LTSOps::OpAND, AllDefConjuncts);
            auto NEPred = Mgr->MakeExpr(LTSOps::OpOR, NEDisjuncts);
            SetNoExceptionPred(NEPred);
        }

        ORInterpreter::~ORInterpreter()
        {
            // Nothing here
        }

        i64 ORInterpreter::Evaluate(const StateVec* StateVector) const
        {
            bool Exception = false;
            EvaluateSubInterps(StateVector);
            for (auto SubEval : SubEvals) {
                if (SubEval == ExceptionValue) {
                    Exception = true;
                } else if (SubEval != 0) {
                    return 1;
                }
            }

            return (Exception ? ExceptionValue : 0);
        }

        ANDInterpreter::ANDInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                       ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            auto Mgr = Exp->GetMgr();
            const u32 NumDisjuncts = SubInterps.size();

            vector<ExpT> NEDisjuncts(NumDisjuncts + 1);
            vector<ExpT> AllDefConjuncts(NumDisjuncts);

            for (u32 i = 0; i < NumDisjuncts; ++i) {
                NEDisjuncts[i] =
                    Mgr->MakeExpr(LTSOps::OpAND, SubInterps[i]->GetNoExceptionPredicate(),
                                  Mgr->MakeExpr(LTSOps::OpNOT, SubInterps[i]->GetExp()));
                AllDefConjuncts[i] = SubInterps[i]->GetNoExceptionPredicate();
            }
            NEDisjuncts[NumDisjuncts] = Mgr->MakeExpr(LTSOps::OpAND, AllDefConjuncts);
            auto NEPred = Mgr->MakeExpr(LTSOps::OpOR, NEDisjuncts);
            SetNoExceptionPred(NEPred);
        }

        ANDInterpreter::~ANDInterpreter()
        {
            // Nothing here
        }

        i64 ANDInterpreter::Evaluate(const StateVec* StateVector) const
        {
            bool Exception = false;
            EvaluateSubInterps(StateVector);
            for (auto SubEval : SubEvals) {
                if (SubEval == ExceptionValue) {
                    Exception = true;
                } else if (SubEval == 0) {
                    return 0;
                }
            }

            return (Exception ? ExceptionValue : 1);
        }

        IMPLIESInterpreter::IMPLIESInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                               ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            auto Mgr = Exp->GetMgr();

            auto NEPred1 = Mgr->MakeExpr(LTSOps::OpAND, SubInterps[0]->GetNoExceptionPredicate(),
                                         Mgr->MakeExpr(LTSOps::OpNOT, SubInterps[0]->GetExp()));
            auto NEPred2 = Mgr->MakeExpr(LTSOps::OpAND, SubInterps[1]->GetNoExceptionPredicate(),
                                         SubInterps[1]->GetExp());
            auto NEPred3 = Mgr->MakeExpr(LTSOps::OpAND, SubInterps[0]->GetNoExceptionPredicate(),
                                         SubInterps[1]->GetNoExceptionPredicate());
            auto NEPred = Mgr->MakeExpr(LTSOps::OpOR, NEPred1, NEPred2, NEPred3);
            SetNoExceptionPred(NEPred);
        }

        IMPLIESInterpreter::~IMPLIESInterpreter()
        {
            // Nothing here
        }

        i64 IMPLIESInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            if (SubEvals[0] == ExceptionValue) {
                if (SubEvals[1] != ExceptionValue && SubEvals[1] != 0) {
                    return 1;
                } else {
                    return ExceptionValue;
                }
            } else if (SubEvals[0] == 0) {
                return 1;
            } else {
                // antecedent is true
                if (SubEvals[1] == ExceptionValue) {
                    return ExceptionValue;
                } else {
                    return SubEvals[1];
                }
            }
        }

        IFFInterpreter::IFFInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                       ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            SetNEPredAllDef();
        }

        IFFInterpreter::~IFFInterpreter()
        {
            // Nothing here
        }

        i64 IFFInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            if (SubEvals[0] == ExceptionValue || SubEvals[1] == ExceptionValue) {
                return ExceptionValue;
            } else {
                return (SubEvals[0] == SubEvals[1]);
            }
        }

        XORInterpreter::XORInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                       ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            SetNEPredAllDef();
        }

        XORInterpreter::~XORInterpreter()
        {
            // Nothing here
        }

        i64 XORInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            if (SubEvals[0] == ExceptionValue || SubEvals[1] == ExceptionValue) {
                return ExceptionValue;
            } else {
                return (SubEvals[0] != SubEvals[1]);
            }
        }

        ADDInterpreter::ADDInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                       ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            SetNEPredAllDef();
        }

        ADDInterpreter::~ADDInterpreter()
        {
            // Nothing here
        }

        i64 ADDInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            i64 Retval = 0;

            for (auto SubEval : SubEvals) {
                Retval += SubEval;
                if (SubEval == ExceptionValue) {
                    return ExceptionValue;
                }
            }
            return Retval;
        }

        SUBInterpreter::SUBInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                       ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            SetNEPredAllDef();
        }

        SUBInterpreter::~SUBInterpreter()
        {
            // Nothing here
        }

        i64 SUBInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);

            i64 Retval = 0;
            bool First = true;
            for (auto SubEval : SubEvals) {
                if (First) {
                    Retval += SubEval;
                    First = false;
                } else {
                    Retval -= SubEval;
                }
                if (SubEval == ExceptionValue) {
                    return ExceptionValue;
                }
            }
            return Retval;
        }

        MINUSInterpreter::MINUSInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                           ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            SetNEPredAllDef();
        }

        MINUSInterpreter::~MINUSInterpreter()
        {
            // Nothing here
        }

        i64 MINUSInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            if (SubEvals[0] == ExceptionValue) {
                return ExceptionValue;
            }

            i64 Retval = 0;
            Retval = Retval - SubEvals[0];
            return Retval;
        }

        MULInterpreter::MULInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                       ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            SetNEPredAllDef();
        }

        MULInterpreter::~MULInterpreter()
        {
            // Nothing here
        }

        i64 MULInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            i64 Retval = 1;
            for (auto SubEval : SubEvals) {
                Retval *= SubEval;
                if (SubEval == ExceptionValue) {
                    return ExceptionValue;
                }
            }
            return Retval;
        }

        DIVInterpreter::DIVInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                       ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            SetNEPredAllDef();
            // Also the second argument cannot be zero
            auto Mgr = Exp->GetMgr();

            auto const& Divisor = SubInterps[1]->GetExp();
            auto NonZero = Mgr->MakeExpr(LTSOps::OpNOT,
                                         Mgr->MakeExpr(LTSOps::OpEQ,
                                                       Divisor,
                                                       Mgr->MakeVal("0", Divisor->GetType())));
            auto NEPred = Mgr->MakeExpr(LTSOps::OpAND, NoExceptionPredicate, NonZero);
            SetNoExceptionPred(NEPred);
        }

        DIVInterpreter::~DIVInterpreter()
        {
            // Nothing here
        }

        i64 DIVInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            if (SubEvals[0] == ExceptionValue || SubEvals[1] == ExceptionValue) {
                return ExceptionValue;
            }
            if (SubEvals[1] == 0) {
                return ExceptionValue;
            }
            return (SubEvals[0] / SubEvals[1]);
        }

        MODInterpreter::MODInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                       ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            SetNEPredAllDef();
            // Also the second argument cannot be zero
            auto Mgr = Exp->GetMgr();

            auto const& Divisor = SubInterps[1]->GetExp();
            auto NonZero = Mgr->MakeExpr(LTSOps::OpNOT,
                                         Mgr->MakeExpr(LTSOps::OpEQ,
                                                       Divisor,
                                                       Mgr->MakeVal("0", Divisor->GetType())));
            auto NEPred = Mgr->MakeExpr(LTSOps::OpAND, NoExceptionPredicate, NonZero);
            SetNoExceptionPred(NEPred);
        }

        MODInterpreter::~MODInterpreter()
        {
            // Nothing here
        }

        i64 MODInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            if (SubEvals[0] == ExceptionValue || SubEvals[1] == ExceptionValue) {
                return ExceptionValue;
            }
            if (SubEvals[1] == 0) {
                return ExceptionValue;
            }
            return (SubEvals[0] % SubEvals[1]);
        }

        GTInterpreter::GTInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                     ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            SetNEPredAllDef();
        }

        GTInterpreter::~GTInterpreter()
        {
            // Nothing here
        }

        i64 GTInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            if (SubEvals[0] == ExceptionValue || SubEvals[1] == ExceptionValue) {
                return ExceptionValue;
            }
            return (SubEvals[0] > SubEvals[1]);
        }

        GEInterpreter::GEInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                     ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            SetNEPredAllDef();
        }

        GEInterpreter::~GEInterpreter()
        {
            // Nothing here
        }

        i64 GEInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            if (SubEvals[0] == ExceptionValue || SubEvals[1] == ExceptionValue) {
                return ExceptionValue;
            }
            return (SubEvals[0] >= SubEvals[1]);
        }

        LTInterpreter::LTInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                     ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            SetNEPredAllDef();
        }

        LTInterpreter::~LTInterpreter()
        {
            // Nothing here
        }

        i64 LTInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            if (SubEvals[0] == ExceptionValue || SubEvals[1] == ExceptionValue) {
                return ExceptionValue;
            }
            return (SubEvals[0] < SubEvals[1]);
        }

        LEInterpreter::LEInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                     ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            SetNEPredAllDef();
        }

        LEInterpreter::~LEInterpreter()
        {
            // Nothing here
        }

        i64 LEInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            if (SubEvals[0] == ExceptionValue || SubEvals[1] == ExceptionValue) {
                return ExceptionValue;
            }
            return (SubEvals[0] <= SubEvals[1]);
        }

        IndexInterpreter::IndexInterpreter(LValueInterpreter* ArrayInterp,
                                           RValueInterpreter* IndexInterp,
                                           ExpPtrT Exp)
            : LValueInterpreter(Exp, ExpT::NullPtr),
              ArrayInterp(ArrayInterp), IndexInterp(IndexInterp)
        {
            auto const& ArrType = ArrayInterp->GetExp()->GetType();
            auto const& ActualIndexType = ArrType->SAs<ArrayType>()->GetIndexType();

            ElemSize = ArrType->SAs<ArrayType>()->GetValueType()->GetByteSize();
            ElemSize = Align(ElemSize, ElemSize);

            IndexSymmetric = ActualIndexType->Is<SymmetricType>();
            IndexRange = ActualIndexType->Is<RangeType>();
            if (IndexRange) {
                auto IndexAsRange = ActualIndexType->SAs<RangeType>();
                IndexRangeLow = IndexAsRange->GetLow();
                IndexRangeHigh = IndexAsRange->GetHigh();
            }

            auto NEPred1 = ArrayInterp->GetNoExceptionPredicate();
            auto NEPred2 = IndexInterp->GetNoExceptionPredicate();
            ExpT NEPred3 = ExpT::NullPtr;
            auto const& IndexExp = IndexInterp->GetExp();

            auto Mgr = Exp->GetMgr();

            if (IndexSymmetric) {
                auto ClearValue = ActualIndexType->GetClearValue();
                NEPred3 = Mgr->MakeExpr(LTSOps::OpNOT,
                                        Mgr->MakeExpr(LTSOps::OpEQ,
                                                      IndexExp,
                                                      Mgr->MakeVal(ClearValue, ActualIndexType)));
            } else if (IndexRange) {
                NEPred3 = Mgr->MakeExpr(LTSOps::OpAND,
                                        Mgr->MakeExpr(LTSOps::OpGE,
                                                      IndexExp,
                                                      Mgr->MakeVal(to_string(IndexRangeLow),
                                                                   ActualIndexType)),
                                        Mgr->MakeExpr(LTSOps::OpLE,
                                                      IndexExp,
                                                      Mgr->MakeVal(to_string(IndexRangeHigh),
                                                                   ActualIndexType)));
            }

            auto NEPred = Mgr->MakeExpr(LTSOps::OpAND, NEPred1, NEPred2, NEPred3);
            SetNoExceptionPred(NEPred3);
        }

        IndexInterpreter::~IndexInterpreter()
        {
            // Nothing here
        }

        i64 IndexInterpreter::Evaluate(const StateVec* StateVector) const
        {
            if (!Scalar) {
                throw ESMCError((string)"Evaluate() called on non-scalar type");
            }

            i64 Offset = GetOffset(StateVector);
            if (Offset == ExceptionValue) {
                return ExceptionValue;
            }

            auto BasePtr = StateVector->GetStateBuffer();
            auto DstPtr = BasePtr + Offset;

            i64 RawVal = 0;
            if (Size == 1) {
                RawVal = *DstPtr;
            } else if (Size == 2) {
                RawVal = *((u16*)(DstPtr));
            } else {
                RawVal = *((u32*)(DstPtr));
            }

            return RawVal + Low;
        }

        UpdateStatusT IndexInterpreter::Write(i64 Value, const StateVec* InStateVector,
                                              StateVec* StateVector, ExpT& NEPred) const
        {
            if (!Scalar) {
                throw ESMCError((string)"WriteScalar() called on non-scalar type");
            }

            auto Offset = GetOffset(InStateVector);
            if (Offset == ExceptionValue) {
                NEPred = NoExceptionPredicate;
                return UpdateStatusT::EvalException;
            }

            auto BasePtr = StateVector->GetStateBuffer();
            auto DstPtr = BasePtr + Offset;

            if (Value < Low || Value > High) {
                return UpdateStatusT::BoundsViolation;
            }

            i64 RawVal = Value - Low;

            if (Size == 1) {
                *((u08*)(DstPtr)) = (u08)RawVal;
            } else if (Size == 2) {
                *((u16*)(DstPtr)) = (u16)RawVal;
            } else {
                *((u32*)(DstPtr)) = (u32)RawVal;
            }

            return UpdateStatusT::UpdateOK;
        }

        i64 IndexInterpreter::GetOffset(const StateVec* StateVector) const
        {
            auto BaseOffset = ArrayInterp->GetOffset(StateVector);
            if (BaseOffset == ExceptionValue) {
                return ExceptionValue;
            }
            auto IndexValue = IndexInterp->Evaluate(StateVector);
            if (IndexSymmetric) {
                if (IndexValue == 0) {
                    return ExceptionValue;
                } else {
                    IndexValue = IndexValue - 1;
                }
            } else if (IndexRange) {
                if (IndexValue < IndexRangeLow || IndexValue > IndexRangeHigh) {
                    return ExceptionValue;
                } else {
                    IndexValue = IndexValue - IndexRangeLow;
                }
            }

            return BaseOffset + (IndexValue * ElemSize);
        }

        FieldInterpreter::FieldInterpreter(LValueInterpreter* RecInterp,
                                           u32 FieldOffset, ExpPtrT Exp)
            : LValueInterpreter(Exp, ExpT::NullPtr),
              RecInterp(RecInterp), FieldOffset(FieldOffset)
        {
            SetNoExceptionPred(RecInterp->GetNoExceptionPredicate());
        }

        FieldInterpreter::~FieldInterpreter()
        {
            // Nothing here
        }

        i64 FieldInterpreter::Evaluate(const StateVec* StateVector) const
        {
            if (!Scalar) {
                throw ESMCError((string)"Evaluate() called on non-scalar type");
            }

            auto BasePtr = StateVector->GetStateBuffer();

            auto Offset = GetOffset(StateVector);
            if (Offset == ExceptionValue) {
                return ExceptionValue;
            }
            auto DstPtr = BasePtr + Offset;

            i64 RawVal = 0;
            if (Size == 1) {
                RawVal = (*DstPtr);
            } else if (Size == 2) {
                RawVal = *((u16*)(DstPtr));
            } else {
                RawVal = *((u32*)(DstPtr));
            }

            return RawVal + Low;
        }

        i64 FieldInterpreter::GetOffset(const StateVec* StateVector) const
        {
            auto BaseOffset = RecInterp->GetOffset(StateVector);
            if (BaseOffset == ExceptionValue) {
                return ExceptionValue;
            }
            return BaseOffset + FieldOffset;
        }

        UpdateStatusT FieldInterpreter::Write(i64 Value, const StateVec* InStateVector,
                                              StateVec* StateVector, ExpT& NEPred) const
        {
            if (!Scalar) {
                throw ESMCError((string)"Write() called on non-scalar type");
            }

            if (Value < Low || Value > High) {
                return UpdateStatusT::BoundsViolation;
            }

            i64 RawVal = Value - Low;

            auto BasePtr = StateVector->GetStateBuffer();
            auto Offset = GetOffset(InStateVector);

            if (Offset == ExceptionValue) {
                NEPred = NoExceptionPredicate;
                return UpdateStatusT::EvalException;
            }
            auto DstPtr = BasePtr + Offset;

            if (Size == 1) {
                *((u08*)(DstPtr)) = (u08)RawVal;
            } else if (Size == 2) {
                *((u16*)(DstPtr)) = (u16)RawVal;
            } else {
                *((u32*)(DstPtr)) = (u32)RawVal;
            }

            return UpdateStatusT::UpdateOK;
        }

        LTSCompiler::LTSCompiler()
        {
            // Nothing here
        }

        LTSCompiler::~LTSCompiler()
        {
            for (auto Interp : RegisteredInterps) {
                delete Interp;
            }
        }

        void LTSCompiler::RegisterInterp(RValueInterpreter *Interp)
        {
            RegisteredInterps.push_back(Interp);
            if (Interp->Is<UFInterpreter>()) {
                auto AsUFInterp = Interp->SAs<UFInterpreter>();
                auto OpCode = AsUFInterp->GetOpCode();
                UFInterpreters[OpCode].push_back(AsUFInterp);
            }
        }

        void LTSCompiler::CompileExp(const ExpT& Exp, LabelledTS* TheLTS)
        {
            OffsetCompiler::Do(Exp);
            RValueInterpreter::MakeInterpreter(Exp, this);
        }

        void LTSCompiler::CompileLTS(LabelledTS* TheLTS)
        {
            auto Mgr = TheLTS->GetMgr();
            for (auto const& GCmd : TheLTS->GuardedCommands) {
                CompileExp(GCmd->GetGuard(), TheLTS);
                CompileExp(GCmd->GetFixedInterpretation(), TheLTS);
                for (auto const& Update : GCmd->GetUpdates()) {
                    CompileExp(Update->GetLHS(), TheLTS);
                    CompileExp(Update->GetRHS(), TheLTS);
                }

                // Lower the guards and commands to a form
                // that we can symbolically execute without
                // losing our sanity

                auto LoweredGuard =
                    Mgr->ApplyTransform<LTS::Detail::ArrayRValueTransformer>(GCmd->GetGuard());
                LoweredGuard = Mgr->SimplifyFP(LoweredGuard);
                GCmd->SetLoweredGuard(LoweredGuard);

                for (auto const& Update : GCmd->GetUpdates()) {
                    auto LoweredBoundsConstraint =
                        Mgr->ApplyTransform<ArrayRValueTransformer>(Update->GetBoundsConstraint());
                    LoweredBoundsConstraint = Mgr->SimplifyFP(LoweredBoundsConstraint);
                    Update->SetLoweredBoundsConstraint(LoweredBoundsConstraint);
                }

                auto&& LoweredUpdates = ArrayTransformAssignments(GCmd->GetUpdates(), Mgr);
                GCmd->SetLoweredUpdates(LoweredUpdates);
            }

            CompileExp(TheLTS->InvariantExp, TheLTS);
            CompileExp(TheLTS->FinalCondExp, TheLTS);

            for (auto const& ChanBufferExp : TheLTS->ChanBuffersToSort) {
                CompileExp(get<1>(ChanBufferExp), TheLTS);
            }

            for (auto const& InitStateGen : TheLTS->InitStateGenerators) {
                for (auto const& Update : InitStateGen->GetUpdates()) {
                    CompileExp(Update->GetLHS(), TheLTS);
                    CompileExp(Update->GetRHS(), TheLTS);
                }

                auto&& LoweredUpdates = ArrayTransformAssignments(InitStateGen->GetUpdates(), Mgr);
                InitStateGen->SetLoweredUpdates(LoweredUpdates);
            }
        }

        LTSAssignRef LTSCompiler::ArrayTransformAssignment(const ExpT& LHS,
                                                           const ExpT& RHS,
                                                           const MgrT::SubstMapT& AccumMap)
        {
            auto Mgr = LHS->GetMgr();
            if (LHS->Is<VarExpression>()) {
                return new LTSAssignSimple(LHS, RHS);
            } else if (LHS->Is<OpExpression>()) {
                auto LHSAsOp = LHS->SAs<OpExpression>();
                auto OpCode = LHSAsOp->GetOpCode();
                auto const& Children = LHSAsOp->GetChildren();

                if (OpCode == LTSOps::OpIndex || OpCode == LTSOps::OpSelect) {
                    auto SubstLVal = Mgr->Substitute(AccumMap, Children[0]);
                    auto StoreExp = Mgr->MakeExpr(LTSOps::OpStore,
                                                  SubstLVal,
                                                  Children[1], RHS);
                    return ArrayTransformAssignment(Children[0], StoreExp, AccumMap);
                } else if (OpCode == LTSOps::OpField || OpCode == LTSOps::OpProject) {
                    auto SubstLVal = Mgr->Substitute(AccumMap, Children[0]);
                    auto UpdateExp = Mgr->MakeExpr(LTSOps::OpUpdate,
                                                   SubstLVal,
                                                   Children[1], RHS);
                    return ArrayTransformAssignment(Children[0], UpdateExp, AccumMap);
                } else {
                    throw InternalError((string)"Non LValue obtained for LHS in " +
                                        "LTSCompiler::ArrayTransformAssignment()");
                }
            } else {
                throw InternalError((string)"Non LValue obtained for LHS in " +
                                    "LTSCompiler::ArrayTransformAssignment()");
            }
        }

        inline vector<LTSAssignRef>
        LTSCompiler::ArrayTransformAssignments(const vector<LTSAssignRef>& Updates, MgrT* Mgr)
        {
            MgrT::SubstMapT AccumMap;
            for (auto const& Update : Updates) {
                auto XFormedLHS =
                    Mgr->ApplyTransform<ArrayRValueTransformer>(Update->GetLHS());
                auto XFormedRHS =
                    Mgr->ApplyTransform<ArrayRValueTransformer>(Update->GetRHS());
                auto XFormedUpdate = ArrayTransformAssignment(XFormedLHS, XFormedRHS, AccumMap);
                AccumMap[XFormedUpdate->GetLHS()] = Mgr->SimplifyFP(XFormedUpdate->GetRHS());
            }

            // Make the new set of updates by iterating over the
            // accumulated substitutions
            vector<LTSAssignRef> Retval;
            for (auto const& Subst : AccumMap) {
                auto LHS = Mgr->SimplifyFP(Subst.first);
                auto RHS = Mgr->SimplifyFP(Subst.second);
                Retval.push_back(new LTSAssignSimple(LHS, RHS));
            }
            return Retval;
        }

        void LTSCompiler::UpdateModel(const Z3Model& Model,
                                      const unordered_set<i64>& InterpretedOps,
                                      const unordered_map<i64, ExpT>& AllFalsePreds)
        {
            // Push the model through all the registered
            // interpreters
            for (auto const& Interp : RegisteredInterps) {
                Interp->UpdateModel(Model, InterpretedOps, AllFalsePreds);
            }
        }

        const unordered_map<i64, vector<const UFInterpreter*>>&
        LTSCompiler::GetUFInterpreters() const
        {
            return UFInterpreters;
        }

    } /* end namespace */
} /* end namespace */

//
// Compiler.cpp ends here
