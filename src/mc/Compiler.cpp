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

#include <boost/lexical_cast.hpp>

#include "../uflts/LabelledTS.hpp"
#include "../utils/SizeUtils.hpp"
#include "../uflts/LTSTransitions.hpp"
#include "../uflts/LTSAssign.hpp"
#include "../tpinterface/TheoremProver.hpp"
#include "../uflts/LTSFairnessSet.hpp"

#include "Compiler.hpp"
#include "StateVec.hpp"
#include "LTSChecker.hpp"

namespace ESMC {
    namespace MC {

        using namespace ESMC::LTS;

        const i64 UndefValue = INT64_MAX;

        OffsetCompiler::OffsetCompiler(const LabelledTS* TheLTS)
            : VisitorBaseT("OffsetCompiler"), TheLTS(TheLTS)
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
            if (Exp->GetVarType()->Is<ExprFieldAccessType>()) {
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
            auto Val = Type->As<ExprScalarType>()->ConstToVal(ConstVal);
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
                    auto ArrayType = Children[0]->GetType()->As<ExprArrayType>();
                    auto IndexType = ArrayType->GetIndexType();
                    auto ValueType = ArrayType->GetValueType();
                    auto ValueSize = ValueType->GetByteSize();
                    ValueSize = Align(ValueSize, ValueSize);
                    
                    if (IndexType->Is<ExprSymmetricType>()) {
                        Exp->ExtensionData.Offset = 
                            Children[0]->ExtensionData.Offset + 
                            (ValueSize * (Children[1]->ExtensionData.ConstVal - 1));
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
                auto RecType = Children[0]->GetType()->As<ExprRecordType>();
                Exp->ExtensionData.FieldOffset = RecType->GetFieldOffset(FieldName);

                if (Children[0]->ExtensionData.Offset != -1) {
                    Exp->ExtensionData.Offset = Children[0]->ExtensionData.Offset + 
                        Exp->ExtensionData.FieldOffset;
                }
            }
        }

        void OffsetCompiler::Do(const ExpT& Exp, const LabelledTS* TheLTS)
        {
            OffsetCompiler TheCompiler(TheLTS);
            Exp->Accept(&TheCompiler);
        }

        // Interpreters
        RValueInterpreter::RValueInterpreter(ExpPtrT Exp)
            : Exp(Exp)
        {
            // Nothing here
        }

        RValueInterpreter::~RValueInterpreter()
        {
            // Nothing here
        }

        ExpPtrT RValueInterpreter::GetExp() const
        {
            return Exp;
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

            if (Exp->GetType()->Is<ExprFieldAccessType>()) {
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
                auto const& RecType = Children[0]->GetType()->SAs<ExprRecordType>();
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
                                            const unordered_set<i64>& InterpretedOps) const
        {
            return;
        }

        LValueInterpreter::LValueInterpreter(ExpPtrT Exp)
            : RValueInterpreter(Exp)
        {
            auto const& Type  = Exp->GetType();
            Scalar = Type->Is<ExprScalarType>();
            if (!Scalar) {
                Size = 0;
                High = INT64_MAX;
                Low = INT64_MIN;
            } else {
                Size = Type->GetByteSize();
                if (Type->Is<ExprRangeType>()) {
                    auto TypeAsRange = Type->SAs<ExprRangeType>();
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

        bool LValueInterpreter::Update(const RValueInterpreter *RHS, 
                                       const StateVec *InStateVector, 
                                       StateVec *OutStateVector) const
        {
            return Write(RHS->Evaluate(InStateVector), InStateVector, OutStateVector);
        }
        
        CompiledConstInterpreter::CompiledConstInterpreter(i64 Value, ExpPtrT Exp)
            : RValueInterpreter(Exp), Value(Value)
        {
            // Nothing here
        }

        CompiledConstInterpreter::~CompiledConstInterpreter()
        {
            // Nothing here
        }

        i64 CompiledConstInterpreter::Evaluate(const StateVec *StateVector) const
        {
            return Value;
        }

        CompiledLValueInterpreter::CompiledLValueInterpreter(u32 Offset, ExpPtrT Exp)
            : LValueInterpreter(Exp), Offset(Offset)
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

        bool CompiledLValueInterpreter::Write(i64 Value, const StateVec* InStateVector,
                                              StateVec* StateVector) const 
        {
            i64 RawValue = 0;
            if (!Scalar) {
                throw InternalError((string)"Write() called on non-scalar type");
            }

            if (Value < Low || Value > High) {
                return false;
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

            return true;
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
            : RValueInterpreter(Exp), 
              ArgInterps(ArgInterps), SubEvals(ArgInterps.size()),
              NumArgInterps(ArgInterps.size()), Model(Z3Model::NullModel),
              Enabled(false), MyOpCode(Exp->As<OpExpression>()->GetOpCode())
        {
            // Nothing here
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
                auto ArgType = CurInterp->GetExp()->GetType()->As<Exprs::ExprScalarType>();
                auto AppArgExp = Mgr->MakeVal(ArgType->ValToConst(SubEvals[i]),
                                              CurInterp->GetExp()->GetType());
                AppArgExps.push_back(AppArgExp);
            }

            auto ConcAppExp = Mgr->MakeExpr(OpCode, AppArgExps);
            
            // Evaluate the expression in the model
            TPRef TP = Model.GetTPPtr();
            auto EvalExp = TP->Evaluate(ConcAppExp);
            auto EvalAsConst = EvalExp->As<Exprs::ConstExpression>();
            if (EvalAsConst == nullptr) {
                throw ESMCError((string)"Evaluating a term on the model did not " + 
                                "result in a constant valued interpretation.\nTerm:\n" + 
                                ConcAppExp->ToString() + "\nEvaluation:\n" + EvalExp->ToString());
            }
            auto RangeType = EvalExp->GetType()->As<Exprs::ExprScalarType>();
            auto Val = RangeType->ConstToVal(EvalAsConst->GetConstValue());

            // cache the result
            EvalMap[SubEvals] = Val;
            return Val;
        }

        i64 UFInterpreter::Evaluate(const StateVec* StateVector) const
        {
            if (!Enabled) {
                return 0;
            }

            // We are guaranteed that range and domain are scalars
            for (u32 i = 0; i < NumArgInterps; ++i) {
                SubEvals[i] = ArgInterps[i]->Evaluate(StateVector);
            }
            return DoEval();
        }

        void UFInterpreter::UpdateModel(const Z3Model& Model,
                                        const unordered_set<i64>& InterpretedOps) const
        {
            if (InterpretedOps.find(MyOpCode) == InterpretedOps.end()) {
                Enabled = false;
            } else {
                EvalMap.clear();
                this->Model = Model;
                Enabled = true;
            }
        }

        OpInterpreter::OpInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                     ExpPtrT Exp)
            : RValueInterpreter(Exp), SubInterps(SubInterps), 
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

        EQInterpreter::EQInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                     ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            // Nothing here
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
            // Nothing here
        }

        NOTInterpreter::~NOTInterpreter()
        {
            // Nothing here
        }

        i64 NOTInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            return (SubEvals[0] == 0);
        }

        ITEInterpreter::ITEInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                       ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            // Nothing here
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
            return (SubEvals[0] != 0 ? SubEvals[1] : SubEvals[2]);
        }

        ORInterpreter::ORInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                     ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            // Nothing here
        }

        ORInterpreter::~ORInterpreter()
        {
            // Nothing here
        }

        i64 ORInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            for (auto SubEval : SubEvals) {
                if (SubEval != 0) {
                    return 1;
                }
            }
            return 0;
        }

        ANDInterpreter::ANDInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                       ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            // Nothing here
        }

        ANDInterpreter::~ANDInterpreter()
        {
            // Nothing here
        }

        i64 ANDInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            for (auto SubEval : SubEvals) {
                if (SubEval == 0) {
                    return 0;
                }
            }
            return 1;
        }

        IMPLIESInterpreter::IMPLIESInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                               ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            // Nothing here
        }

        IMPLIESInterpreter::~IMPLIESInterpreter()
        {
            // Nothing here
        }

        i64 IMPLIESInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            return (SubEvals[0] == 0 || SubEvals[1] != 0);
        }

        IFFInterpreter::IFFInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                       ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            // Nothing here
        }

        IFFInterpreter::~IFFInterpreter()
        {
            // Nothing here
        }

        i64 IFFInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            return (SubEvals[0] == SubEvals[1]);
        }

        XORInterpreter::XORInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                       ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            // Nothing here
        }

        XORInterpreter::~XORInterpreter()
        {
            // Nothing here
        }

        i64 XORInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            return (SubEvals[0] != SubEvals[1]);
        }

        ADDInterpreter::ADDInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                       ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            // Nothing here
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
            }
            return Retval;
        }

        SUBInterpreter::SUBInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                       ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            // Nothing here
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
            }
            return Retval;
        }
        
        MINUSInterpreter::MINUSInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                           ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            // Nothing here
        }

        MINUSInterpreter::~MINUSInterpreter()
        {
            // Nothing here
        }

        i64 MINUSInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            i64 Retval = 0;
            Retval = Retval - SubEvals[0];
            return Retval;
        }

        MULInterpreter::MULInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                       ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            // Nothing here
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
            }
            return Retval;
        }
        
        DIVInterpreter::DIVInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                       ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            // Nothing here
        }

        DIVInterpreter::~DIVInterpreter()
        {
            // Nothing here
        }

        i64 DIVInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            return (SubEvals[0] / SubEvals[1]);
        }

        MODInterpreter::MODInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                       ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            // Nothing here
        }

        MODInterpreter::~MODInterpreter()
        {
            // Nothing here
        }

        i64 MODInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            return (SubEvals[0] % SubEvals[1]);
        }

        GTInterpreter::GTInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                     ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            // Nothing here
        }

        GTInterpreter::~GTInterpreter()
        {
            // Nothing here
        }

        i64 GTInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            return (SubEvals[0] > SubEvals[1]);
        }
        
        GEInterpreter::GEInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                     ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            // Nothing here
        }

        GEInterpreter::~GEInterpreter()
        {
            // Nothing here
        }

        i64 GEInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            return (SubEvals[0] >= SubEvals[1]);
        }

        LTInterpreter::LTInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                     ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            // Nothing here
        }

        LTInterpreter::~LTInterpreter()
        {
            // Nothing here
        }

        i64 LTInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            return (SubEvals[0] < SubEvals[1]);
        }

        LEInterpreter::LEInterpreter(const vector<RValueInterpreter*>& SubInterps,
                                     ExpPtrT Exp)
            : OpInterpreter(SubInterps, Exp)
        {
            // Nothing here
        }

        LEInterpreter::~LEInterpreter()
        {
            // Nothing here
        }

        i64 LEInterpreter::Evaluate(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            return (SubEvals[0] <= SubEvals[1]);
        }

        IndexInterpreter::IndexInterpreter(LValueInterpreter* ArrayInterp,
                                           RValueInterpreter* IndexInterp,
                                           ExpPtrT Exp)
            : LValueInterpreter(Exp),
              ArrayInterp(ArrayInterp), IndexInterp(IndexInterp)
        {
            auto const& ArrayType = ArrayInterp->GetExp()->GetType();
            
            ElemSize = ArrayType->SAs<ExprArrayType>()->GetValueType()->GetByteSize();
            ElemSize = Align(ElemSize, ElemSize);
            IndexSymmetric = IndexInterp->GetExp()->GetType()->Is<ExprSymmetricType>();
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
            if (Offset == UndefValue) {
                return UndefValue;
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

        bool IndexInterpreter::Write(i64 Value, const StateVec* InStateVector,
                                     StateVec* StateVector) const
        {
            if (!Scalar) {
                throw ESMCError((string)"WriteScalar() called on non-scalar type");
            }

            auto Offset = GetOffset(InStateVector);
            if (Offset == UndefValue) {
                return false;
            }
            auto BasePtr = StateVector->GetStateBuffer();
            auto DstPtr = BasePtr + Offset;

            i64 RawVal = Value - Low;

            if (Size == 1) {
                *((u08*)(DstPtr)) = (u08)RawVal;
            } else if (Size == 2) {
                *((u16*)(DstPtr)) = (u16)RawVal;
            } else {
                *((u32*)(DstPtr)) = (u32)RawVal;
            }
            
            return true;
        }

        i64 IndexInterpreter::GetOffset(const StateVec* StateVector) const
        {
            auto BaseOffset = ArrayInterp->GetOffset(StateVector);
            if (BaseOffset == UndefValue) {
                return UndefValue;
            }
            auto IndexValue = IndexInterp->Evaluate(StateVector);
            if (IndexSymmetric) {
                if (IndexValue == 0) {
                    return UndefValue;
                } else {
                    IndexValue = IndexValue - 1;
                }
            }
            
            return BaseOffset + (IndexValue * ElemSize);
        }

        FieldInterpreter::FieldInterpreter(LValueInterpreter* RecInterp,
                                           u32 FieldOffset, ExpPtrT Exp)
            : LValueInterpreter(Exp),
              RecInterp(RecInterp), FieldOffset(FieldOffset)
        {
            // Nothing here
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
            if (Offset == UndefValue) {
                return UndefValue;
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
            if (BaseOffset == UndefValue) {
                return UndefValue;
            }
            return BaseOffset + FieldOffset;
        }

        bool FieldInterpreter::Write(i64 Value, const StateVec* InStateVector,
                                     StateVec* StateVector) const
        {
            if (!Scalar) {
                throw ESMCError((string)"Write() called on non-scalar type");
            }

            i64 RawVal = Value - Low;

            auto BasePtr = StateVector->GetStateBuffer();
            auto Offset = GetOffset(InStateVector);

            if (Offset == UndefValue) {
                return false;
            }
            auto DstPtr = BasePtr + Offset;

            if (Size == 1) {
                *((u08*)(DstPtr)) = (u08)RawVal;
            } else if (Size == 2) {
                *((u16*)(DstPtr)) = (u16)RawVal;
            } else {
                *((u32*)(DstPtr)) = (u32)RawVal;
            }

            return true;
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
        }

        void LTSCompiler::CompileExp(const ExpT& Exp, LabelledTS* TheLTS)
        {
            OffsetCompiler::Do(Exp, TheLTS);
            RValueInterpreter::MakeInterpreter(Exp, this);
        }
        
        void LTSCompiler::CompileLTS(LabelledTS* TheLTS)
        {
            for (auto const& GCmd : TheLTS->GuardedCommands) {
                CompileExp(GCmd->GetGuard(), TheLTS);
                for (auto const& Update : GCmd->GetUpdates()) {
                    CompileExp(Update->GetLHS(), TheLTS);
                    CompileExp(Update->GetRHS(), TheLTS);
                }
            }
            CompileExp(TheLTS->InvariantExp, TheLTS);
            CompileExp(TheLTS->FinalCondExp, TheLTS);

            for (auto const& ChanExpPair : TheLTS->ChanBuffersToSort) {
                CompileExp(ChanExpPair.first, TheLTS);
                CompileExp(ChanExpPair.second, TheLTS);
            }

            for (auto const& InitStateGen : TheLTS->InitStateGenerators) {
                for (auto const& Update : InitStateGen) {
                    CompileExp(Update->GetLHS(), TheLTS);
                    CompileExp(Update->GetRHS(), TheLTS);
                }
            }
        }
        
        inline bool LTSCompiler::HasMsgLValue(const ExpT& Exp, ESMC::LTS::LabelledTS* TheLTS)
        {
            auto Mgr = TheLTS->GetMgr();
            auto&& Vars = 
                Mgr->Gather(Exp, 
                            [&] (const ExpBaseT* Exp) -> bool 
                            {
                                auto ExpAsVar = Exp->As<VarExpression>();
                                if (ExpAsVar != nullptr) {
                                    if (ExpAsVar->GetType() == TheLTS->GetUnifiedMType() &&
                                        ExpAsVar->GetVarName() == TheLTS->GetProductMsgName()) {
                                        return true;
                                    }
                                }
                                return false;
                            });
            return (Vars.size() > 0);
        }

        // Removes the dependence on the __trans_msg__ variable
        vector<GCmdRef> LTSCompiler::CompileCommands(const vector<GCmdRef>& Commands,
                                                     LabelledTS* TheLTS)
        {
            vector<GCmdRef> Retval;
            auto Mgr = TheLTS->GetMgr();
            for (auto const& Cmd : Commands) {
                MgrT::SubstMapT SubstMap;
                auto const& Updates = Cmd->GetUpdates();
                vector<LTSAssignRef> NewUpdates;
                for (auto const& Update : Updates) {
                    auto const& LHS = Update->GetLHS();
                    auto const& RHS = Update->GetRHS();

                    if (HasMsgLValue(LHS, TheLTS)) {
                        auto NewSubstitution = Mgr->TermSubstitute(SubstMap, RHS);
                        SubstMap[LHS] = NewSubstitution;
                    } else {
                        auto NewRHS = Mgr->TermSubstitute(SubstMap, RHS);
                        auto SimpLHS = Mgr->Simplify(LHS);
                        auto SimpRHS = Mgr->Simplify(NewRHS);
                        NewUpdates.push_back(new LTSAssignSimple(SimpLHS, SimpRHS));
                    }
                }

                set<LTSFairObjRef> FairObjSet(Cmd->GetFairnessObjs().begin(),
                                              Cmd->GetFairnessObjs().end());

                Retval.push_back(new LTSGuardedCommand(Cmd->GetMgr(),
                                                       Cmd->GetGuardComps(),
                                                       NewUpdates, 
                                                       Cmd->GetMsgType(),
                                                       Cmd->GetMsgTypeID(),
                                                       FairObjSet,
                                                       Cmd->GetProductTransition()));
                Retval.back()->SetCmdID(Cmd->GetCmdID());
            }
            return Retval;
        }

        void LTSCompiler::UpdateModel(const Z3Model& Model,
                                      const unordered_set<i64>& InterpretedOps)
        {
            // Push the model through all the registered
            // interpreters
            for (auto const& Interp : RegisteredInterps) {
                Interp->UpdateModel(Model, InterpretedOps);
            }
        }

    } /* end namespace */
} /* end namespace */

// 
// Compiler.cpp ends here
