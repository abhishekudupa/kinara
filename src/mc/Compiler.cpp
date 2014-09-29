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

#include <boost/lexical_cast.hpp>

#include "../uflts/LabelledTS.hpp"
#include "../utils/SizeUtils.hpp"
#include "../uflts/LTSTransitions.hpp"
#include "../uflts/LTSAssign.hpp"

#include "Compiler.hpp"
#include "StateVec.hpp"
#include "ZeroPage.hpp"
#include "LTSChecker.hpp"

namespace ESMC {
    namespace MC {

        const i64 UndefValue = INT64_MAX;
        
        using namespace ESMC::LTS;

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
            if (Exp->GetVarName() == TheLTS->GetProductMsgName() &&
                Exp->GetVarType() == TheLTS->GetUnifiedMType()) {
                Exp->ExtensionData.Offset = 0;
                Exp->ExtensionData.IsMsg = true;
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

            if (ConstVal == "clear") {
                if (!Type->Is<ExprScalarType>()) {
                    Exp->ExtensionData.ClearConstant = true;
                    Exp->ExtensionData.ConstCompiled = true;
                    return;
                } else {
                    Exp->ExtensionData.ConstCompiled = true;
                    Exp->ExtensionData.ConstVal = UndefValue;
                    return;
                }
            }
            
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
                Exp->ExtensionData.IsMsg = Children[0]->ExtensionData.IsMsg;
                if (Children[0]->ExtensionData.Offset != -1 &&
                    Children[1]->Is<Exprs::ConstExpression>()) {
                    auto ArrayType = Children[0]->GetType()->As<ExprArrayType>();
                    auto ValueType = ArrayType->GetValueType();
                    auto ValueSize = ValueType->GetByteSize();
                    ValueSize = Align(ValueSize, ValueSize);
                    Exp->ExtensionData.Offset = 
                        Children[0]->ExtensionData.Offset + 
                        (ValueSize * (Children[1]->ExtensionData.ConstVal));
                    return;
                }
            }

            if (OpCode == LTSOps::OpField) {
                Exp->ExtensionData.IsMsg = Children[0]->ExtensionData.IsMsg;
                auto ChildAsVar = Children[1]->As<Exprs::VarExpression>();
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
        RValueInterpreter::RValueInterpreter(bool Scalar, u32 Size)
            : Scalar(Scalar), Size(Size)
        {
            // Nothing here
        }

        RValueInterpreter::~RValueInterpreter()
        {
            // Nothing here
        }

        bool RValueInterpreter::IsScalar() const
        {
            return Scalar;
        }

        u32 RValueInterpreter::GetSize() const
        {
            return Size;
        }

        void RValueInterpreter::MakeInterpreter(const ExpT& Exp, LTSCompiler* Compiler)
        {
            auto& Ext = Exp->ExtensionData;
            if (Ext.Interp != nullptr) {
                return;
            }
            auto Type = Exp->GetType();
            
            if (Ext.ConstCompiled) {
                if (Ext.ClearConstant) {
                    Ext.Interp = 
                        new CompiledConstInterpreter(Type->GetByteSize(), ZeroPage::Get());
                    Compiler->RegisterInterp(Ext.Interp);
                } else {
                    auto TypeAsScalar = Type->As<ExprScalarType>();
                    if (!TypeAsScalar->Is<ExprIntType>()) {
                        Ext.Interp = 
                            new CompiledConstInterpreter(Type->GetByteSize(), Ext.ConstVal);
                    } else {
                        Ext.Interp = new CompiledConstInterpreter(4, Ext.ConstVal);
                    }
                    Compiler->RegisterInterp(Ext.Interp);
                }
                return;
            }

            if (Ext.Offset != -1) {
                i64 Low = 0;
                i64 High = INT64_MAX;
                
                auto TypeAsRange = Type->As<Exprs::ExprRangeType>();
                auto TypeAsScalar = Type->As<Exprs::ExprScalarType>();
                if (TypeAsRange != nullptr) {
                    Low = TypeAsRange->GetLow();
                    High = TypeAsRange->GetHigh();
                } else if (TypeAsScalar != nullptr) {
                    Low = 0;
                    High = TypeAsScalar->GetCardinality() - 1;
                }

                Ext.Interp = new CompiledLValueInterpreter(Type->GetByteSize(),
                                                           Ext.IsMsg, 
                                                           Type->Is<Exprs::ExprScalarType>(),
                                                           Ext.Offset,
                                                           Low, High);
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
                Ext.Interp = new EQInterpreter(SubInterps);
                break;
            case LTSOps::OpNOT:
                Ext.Interp = new NOTInterpreter(SubInterps);
                break;
            case LTSOps::OpITE:
                Ext.Interp = new ITEInterpreter(SubInterps);
                break;
            case LTSOps::OpOR:
                Ext.Interp = new ORInterpreter(SubInterps);
                break;
            case LTSOps::OpAND:
                Ext.Interp = new ANDInterpreter(SubInterps);
                break;
            case LTSOps::OpIMPLIES:
                Ext.Interp = new IMPLIESInterpreter(SubInterps);
                break;
            case LTSOps::OpIFF:
                Ext.Interp = new IFFInterpreter(SubInterps);
                break;
            case LTSOps::OpXOR:
                Ext.Interp = new XORInterpreter(SubInterps);
                break;
            case LTSOps::OpADD:
                Ext.Interp = new ADDInterpreter(SubInterps);
                break;
            case LTSOps::OpSUB:
                Ext.Interp = new SUBInterpreter(SubInterps);
                break;
            case LTSOps::OpMINUS:
                Ext.Interp = new MINUSInterpreter(SubInterps);
                break;
            case LTSOps::OpMUL:
                Ext.Interp = new MULInterpreter(SubInterps);
                break;
            case LTSOps::OpDIV:
                Ext.Interp = new DIVInterpreter(SubInterps);
                break;
            case LTSOps::OpMOD:
                Ext.Interp = new MODInterpreter(SubInterps);
                break;
            case LTSOps::OpGT:
                Ext.Interp = new GTInterpreter(SubInterps);
                break;
            case LTSOps::OpGE:
                Ext.Interp = new GEInterpreter(SubInterps);
                break;
            case LTSOps::OpLT:
                Ext.Interp = new LTInterpreter(SubInterps);
                break;
            case LTSOps::OpLE:
                Ext.Interp = new LEInterpreter(SubInterps);
                break;
            case LTSOps::OpIndex: {
                i64 Low = 0;
                i64 High = INT64_MAX;
                
                auto TypeAsRange = Type->As<Exprs::ExprRangeType>();
                if (TypeAsRange != nullptr) {
                    Low = TypeAsRange->GetLow();
                    High = TypeAsRange->GetHigh();
                } else if (Type->Is<Exprs::ExprScalarType>()) {
                    Low = 0;
                    High = Type->GetCardinality() - 1;
                }

                if (!SubInterps[0]->Is<LValueInterpreter>()) {
                    throw InternalError((string)"Expected an LValueInterpreter.\nAt: " +
                                        __FILE__ + ":" + to_string(__LINE__));
                }

                auto ArrayType = Children[0]->GetType()->SAs<Exprs::ExprArrayType>();
                u32 TotalSize = ArrayType->GetByteSize();
                u32 ElemSize = TotalSize / ArrayType->GetIndexType()->GetCardinality();
                Ext.Interp = new IndexInterpreter(Type->GetByteSize(),
                                                  Ext.IsMsg, Type->Is<Exprs::ExprScalarType>(),
                                                  SubInterps[0]->As<LValueInterpreter>(), 
                                                  SubInterps[1],
                                                  ElemSize,
                                                  Low, High);
            }
                break;

            case LTSOps::OpField: {
                i64 Low = 0;
                i64 High = INT64_MAX;
                
                auto TypeAsRange = Type->As<Exprs::ExprRangeType>();
                if (TypeAsRange != nullptr) {
                    Low = TypeAsRange->GetLow();
                    High = TypeAsRange->GetHigh();
                } else if (Type->Is<Exprs::ExprScalarType>()) {
                    Low = 0;
                    High = Type->GetCardinality() - 1;
                }
                if (!SubInterps[0]->Is<LValueInterpreter>()) {
                    throw InternalError((string)"Expected an LValueInterpreter.\nAt: " +
                                        __FILE__ + ":" + to_string(__LINE__));
                } 
                Ext.Interp = new FieldInterpreter(Type->GetByteSize(),
                                                  Ext.IsMsg, Type->Is<Exprs::ExprScalarType>(),
                                                  SubInterps[0]->As<LValueInterpreter>(), 
                                                  Ext.FieldOffset,
                                                  Low, High);
            }
                break;

            default:
                throw UnimplementedException("Uninterpreted Function Interpretations",
                                             __FILE__, __LINE__);
            }

            Compiler->RegisterInterp(Ext.Interp);
        }

        LValueInterpreter::LValueInterpreter(u32 Size, bool Msg, 
                                             bool Scalar, i64 Low, i64 High)
            : RValueInterpreter(Scalar, Size), Msg(Msg), Low(Low), High(High)
        {
            // Nothing here
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

        bool LValueInterpreter::IsMsg() const
        {
            return Msg;
        }

        void LValueInterpreter::Update(const RValueInterpreter *RHS, 
                                       const StateVec *InStateVector, 
                                       StateVec *OutStateVector) const
        {
            if (Scalar) {
                WriteScalar(RHS->EvaluateScalar(InStateVector),
                            OutStateVector);
            } else {
                Write(RHS->Evaluate(InStateVector), OutStateVector);
            }
        }
        
        CompiledConstInterpreter::CompiledConstInterpreter(u32 Size, i64 Value)
            : RValueInterpreter(true, Size), Value(Value), Ptr(nullptr)
        {
            // Nothing here
        }

        CompiledConstInterpreter::CompiledConstInterpreter(u32 Size, u08* Ptr)
            : RValueInterpreter(false, Size), Value(INT64_MAX), Ptr(Ptr)
        {
            // Nothing here
        }

        CompiledConstInterpreter::~CompiledConstInterpreter()
        {
            // Nothing here
        }

        i64 CompiledConstInterpreter::EvaluateScalar(const StateVec *StateVector) const
        {
            if (Scalar) {
                return Value;
            }
            throw InternalError((string)"EvaluateScalar() called on non-scalar type");
        }

        i64 CompiledConstInterpreter::EvaluateScalarNE(const StateVec* StateVector) const
        {
            return EvaluateScalar(StateVector);
        }

        const u08* CompiledConstInterpreter::Evaluate(const StateVec *StateVector) const
        {
            if (Scalar) {
                throw InternalError((string)"Evaluate() called on scalar type");
            } else {
                return Ptr;
            }
        }

        const u08* CompiledConstInterpreter::EvaluateNE(const StateVec* StateVector) const
        {
            return Evaluate(StateVector);
        }

        CompiledLValueInterpreter::CompiledLValueInterpreter(u32 Size, bool Msg, 
                                                             bool Scalar, u32 Offset, 
                                                             i64 Low, i64 High)
            : LValueInterpreter(Size, Msg, Scalar, Low, High), Offset(Offset)
        {
            // Nothing here
        }

        CompiledLValueInterpreter::~CompiledLValueInterpreter()
        {
            // Nothing here
        }

        i64 CompiledLValueInterpreter::EvaluateScalar(const StateVec* StateVector) const
        {
            if (!Scalar) {
                throw InternalError((string)"EvaluateScalar() called on non-scalar type");
            }
            i64 RawVal = 0;
            if (Msg) {
                if (Size == 1) {
                    RawVal = StateVector->ReadByteMsg(Offset);
                } else if (Size == 2) {
                    RawVal = StateVector->ReadShortMsg(Offset);
                } else if (Size == 4) {
                    RawVal = StateVector->ReadWordMsg(Offset);
                }
            } else {
                if (Size == 1) {
                    RawVal = StateVector->ReadByte(Offset);
                } else if (Size == 2) {
                    RawVal = StateVector->ReadShort(Offset);
                } else if (Size == 4) {
                    RawVal = StateVector->ReadWord(Offset);
                }
            }
            if (RawVal == 0) {
                return UndefValue;
            } else {
                return RawVal + Low - 1;
            }
        }

        i64 CompiledLValueInterpreter::EvaluateScalarNE(const StateVec* StateVector) const
        {
            return EvaluateScalar(StateVector);
        }

        const u08* CompiledLValueInterpreter::Evaluate(const StateVec* StateVector) const
        {
            if (Msg) {
                return (StateVector->GetMsgBuffer() + Offset);
            } else {
                return (StateVector->GetStateBuffer() + Offset);
            }
        }

        const u08* CompiledLValueInterpreter::EvaluateNE(const StateVec* StateVector) const
        {
            return Evaluate(StateVector);
        }

        void CompiledLValueInterpreter::WriteScalar(i64 Value, StateVec* StateVector) const 
        {
            i64 RawValue = 0;
            if (!Scalar) {
                throw InternalError((string)"WriteScalar() called on non-scalar type");
            }
            if (Value == UndefValue) {
                RawValue = 0;
            }
            else if (Value < Low || Value > High) {
                throw MCException(MCExceptionType::MCOOBWRITE, 0);
            } else {
                RawValue = Value - Low + 1;
            }
            if (Msg) {
                if (Size == 1) {
                    StateVector->WriteByteMsg(Offset, (u08)RawValue);
                } else if (Size == 2) {
                    StateVector->WriteShortMsg(Offset, (u16)RawValue);
                } else if (Size == 4) {
                    StateVector->WriteWordMsg(Offset, (u32)RawValue);
                }
            } else {
                if (Size == 1) {
                    StateVector->WriteByte(Offset, (u08)RawValue);
                } else if (Size == 2) {
                    StateVector->WriteShort(Offset, (u16)RawValue);
                } else if (Size == 4) {
                    StateVector->WriteWord(Offset, (u32)RawValue);
                }
            }
        }

        void CompiledLValueInterpreter::Write(const u08* Ptr, StateVec* StateVector) const
        {
            u08* DstPtr;
            if (Msg) {
                DstPtr = StateVector->GetMsgBuffer() + Offset;
            } else {
                DstPtr = StateVector->GetStateBuffer() + Offset;
            }
            memcpy(DstPtr, Ptr, Size);
        }


        u32 CompiledLValueInterpreter::GetOffset() const
        {
            return Offset;
        }

        OpInterpreter::OpInterpreter(bool Scalar, u32 Size,
                                     const vector<RValueInterpreter*>& SubInterps)
            : RValueInterpreter(Scalar, Size), SubInterps(SubInterps), 
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
                auto EvalValue = SubInterps[i]->EvaluateScalar(StateVector);
                if (EvalValue == UndefValue) {
                    throw MCException(MCExceptionType::MCUNDEFVALUE, 0);
                }
                SubEvals[i] = EvalValue;
            }
        }

        EQInterpreter::EQInterpreter(const vector<RValueInterpreter*>& SubInterps)
            : OpInterpreter(true, 1, SubInterps)
        {
            // Nothing here
        }

        EQInterpreter::~EQInterpreter()
        {
            // Nothing here
        }

        i64 EQInterpreter::EvaluateScalar(const StateVec* StateVector) const
        {
            if (SubInterps[0]->IsScalar()) {
                auto SubEval1 = SubInterps[0]->EvaluateScalar(StateVector);
                auto SubEval2 = SubInterps[1]->EvaluateScalar(StateVector);
                return (SubEval1 == SubEval2);
            } else {
                return (memcmp(SubInterps[0]->Evaluate(StateVector),
                               SubInterps[1]->Evaluate(StateVector),
                               SubInterps[0]->GetSize()) == 0);
            }
        }

        i64 EQInterpreter::EvaluateScalarNE(const StateVec* StateVector) const
        {
            if (SubInterps[0]->IsScalar()) {
                auto SubEval1 = SubInterps[0]->EvaluateScalarNE(StateVector);
                auto SubEval2 = SubInterps[1]->EvaluateScalarNE(StateVector);
                return (SubEval1 == SubEval2);
            } else {
                auto SubPtr1 = SubInterps[0]->EvaluateNE(StateVector);
                auto SubPtr2 = SubInterps[0]->EvaluateNE(StateVector);
                if (SubPtr1 == nullptr && SubPtr2 == nullptr) {
                    return 1;
                } else if (SubPtr1 == nullptr || SubPtr2 == nullptr) {
                    return 0;
                } else {
                    return (memcmp(SubPtr1, SubPtr2, SubInterps[0]->GetSize()) == 0);
                }
            }
        }

        const u08* EQInterpreter::Evaluate(const StateVec* StateVector) const
        {
            throw InternalError((string)"Evaluate() called on scalar type");
        }

        const u08* EQInterpreter::EvaluateNE(const StateVec* StateVector) const
        {
            throw InternalError((string)"Evaluate() called on scalar type");
        }


        NOTInterpreter::NOTInterpreter(const vector<RValueInterpreter*>& SubInterps)
            : OpInterpreter(true, 1, SubInterps)
        {
            // Nothing here
        }

        NOTInterpreter::~NOTInterpreter()
        {
            // Nothing here
        }

        i64 NOTInterpreter::EvaluateScalar(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            return (SubEvals[0] == 0);
        }

        i64 NOTInterpreter::EvaluateScalarNE(const StateVec* StateVector) const
        {
            auto SubEval = SubInterps[0]->EvaluateScalarNE(StateVector);
            if (SubEval == UndefValue) {
                return UndefValue;
            } else {
                return (SubEval == 0 ? 1 : 0);
            }
        }

        const u08* NOTInterpreter::Evaluate(const StateVec* StateVector) const
        {
            throw InternalError((string)"Evaluate() called on scalar type");
        }

        const u08* NOTInterpreter::EvaluateNE(const StateVec* StateVector) const
        {
            throw InternalError((string)"Evaluate() called on scalar type");
        }

        ITEInterpreter::ITEInterpreter(const vector<RValueInterpreter*>& SubInterps)
            : OpInterpreter(SubInterps[1]->IsScalar(), SubInterps[1]->GetSize(),
                            SubInterps)
        {
            // Nothing here
        }

        ITEInterpreter::~ITEInterpreter()
        {
            // Nothing here
        }

        i64 ITEInterpreter::EvaluateScalar(const StateVec* StateVector) const
        {
            if (!Scalar) {
                throw InternalError((string)"EvaluateScalar() called on non-scalar type");
            }
            EvaluateSubInterps(StateVector);
            return (SubEvals[0] != 0 ? SubEvals[1] : SubEvals[2]);
        }

        i64 ITEInterpreter::EvaluateScalarNE(const StateVec* StateVector) const
        {
            if (!Scalar) {
                throw InternalError((string)"EvaluateScalar() called on non-scalar type");
            }
            auto PredEval = SubInterps[0]->EvaluateScalarNE(StateVector);
            if (PredEval == UndefValue) {
                return UndefValue;
            }
            else if (PredEval != 0) {
                return SubInterps[1]->EvaluateScalarNE(StateVector);
            } else {
                return SubInterps[2]->EvaluateScalarNE(StateVector);
            }
        }

        const u08* ITEInterpreter::Evaluate(const StateVec *StateVector) const
        {
            if (Scalar) {
                throw InternalError((string)"Evaluate() called on scalar type");
            }

            return (SubInterps[0]->EvaluateScalar(StateVector) != 0 ? 
                    SubInterps[1]->Evaluate(StateVector) : 
                    SubInterps[2]->Evaluate(StateVector));
        }

        const u08* ITEInterpreter::EvaluateNE(const StateVec *StateVector) const
        {
            if (Scalar) {
                throw InternalError((string)"Evaluate() called on scalar type");
            }
            
            auto PredEval = SubInterps[0]->EvaluateScalarNE(StateVector);
            if (PredEval == UndefValue) {
                return nullptr;
            } else if (PredEval != 0) {
                return SubInterps[1]->EvaluateNE(StateVector);
            } else {
                return SubInterps[2]->EvaluateNE(StateVector);
            }
        }

        ORInterpreter::ORInterpreter(const vector<RValueInterpreter*>& SubInterps)
            : OpInterpreter(true, 1, SubInterps)
        {
            // Nothing here
        }

        ORInterpreter::~ORInterpreter()
        {
            // Nothing here
        }

        i64 ORInterpreter::EvaluateScalar(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            for (auto SubEval : SubEvals) {
                if (SubEval != 0) {
                    return 1;
                }
            }
            return 0;
        }

        i64 ORInterpreter::EvaluateScalarNE(const StateVec* StateVector) const
        {
            bool Exception = false;
            for (u32 i = 0; i < NumSubInterps; ++i) {
                auto CurValue = SubInterps[i]->EvaluateScalarNE(StateVector);
                if (CurValue != UndefValue && CurValue != 0) {
                    return 1;
                } else if (CurValue == UndefValue) {
                    Exception = true;
                }
            }

            if (Exception) {
                return UndefValue;
            } else {
                return 0;
            }
        }

        const u08* ORInterpreter::Evaluate(const StateVec* StateVector) const
        {
            throw InternalError((string)"Evaluate() called on scalar type");
        }

        const u08* ORInterpreter::EvaluateNE(const StateVec* StateVector) const
        {
            throw InternalError((string)"EvaluateNE() called on scalar type");
        }

        ANDInterpreter::ANDInterpreter(const vector<RValueInterpreter*>& SubInterps)
            : OpInterpreter(true, 1, SubInterps)
        {
            // Nothing here
        }

        ANDInterpreter::~ANDInterpreter()
        {
            // Nothing here
        }

        i64 ANDInterpreter::EvaluateScalar(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            for (auto SubEval : SubEvals) {
                if (SubEval == 0) {
                    return 0;
                }
            }
            return 1;
        }

        i64 ANDInterpreter::EvaluateScalarNE(const StateVec* StateVector) const
        {
            bool Exception = false;
            for (u32 i = 0; i < NumSubInterps; ++i) {
                auto CurEval = SubInterps[i]->EvaluateScalarNE(StateVector);
                if (CurEval == 0) {
                    return 0;
                } else if (CurEval == UndefValue) {
                    Exception = true;
                }
            }

            if (Exception) {
                return UndefValue;
            } else {
                return 1;
            }
        }

        const u08* ANDInterpreter::Evaluate(const StateVec* StateVector) const
        {
            throw InternalError((string)"Evaluate() called on scalar type");
        }

        const u08* ANDInterpreter::EvaluateNE(const StateVec* StateVector) const
        {
            throw InternalError((string)"EvaluateNE() called on scalar type");
        }

        IMPLIESInterpreter::IMPLIESInterpreter(const vector<RValueInterpreter*>& SubInterps)
            : OpInterpreter(true, 1, SubInterps)
        {
            // Nothing here
        }

        IMPLIESInterpreter::~IMPLIESInterpreter()
        {
            // Nothing here
        }

        i64 IMPLIESInterpreter::EvaluateScalar(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            return (SubEvals[0] == 0 || SubEvals[1] != 0);
        }

        i64 IMPLIESInterpreter::EvaluateScalarNE(const StateVec* StateVector) const
        {
            auto SubEval0 = SubInterps[0]->EvaluateScalarNE(StateVector);
            auto SubEval1 = SubInterps[1]->EvaluateScalarNE(StateVector);

            bool Exception = false;
            if (SubEval0 == UndefValue || SubEval1 == UndefValue) {
                Exception = true;
            }
            if (SubEval0 != UndefValue && SubEval0 == 0) {
                return 1;
            } else if (SubEval1 != UndefValue && SubEval1 != 0) {
                return 1;
            } else if (Exception) {
                return UndefValue;
            } else {
                return 0;
            }
        }
        

        const u08* IMPLIESInterpreter::Evaluate(const StateVec* StateVector) const
        {
            throw InternalError((string)"Evaluate() called on scalar type");
        }

        const u08* IMPLIESInterpreter::EvaluateNE(const StateVec* StateVector) const
        {
            throw InternalError((string)"EvaluateNE() called on scalar type");
        }

        IFFInterpreter::IFFInterpreter(const vector<RValueInterpreter*>& SubInterps)
            : OpInterpreter(true, 1, SubInterps)
        {
            // Nothing here
        }

        IFFInterpreter::~IFFInterpreter()
        {
            // Nothing here
        }

        i64 IFFInterpreter::EvaluateScalar(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            return (SubEvals[0] == SubEvals[1]);
        }

        i64 IFFInterpreter::EvaluateScalarNE(const StateVec* StateVector) const
        {
            auto SubEval0 = SubInterps[0]->EvaluateScalarNE(StateVector);
            auto SubEval1 = SubInterps[1]->EvaluateScalarNE(StateVector);

            return (SubEval0 == SubEval1);
        }


        const u08* IFFInterpreter::Evaluate(const StateVec* StateVector) const
        {
            throw InternalError((string)"Evaluate() called on scalar type");
        }

        const u08* IFFInterpreter::EvaluateNE(const StateVec* StateVector) const
        {
            throw InternalError((string)"EvaluateNE() called on scalar type");
        }

        XORInterpreter::XORInterpreter(const vector<RValueInterpreter*>& SubInterps)
            : OpInterpreter(true, 1, SubInterps)
        {
            // Nothing here
        }

        XORInterpreter::~XORInterpreter()
        {
            // Nothing here
        }

        i64 XORInterpreter::EvaluateScalar(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            return (SubEvals[0] != SubEvals[1]);
        }

        i64 XORInterpreter::EvaluateScalarNE(const StateVec* StateVector) const
        {
            auto SubEval0 = SubInterps[0]->EvaluateScalarNE(StateVector);
            auto SubEval1 = SubInterps[1]->EvaluateScalarNE(StateVector);
            
            return (SubEval0 != SubEval1);
        }

        const u08* XORInterpreter::Evaluate(const StateVec* StateVector) const
        {
            throw InternalError((string)"Evaluate() called on scalar type");
        }

        const u08* XORInterpreter::EvaluateNE(const StateVec* StateVector) const
        {
            throw InternalError((string)"EvaluateNE() called on scalar type");
        }

        ADDInterpreter::ADDInterpreter(const vector<RValueInterpreter*>& SubInterps)
            : OpInterpreter(true, 4, SubInterps)
        {
            // Nothing here
        }

        ADDInterpreter::~ADDInterpreter()
        {
            // Nothing here
        }

        i64 ADDInterpreter::EvaluateScalar(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            i64 Retval = 0;
            for (auto SubEval : SubEvals) {
                Retval += SubEval;
            }
            return Retval;
        }

        i64 ADDInterpreter::EvaluateScalarNE(const StateVec* StateVector) const
        {
            i64 Retval = 0;
            for (u32 i = 0; i < NumSubInterps; ++i) {
                auto CurSubEval = SubInterps[i]->EvaluateScalarNE(StateVector);
                if (CurSubEval == UndefValue) {
                    return UndefValue;
                }
                Retval += CurSubEval;
            }
            return Retval;
        }


        const u08* ADDInterpreter::Evaluate(const StateVec* StateVector) const
        {
            throw InternalError((string)"Evaluate() called on scalar type");
        }

        const u08* ADDInterpreter::EvaluateNE(const StateVec* StateVector) const
        {
            throw InternalError((string)"EvaluateNE() called on scalar type");
        }

        SUBInterpreter::SUBInterpreter(const vector<RValueInterpreter*>& SubInterps)
            : OpInterpreter(true, 4, SubInterps)
        {
            // Nothing here
        }

        SUBInterpreter::~SUBInterpreter()
        {
            // Nothing here
        }

        i64 SUBInterpreter::EvaluateScalar(const StateVec* StateVector) const
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

        i64 SUBInterpreter::EvaluateScalarNE(const StateVec* StateVector) const
        {
            i64 Retval = 0;
            for (u32 i = 0; i < NumSubInterps; ++i) {
                auto CurSubEval = SubInterps[i]->EvaluateScalarNE(StateVector);
                if (CurSubEval == UndefValue) {
                    return UndefValue;
                }
                if (i == 0) {
                    Retval = CurSubEval;
                } else {
                    Retval -= CurSubEval;
                }
            }
            return Retval;
        }

        const u08* SUBInterpreter::Evaluate(const StateVec* StateVector) const
        {
            throw InternalError((string)"Evaluate() called on scalar type");
        }

        const u08* SUBInterpreter::EvaluateNE(const StateVec* StateVector) const
        {
            throw InternalError((string)"EvaluateNE() called on scalar type");
        }
        
        MINUSInterpreter::MINUSInterpreter(const vector<RValueInterpreter*>& SubInterps)
            : OpInterpreter(true, 4, SubInterps)
        {
            // Nothing here
        }

        MINUSInterpreter::~MINUSInterpreter()
        {
            // Nothing here
        }

        i64 MINUSInterpreter::EvaluateScalar(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            i64 Retval = 0;
            Retval = Retval - SubEvals[0];
            return Retval;
        }

        i64 MINUSInterpreter::EvaluateScalarNE(const StateVec* StateVector) const
        {
            auto SubEval = SubInterps[0]->EvaluateScalarNE(StateVector);
            if (SubEval == UndefValue) {
                return UndefValue;
            } else {
                return (-SubEval);
            }
        }

        const u08* MINUSInterpreter::Evaluate(const StateVec* StateVector) const
        {
            throw InternalError((string)"Evaluate() called on scalar type");
        }

        const u08* MINUSInterpreter::EvaluateNE(const StateVec* StateVector) const
        {
            throw InternalError((string)"EvaluateNE() called on scalar type");
        }

        MULInterpreter::MULInterpreter(const vector<RValueInterpreter*>& SubInterps)
            : OpInterpreter(true, 4, SubInterps)
        {
            // Nothing here
        }

        MULInterpreter::~MULInterpreter()
        {
            // Nothing here
        }

        i64 MULInterpreter::EvaluateScalar(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            i64 Retval = 1;
            for (auto SubEval : SubEvals) {
                Retval *= SubEval;
            }
            return Retval;
        }

        i64 MULInterpreter::EvaluateScalarNE(const StateVec* StateVector) const
        {
            i64 Retval = 1;
            for (u32 i = 0; i < NumSubInterps; ++i) {
                auto CurSubEval = SubInterps[i]->EvaluateScalarNE(StateVector);
                if (CurSubEval == UndefValue) {
                    return UndefValue;
                }
                Retval *= CurSubEval;
            }

            return Retval;
        }

        const u08* MULInterpreter::Evaluate(const StateVec* StateVector) const
        {
            throw InternalError((string)"Evaluate() called on scalar type");
        }

        const u08* MULInterpreter::EvaluateNE(const StateVec* StateVector) const
        {
            throw InternalError((string)"EvaluateNE() called on scalar type");
        }
        
        DIVInterpreter::DIVInterpreter(const vector<RValueInterpreter*>& SubInterps)
            : OpInterpreter(true, 4, SubInterps)
        {
            // Nothing here
        }

        DIVInterpreter::~DIVInterpreter()
        {
            // Nothing here
        }

        i64 DIVInterpreter::EvaluateScalar(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            return (SubEvals[0] / SubEvals[1]);
        }

        i64 DIVInterpreter::EvaluateScalarNE(const StateVec* StateVector) const
        {
            auto SubEval0 = SubInterps[0]->EvaluateScalarNE(StateVector);
            auto SubEval1 = SubInterps[1]->EvaluateScalarNE(StateVector);
            
            if (SubEval0 == UndefValue || SubEval1 == UndefValue) {
                return UndefValue;
            } else {
                return SubEval0 / SubEval1;
            }
        }

        const u08* DIVInterpreter::Evaluate(const StateVec* StateVector) const
        {
            throw InternalError((string)"Evaluate() called on scalar type");
        }

        const u08* DIVInterpreter::EvaluateNE(const StateVec* StateVector) const
        {
            throw InternalError((string)"EvaluateNE() called on scalar type");
        }

        MODInterpreter::MODInterpreter(const vector<RValueInterpreter*>& SubInterps)
            : OpInterpreter(true, 4, SubInterps)
        {
            // Nothing here
        }

        MODInterpreter::~MODInterpreter()
        {
            // Nothing here
        }

        i64 MODInterpreter::EvaluateScalar(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            return (SubEvals[0] % SubEvals[1]);
        }

        i64 MODInterpreter::EvaluateScalarNE(const StateVec* StateVector) const
        {
            auto SubEval0 = SubInterps[0]->EvaluateScalarNE(StateVector);
            auto SubEval1 = SubInterps[1]->EvaluateScalarNE(StateVector);

            if (SubEval0 == UndefValue || SubEval1 == UndefValue) {
                return UndefValue;
            } else {
                return (SubEval0 % SubEval1);
            }
        }

        const u08* MODInterpreter::Evaluate(const StateVec* StateVector) const
        {
            throw InternalError((string)"Evaluate() called on scalar type");
        }

        const u08* MODInterpreter::EvaluateNE(const StateVec* StateVector) const
        {
            throw InternalError((string)"EvaluateNE() called on scalar type");
        }

        GTInterpreter::GTInterpreter(const vector<RValueInterpreter*>& SubInterps)
            : OpInterpreter(true, 1, SubInterps)
        {
            // Nothing here
        }

        GTInterpreter::~GTInterpreter()
        {
            // Nothing here
        }

        i64 GTInterpreter::EvaluateScalar(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            return (SubEvals[0] > SubEvals[1]);
        }

        i64 GTInterpreter::EvaluateScalarNE(const StateVec* StateVector) const
        {
            auto SubEval0 = SubInterps[0]->EvaluateScalarNE(StateVector);
            auto SubEval1 = SubInterps[1]->EvaluateScalarNE(StateVector);
            
            if (SubEval0 == UndefValue || SubEval1 == UndefValue) {
                return UndefValue;
            } else {
                return (SubEval0 > SubEval1);
            }
        }

        const u08* GTInterpreter::Evaluate(const StateVec* StateVector) const
        {
            throw InternalError((string)"Evaluate() called on scalar type");
        }

        const u08* GTInterpreter::EvaluateNE(const StateVec* StateVector) const
        {
            throw InternalError((string)"EvaluateNE() called on scalar type");
        }
        
        GEInterpreter::GEInterpreter(const vector<RValueInterpreter*>& SubInterps)
            : OpInterpreter(true, 1, SubInterps)
        {
            // Nothing here
        }

        GEInterpreter::~GEInterpreter()
        {
            // Nothing here
        }

        i64 GEInterpreter::EvaluateScalar(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            return (SubEvals[0] >= SubEvals[1]);
        }

        i64 GEInterpreter::EvaluateScalarNE(const StateVec* StateVector) const
        {
            auto SubEval0 = SubInterps[0]->EvaluateScalarNE(StateVector);
            auto SubEval1 = SubInterps[1]->EvaluateScalarNE(StateVector);
            
            if (SubEval0 == UndefValue || SubEval1 == UndefValue) {
                return UndefValue;
            } else {
                return (SubEval0 >= SubEval1);
            }
        }


        const u08* GEInterpreter::Evaluate(const StateVec* StateVector) const
        {
            throw InternalError((string)"Evaluate() called on scalar type");
        }

        const u08* GEInterpreter::EvaluateNE(const StateVec* StateVector) const
        {
            throw InternalError((string)"EvaluateNE() called on scalar type");
        }

        LTInterpreter::LTInterpreter(const vector<RValueInterpreter*>& SubInterps)
            : OpInterpreter(true, 1, SubInterps)
        {
            // Nothing here
        }

        LTInterpreter::~LTInterpreter()
        {
            // Nothing here
        }

        i64 LTInterpreter::EvaluateScalar(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            return (SubEvals[0] < SubEvals[1]);
        }

        i64 LTInterpreter::EvaluateScalarNE(const StateVec* StateVector) const
        {
            auto SubEval0 = SubInterps[0]->EvaluateScalarNE(StateVector);
            auto SubEval1 = SubInterps[1]->EvaluateScalarNE(StateVector);
            
            if (SubEval0 == UndefValue || SubEval1 == UndefValue) {
                return UndefValue;
            } else {
                return (SubEval0 < SubEval1);
            }
        }


        const u08* LTInterpreter::Evaluate(const StateVec* StateVector) const
        {
            throw InternalError((string)"Evaluate() called on scalar type");
        }

        const u08* LTInterpreter::EvaluateNE(const StateVec* StateVector) const
        {
            throw InternalError((string)"EvaluateNE() called on scalar type");
        }

        LEInterpreter::LEInterpreter(const vector<RValueInterpreter*>& SubInterps)
            : OpInterpreter(true, 1, SubInterps)
        {
            // Nothing here
        }

        LEInterpreter::~LEInterpreter()
        {
            // Nothing here
        }

        i64 LEInterpreter::EvaluateScalar(const StateVec* StateVector) const
        {
            EvaluateSubInterps(StateVector);
            return (SubEvals[0] <= SubEvals[1]);
        }

        i64 LEInterpreter::EvaluateScalarNE(const StateVec* StateVector) const
        {
            auto SubEval0 = SubInterps[0]->EvaluateScalarNE(StateVector);
            auto SubEval1 = SubInterps[1]->EvaluateScalarNE(StateVector);
            
            if (SubEval0 == UndefValue || SubEval1 == UndefValue) {
                return UndefValue;
            } else {
                return (SubEval0 <= SubEval1);
            }
        }

        const u08* LEInterpreter::Evaluate(const StateVec* StateVector) const
        {
            throw InternalError((string)"Evaluate() called on scalar type");
        }

        const u08* LEInterpreter::EvaluateNE(const StateVec* StateVector) const
        {
            throw InternalError((string)"EvaluateNE() called on scalar type");
        }

        IndexInterpreter::IndexInterpreter(u32 Size, bool Msg, bool IsScalar,
                                           LValueInterpreter* ArrayInterp,
                                           RValueInterpreter* IndexInterp,
                                           u32 ElemSize,
                                           i64 Low, i64 High)
            : LValueInterpreter(Size, Msg, IsScalar, Low, High),
              ArrayInterp(ArrayInterp), IndexInterp(IndexInterp),
              ElemSize(ElemSize)
        {
            // Nothing here
        }

        IndexInterpreter::~IndexInterpreter()
        {
            // Nothing here
        }
        
        i64 IndexInterpreter::EvaluateScalar(const StateVec* StateVector) const
        {
            if (!Scalar) {
                throw ESMCError((string)"EvaluateScalar() called on non-scalar type");
            }
            i64 Offset = IndexInterp->EvaluateScalar(StateVector);
            if (Offset == UndefValue) {
                throw MCException(MCExceptionType::MCUNDEFVALUE, 0);
            }
            Offset *= ElemSize;
            
            const u08* DstPtr = 
                ArrayInterp->SAs<LValueInterpreter>()->Evaluate(StateVector) + Offset;

            i64 RawVal = 0;
            if (Size == 1) {
                RawVal = *DstPtr;
            } else if (Size == 2) {
                RawVal = *((u16*)(DstPtr));
            } else {
                RawVal = *((u32*)(DstPtr));
            }

            if (RawVal == 0) {
                return UndefValue;
            } else {
                return RawVal - 1 + Low;
            }
        }

        i64 IndexInterpreter::EvaluateScalarNE(const StateVec* StateVector) const
        {
            if (!Scalar) {
                throw ESMCError((string)"EvaluateScalar() called on non-scalar type");
            }
            i64 Offset = IndexInterp->EvaluateScalarNE(StateVector);
            if (Offset == UndefValue) {
                return UndefValue;
            }
            Offset *= ElemSize;
            
            const u08* DstPtr = 
                ArrayInterp->SAs<LValueInterpreter>()->EvaluateNE(StateVector) + Offset;

            if (DstPtr == nullptr) {
                return UndefValue;
            }

            i64 RawVal = 0;
            if (Size == 1) {
                RawVal = *DstPtr;
            } else if (Size == 2) {
                RawVal = *((u16*)(DstPtr));
            } else {
                RawVal = *((u32*)(DstPtr));
            }

            if (RawVal == 0) {
                return UndefValue;
            } else {
                return RawVal - 1 + Low;
            }
        }

        const u08* IndexInterpreter::Evaluate(const StateVec* StateVector) const
        {
            if (Scalar) {
                throw ESMCError((string)"Evaluate() called on scalar type");
            }
            i64 Offset = IndexInterp->EvaluateScalar(StateVector);
            if (Offset == UndefValue) {
                throw MCException(MCExceptionType::MCUNDEFVALUE, 0);
            }
            Offset *= ElemSize;

            const u08* DstPtr = ArrayInterp->Evaluate(StateVector) + Offset;
            return DstPtr;
        }

        const u08* IndexInterpreter::EvaluateNE(const StateVec* StateVector) const
        {
            if (Scalar) {
                throw ESMCError((string)"Evaluate() called on scalar type");
            }
            i64 Offset = IndexInterp->EvaluateScalarNE(StateVector);
            if (Offset == UndefValue) {
                return nullptr;
            }
            Offset *= ElemSize;

            const u08* DstPtr = ArrayInterp->EvaluateNE(StateVector) + Offset;
            return DstPtr;
        }

        void IndexInterpreter::WriteScalar(i64 Value, StateVec* StateVector) const
        {
            if (!Scalar) {
                throw ESMCError((string)"WriteScalar() called on non-scalar type");
            }

            i64 Offset = IndexInterp->EvaluateScalar(StateVector);
            if (Offset == UndefValue) {
                throw MCException(MCExceptionType::MCUNDEFVALUE, 0);
            }
            Offset *= ElemSize;
            
            u08* DstPtr = 
                const_cast<u08*>(ArrayInterp->SAs<LValueInterpreter>()->Evaluate(StateVector) + 
                                 Offset);

            i64 RawVal = 0;
            if (Value == UndefValue) {
                RawVal = 0;
            } else if (Value < Low || Value > High) {
                throw MCException(MCExceptionType::MCOOBWRITE, 0);
            } else {
                RawVal = Value - Low + 1;
            }

            if (Size == 1) {
                *((u08*)(DstPtr)) = (u08)RawVal;
            } else if (Size == 2) {
                *((u16*)(DstPtr)) = (u16)RawVal;
            } else {
                *((u32*)(DstPtr)) = (u32)RawVal;
            }
        }

        void IndexInterpreter::Write(const u08* Ptr, StateVec* StateVector) const
        {
            i64 Offset = IndexInterp->EvaluateScalar(StateVector);
            if (Offset == UndefValue) {
                throw MCException(MCExceptionType::MCUNDEFVALUE, 0);
            }
            Offset *= ElemSize;

            u08* DstPtr = 
                const_cast<u08*>(ArrayInterp->SAs<LValueInterpreter>()->Evaluate(StateVector) + 
                                 Offset);

            memcpy(DstPtr, Ptr, Size);
        }

        FieldInterpreter::FieldInterpreter(u32 Size, bool Msg, bool IsScalar,
                                           LValueInterpreter* RecInterp,
                                           u32 FieldOffset,
                                           i64 Low, i64 High)
            : LValueInterpreter(Size, Msg, IsScalar, Low, High),
              RecInterp(RecInterp), FieldOffset(FieldOffset)
        {
            // Nothing here
        }

        FieldInterpreter::~FieldInterpreter()
        {
            // Nothing here
        }

        i64 FieldInterpreter::EvaluateScalar(const StateVec* StateVector) const
        {
            if (!Scalar) {
                throw ESMCError((string)"EvaluateScalar() called on non-scalar type");
            }

            const u08* DstPtr = 
                RecInterp->SAs<LValueInterpreter>()->Evaluate(StateVector) + FieldOffset;

            i64 RawVal = 0;
            if (Size == 1) {
                RawVal = (*DstPtr);
            } else if (Size == 2) {
                RawVal = *((u16*)(DstPtr));
            } else {
                RawVal = *((u32*)(DstPtr));
            }

            if (RawVal == 0) {
                return UndefValue;
            } else {
                return RawVal + Low - 1;
            }
        }

        i64 FieldInterpreter::EvaluateScalarNE(const StateVec* StateVector) const
        {
            if (!Scalar) {
                throw ESMCError((string)"EvaluateScalar() called on non-scalar type");
            }

            const u08* DstPtr = 
                RecInterp->SAs<LValueInterpreter>()->EvaluateNE(StateVector);

            if (DstPtr == nullptr) {
                return UndefValue;
            }

            DstPtr += FieldOffset;

            i64 RawVal = 0;
            if (Size == 1) {
                RawVal = (*DstPtr);
            } else if (Size == 2) {
                RawVal = *((u16*)(DstPtr));
            } else {
                RawVal = *((u32*)(DstPtr));
            }

            if (RawVal == 0) {
                return UndefValue;
            } else {
                return RawVal + Low - 1;
            }
        }

        const u08* FieldInterpreter::Evaluate(const StateVec* StateVector) const
        {
            if (Scalar) {
                throw ESMCError((string)"Evaluate() called on scalar type");
            }
            return (RecInterp->SAs<LValueInterpreter>()->Evaluate(StateVector) + FieldOffset);
        }

        const u08* FieldInterpreter::EvaluateNE(const StateVec* StateVector) const
        {
            if (Scalar) {
                throw ESMCError((string)"Evaluate() called on scalar type");
            }
            auto Retval = RecInterp->SAs<LValueInterpreter>()->EvaluateNE(StateVector);
            if (Retval == nullptr) {
                return nullptr;
            } else {
                return (Retval + FieldOffset);
            }
        }

        void FieldInterpreter::WriteScalar(i64 Value, StateVec* StateVector) const
        {
            if (!Scalar) {
                throw ESMCError((string)"WriteScalar() called on non-scalar type");
            }

            i64 RawVal = 0;
            if (Value == UndefValue) {
                RawVal = 0;
            } else if (Value < Low || Value > High) {
                throw MCException(MCExceptionType::MCOOBWRITE, 0);
            } else {
                RawVal = Value - Low + 1;
            }
            
            u08* DstPtr = 
                const_cast<u08*>(RecInterp->SAs<LValueInterpreter>()->Evaluate(StateVector) + 
                                 FieldOffset);

            if (Size == 1) {
                *((u08*)(DstPtr)) = (u08)RawVal;
            } else if (Size == 2) {
                *((u16*)(DstPtr)) = (u16)RawVal;
            } else {
                *((u32*)(DstPtr)) = (u32)RawVal;
            }
        }

        void FieldInterpreter::Write(const u08* Ptr, StateVec* StateVector) const
        {
            u08* DstPtr = 
                const_cast<u08*>(RecInterp->SAs<LValueInterpreter>()->Evaluate(StateVector) + 
                                 FieldOffset);
            memcpy(DstPtr, Ptr, Size);
        }

        LTSCompiler::LTSCompiler()
        {
            // Nothing here
        }

        LTSCompiler::~LTSCompiler()
        {
            for (auto Interp : InterpsToFree) {
                delete Interp;
            }
        }

        void LTSCompiler::RegisterInterp(RValueInterpreter *Interp)
        {
            InterpsToFree.push_back(Interp);
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

    } /* end namespace */
} /* end namespace */

// 
// Compiler.cpp ends here
