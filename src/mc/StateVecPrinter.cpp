// StateVecPrinter.cpp ---
//
// Filename: StateVecPrinter.cpp
// Author: Abhishek Udupa
// Created: Wed Aug 20 16:24:03 2014 (-0400)
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

#include "../uflts/LabelledTS.hpp"

#include "StateVecPrinter.hpp"
#include "StateVec.hpp"
#include "AQStructure.hpp"
#include "Compiler.hpp"
#include "OmegaAutomaton.hpp"

namespace ESMC {
namespace MC {

using namespace ESMC::LTS;

ScalarPrinter::ScalarPrinter()
    : Offset(0), Size(0), Type(TypeRef::NullPtr)
{
    // Nothing here
}

ScalarPrinter::ScalarPrinter(u32 Offset, const TypeRef& Type)
    : Offset(Offset), Size(Type->GetByteSize()), Type(Type), Low(0),
      High(INT64_MAX), IsMsgType(false)
{
    if (!Type->Is<ScalarType>()) {
        throw InternalError((string)"Scalar printer with non-scalar type:\n" +
                            Type->ToString() + "\nAt: " + __FILE__ + ":" +
                            to_string(__LINE__));
    }
    if (Type->Is<RangeType>()) {
        Low = Type->SAs<RangeType>()->GetLow();
        High = Type->SAs<RangeType>()->GetHigh();
    } else {
        High = Type->GetCardinality() - 1;
    }
}

ScalarPrinter::ScalarPrinter(const ScalarPrinter& Other)
    : Offset(Other.Offset), Size(Other.Size), Type(Other.Type),
      Low(Other.Low), High(Other.High), MsgNameMap(Other.MsgNameMap),
      IsMsgType(Other.IsMsgType)
{
    // Nothing here
}

ScalarPrinter::ScalarPrinter(u32 Offset, const vector<string> MsgNameMap)
    : Offset(Offset), Size(0), Type(TypeRef::NullPtr),
      Low(-1), High(-1), MsgNameMap(MsgNameMap), IsMsgType(true)
{
    // Nothing here
}

ScalarPrinter::~ScalarPrinter()
{
    // Nothing here
}

ScalarPrinter& ScalarPrinter::operator = (const ScalarPrinter& Other)
{
    if (&Other == this) {
        return *this;
    }
    Offset = Other.Offset;
    Size = Other.Size;
    Type = Other.Type;
    High = Other.High;
    Low = Other.Low;
    MsgNameMap = Other.MsgNameMap;
    IsMsgType = Other.IsMsgType;
    return *this;
}

bool ScalarPrinter::operator == (const ScalarPrinter& Other) const
{
    return (Offset == Other.Offset &&
            Size == Other.Size &&
            Type == Other.Type);
}

string ScalarPrinter::Print(const StateVec* StateVector) const
{
    if (IsMsgType) {
        auto ActVal = StateVector->ReadShort(Offset);
        // return (MsgNameMap[ActVal] + " { Offset : " + to_string(Offset) +
        //         ", Raw Value : " + to_string(ActVal) + " }");
        return MsgNameMap[ActVal];
    }

    if (Type == TypeRef::NullPtr) {
        return (string)"printer error at: " + __FILE__ + ":" + to_string(__LINE__);
    }

    auto TypeAsScalar = Type->SAs<ScalarType>();
    i64 ActVal;
    if (Size == 1) {
        ActVal = StateVector->ReadByte(Offset);
    } else if (Size == 2) {
        ActVal = StateVector->ReadShort(Offset);
    } else {
        ActVal = StateVector->ReadWord(Offset);
    }

    ActVal = ActVal + Low;
    // return (TypeAsScalar->ValToConst(ActVal) + " { Offset : " + to_string(Offset) +
    //         ", Raw Value : " + to_string(ActVal) + " }");
    return TypeAsScalar->ValToConst(ActVal);
}

void StateVecPrinter::MakePrinters(const ExpT& Exp, LabelledTS* TheLTS)
{
    auto Mgr = TheLTS->GetMgr();
    Compiler->CompileExp(Exp, TheLTS);
    auto Type = Exp->GetType();
    if (Type->Is<ScalarType>()) {
        auto Offset = Exp->ExtensionData.Offset;
        ExpsToPrint.push_back(make_pair(Exp, ScalarPrinter(Offset, Type)));
    } else if (Type->Is<ArrayType>()) {
        auto TypeAsArr = Type->SAs<ArrayType>();
        auto const& IndexType = TypeAsArr->GetIndexType();
        auto&& IndexElems = IndexType->GetElementsNoUndef();
        for (auto const& Elem : IndexElems) {
            MakePrinters(Mgr->MakeExpr(LTSOps::OpIndex, Exp,
                                       Mgr->MakeVal(Elem, IndexType)),
                         TheLTS);
        }
    } else if (Type->Is<RecordType>()) {
        auto TypeAsRec = Type->SAs<RecordType>();
        auto const& MemberVec = TypeAsRec->GetMemberVec();
        auto const& FAType = Mgr->MakeType<FieldAccessType>();
        for (auto const& MemType : MemberVec) {
            if (Type == TheLTS->GetUnifiedMType() &&
                MemType.first ==
                TheLTS->GetUnifiedMType()->SAs<UnionType>()->GetTypeIDFieldName()) {
                auto FieldExp = Mgr->MakeExpr(LTSOps::OpField, Exp,
                                              Mgr->MakeVar(MemType.first, FAType));
                Compiler->CompileExp(FieldExp, TheLTS);

                auto Offset = FieldExp->ExtensionData.Offset;
                ExpsToPrint.push_back(make_pair(FieldExp,
                                                ScalarPrinter(Offset,
                                                              TheLTS->GetMsgTypeMap())));
                continue;
            }
            MakePrinters(Mgr->MakeExpr(LTSOps::OpField, Exp,
                                       Mgr->MakeVar(MemType.first, FAType)),
                         TheLTS);
        }
    }
}

StateVecPrinter::StateVecPrinter(LabelledTS* TheLTS, LTSCompiler* Compiler)
    : Compiler(Compiler)
{
    auto const& StateVectorVars = TheLTS->GetStateVectorVars();
    for (auto const& Var : StateVectorVars) {
        MakePrinters(Var, TheLTS);
    }
}

StateVecPrinter::~StateVecPrinter()
{
    // Nothing here
}

vector<string> StateVecPrinter::PrintState(const StateVec* StateVector) const
{
    vector<string> Retval;
    for (auto const& ExpPrinter : ExpsToPrint) {
        auto const& Exp = ExpPrinter.first;
        auto const& Printer = ExpPrinter.second;
        Retval.push_back(Exp->ToString() + " : " + Printer.Print(StateVector));
    }
    return Retval;
}

vector<string> StateVecPrinter::PrintState(const StateVec* StateVector,
                                           const StateVec* PrevStateVector) const
{
    auto&& CurState = PrintState(StateVector);
    auto&& PrevState = PrintState(PrevStateVector);

    const u32 NumLines = CurState.size();
    vector<string> Retval;

    for (u32 i = 0; i < NumLines; ++i) {
        if (CurState[i] != PrevState[i]) {
            Retval.push_back(CurState[i]);
        }
    }
    return Retval;
}

void StateVecPrinter::PrintState(const StateVec* StateVector, ostream& Out) const
{
    auto&& Lines = PrintState(StateVector);
    for (auto const& Line : Lines) {
        Out << Line << endl;
    }
}

void StateVecPrinter::PrintState(const StateVec* StateVector,
                                 const StateVec* PrevStateVector,
                                 ostream& Out) const
{
    auto&& Lines = PrintState(StateVector, PrevStateVector);
    for (auto const& Line : Lines) {
        Out << Line << endl;
    }
}

void StateVecPrinter::PrintState(const ProductState* State, ostream& Out) const
{
    auto&& Lines = PrintState(State->GetSVPtr());
    for (auto const& Line : Lines) {
        Out << Line << endl;
    }

    Out << "Monitor Tracked Index: " << State->GetIndexID() << endl;
    Out << "Monitor State: " << State->GetMonitorState() << endl;
}

void StateVecPrinter::PrintState(const ProductState* State,
                                 const ProductState* Prev,
                                 ostream& Out) const
{
    auto&& Lines = PrintState(State->GetSVPtr(), Prev->GetSVPtr());
    for (auto const& Line : Lines) {
        Out << Line << endl;
    }

    Out << "Monitor Tracked Index: " << State->GetIndexID() << endl;
    Out << "Monitor State: " << State->GetMonitorState() << endl;
}

void StateVecPrinter::PrintState(const ProductState* State,
                                 const ProductStructure* ThePS,
                                 ostream& Out) const
{
    auto&& Lines = PrintState(State->GetSVPtr());
    for (auto const& Line : Lines) {
        Out << Line << endl;
    }

    auto Monitor = ThePS->GetMonitor();
    Out << "Monitor Tracked Index: " << State->GetIndexID() << endl;
    Out << "Monitor State: " <<
        Monitor->GetStateNameForID(State->GetMonitorState())
        << (Monitor->IsAccepting(State->GetMonitorState()) ? " (accepting)" : "")
        << endl;
}

void StateVecPrinter::PrintState(const ProductState* State,
                                 const ProductState* Prev,
                                 const ProductStructure* ThePS,
                                 ostream& Out) const
{
    auto&& Lines = PrintState(State->GetSVPtr(), Prev->GetSVPtr());
    for (auto const& Line : Lines) {
        Out << Line << endl;
    }

    auto Monitor = ThePS->GetMonitor();
    Out << "Monitor Tracked Index: " << State->GetIndexID() << endl;
    Out << "Monitor State: " <<
        Monitor->GetStateNameForID(State->GetMonitorState())
        << (Monitor->IsAccepting(State->GetMonitorState()) ? " (accepting)" : "")
        << endl;
}
}

} /* end namespace ESMC */

//
// StateVecPrinter.cpp ends here
