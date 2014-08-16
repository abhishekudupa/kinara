// SemTest.cpp --- 
// 
// Filename: SemTest.cpp
// Author: Abhishek Udupa
// Created: Sat Jul 26 15:50:12 2014 (-0400)
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

#include "../../src/expr/Expressions.hpp"
#include "../../src/uflts/LTSTermSemanticizer.hpp"

using namespace ESMC;
using namespace Exprs;
using namespace LTS;

int main()
{
    typedef ExprMgr<EmptyExtType, LTSTermSemanticizer> MgrType;

    auto Mgr = MgrType::Make();
    
    typedef MgrType::SemT SemT;
    typedef SemT::Ops Ops;

    // Tests for basic variable creation
    auto IntType = Mgr->MakeType<ExprIntType>();
    auto BoolType = Mgr->MakeType<ExprBoolType>();
    auto RangeType = Mgr->MakeType<ExprRangeType>(0, 100);

    auto AExp = Mgr->MakeVar("DummyA", IntType);
    auto BExp = Mgr->MakeVar("DummyB", RangeType);
    
    auto APlusBExp = Mgr->MakeExpr(Ops::OpADD, AExp, BExp);
    cout << APlusBExp << endl;

    AExp = BExp = APlusBExp = nullptr;

    // Tests for uninterpreted function and simplification
    vector<MgrType::TypeT> FArgTypes(3);
    FArgTypes[0] = IntType;
    FArgTypes[1] = RangeType;
    FArgTypes[2] = BoolType;

    auto FOp = Mgr->MakeUninterpretedFunction("f", FArgTypes, RangeType);
    AExp = Mgr->MakeVar("A", IntType);
    BExp = Mgr->MakeVar("B", BoolType);

    auto TenExp = Mgr->MakeVal("0", RangeType);
    auto OneExp = Mgr->MakeVal("1", RangeType);
    for(u32 i = 0; i < 10; ++i) {
        TenExp = Mgr->MakeExpr(Ops::OpADD, TenExp, OneExp);
    }
    
    auto FApp = Mgr->MakeExpr(FOp, AExp, TenExp, BExp);
    cout << FApp << endl;

    FApp = Mgr->Simplify(FApp);
    cout << "After Simplification:" << endl << FApp << endl;

    // Tests for record and array accesses
    auto ArrayType = Mgr->MakeType<ExprArrayType>(RangeType, RangeType);
    auto ArrVar = Mgr->MakeVar("ArrA", ArrayType);
    auto IndexExp = Mgr->MakeExpr(Ops::OpIndex, ArrVar, TenExp);

    cout << IndexExp << endl;
    IndexExp = Mgr->Simplify(IndexExp);
    
    cout << "After Simplification:" << endl;
    cout << IndexExp << endl;


    // Record access, param types and symmetric types
    auto SymmType = Mgr->MakeType<ExprSymmetricType>("SymmType", 4);
    vector<pair<string, ExprTypeRef>> RecMemVec;

    RecMemVec.push_back(make_pair((string)"RangeField", RangeType));
    RecMemVec.push_back(make_pair((string)"BoolField", BoolType));
    RecMemVec.push_back(make_pair((string)"IntField", IntType));

    auto RecType = Mgr->MakeType<ExprRecordType>("RecType", RecMemVec);
    vector<ExprTypeRef> ParamArgTypes = { SymmType };
    auto ParamType = Mgr->MakeType<ExprParametricType>(RecType, ParamArgTypes);
    
    cout << "ParamType: " << endl << ParamType->ToString() << endl;
    
    auto FAType = Mgr->MakeType<ExprFieldAccessType>();
    auto ConstExp = Mgr->MakeVal("SymmType::0", SymmType);
    vector<MgrType::ExpT> ParamVals(1);
    ParamVals[0] = ConstExp;
    auto InstType = Mgr->InstantiateType(ParamType, ParamVals);

    cout << "Instantiated Param Type: " << endl << InstType->ToString() << endl;
    
    auto RecVar = Mgr->MakeVar("RecVar", InstType);
    auto FieldVar = Mgr->MakeVar("IntField", FAType);


    auto FieldExp = Mgr->MakeExpr(Ops::OpField, RecVar, FieldVar);

    cout << "Index Expression: " << IndexExp << endl;
    cout << "Field Expression: " << FieldExp << endl;

    auto FieldVar2 = Mgr->MakeVar("RangeField", FAType);
    auto FieldExp2 = Mgr->MakeExpr(Ops::OpField, RecVar, FieldVar2);

    auto ComplexExp = Mgr->MakeExpr(Ops::OpADD, FieldExp2, TenExp);
    cout << "Field Exp before Simplficiation: " << endl << ComplexExp << endl;
    cout << "Field Exp after Simplficiation: " << endl << Mgr->Simplify(ComplexExp) << endl;

    // tests for message types
    vector<pair<string, ExprTypeRef>> MType1Fields;
    MType1Fields.push_back(make_pair("IntField1", IntType));
    MType1Fields.push_back(make_pair("BoolField1", BoolType));

    vector<pair<string, ExprTypeRef>> MType2Fields;
    MType2Fields.push_back(make_pair("RangeField2", RangeType));
    MType2Fields.push_back(make_pair("BoolField2", BoolType));
    
    auto MType1 = Mgr->MakeType<ExprRecordType>("MType1", MType1Fields);
    auto MType2 = Mgr->MakeType<ExprRecordType>("MType2", MType2Fields);
    
    auto IDType = Mgr->MakeType<ExprRangeType>(0, 10);
    set<ExprTypeRef> UMTypeMem;
    UMTypeMem.insert(MType1);
    UMTypeMem.insert(MType2);

    auto UMType = Mgr->MakeType<ExprUnionType>("UMType", UMTypeMem, RangeType);

    auto UMTypeVar = Mgr->MakeVar("UMTypeVar", UMType);
    auto UMTypeAsMsg = UMType->As<ExprUnionType>();
    auto const& ActFieldName = UMTypeAsMsg->MapFromMemberField(MType1, "IntField1");
    auto UMTypeExp = Mgr->MakeExpr(Ops::OpField, UMTypeVar, 
                                   Mgr->MakeVar(ActFieldName, FAType));
    cout << UMTypeExp << endl;
    auto const& ActFieldName2 = UMTypeAsMsg->MapFromMemberField(MType2, "BoolField2");
    
    auto UMTypeExp2 = Mgr->MakeExpr(Ops::OpField, UMTypeVar, 
                                    Mgr->MakeVar(ActFieldName2, FAType));

    cout << UMTypeExp2 << endl;

    Mgr->GC();
    delete Mgr;
}

// 
// SemTest.cpp ends here
