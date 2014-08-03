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
    auto ParamType = Mgr->MakeType<ExprParametricType>(RecType, SymmType);
    
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

    // Test for union types
    vector<pair<string, ExprTypeRef>> RecType1Fields = { { (string)"IntField", IntType },
                                                         { (string)"BoolField", BoolType } };
    
    vector<pair<string, ExprTypeRef>> RecType2Fields = { { (string)"RangeField", RangeType },
                                                         { (string)"BoolField", BoolType } };

    auto RecType1 = Mgr->MakeType<ExprRecordType>("RecType1", RecType1Fields);
    auto RecType2 = Mgr->MakeType<ExprRecordType>("RecType2", RecType2Fields);

    set<ExprTypeRef> UnionMem = { RecType1, RecType2 };
    auto UnionType = Mgr->MakeType<ExprUnionType>("UType", UnionMem);
    
    auto UnionVar = Mgr->MakeVar("UnionVar", UnionType);
    auto UFAType = Mgr->MakeType<ExprUFAType>(RecType2);
    FieldVar = Mgr->MakeVar("RangeField", UFAType);
    
    FieldExp = Mgr->MakeExpr(Ops::OpField, UnionVar, FieldVar);

    cout << FieldExp << endl;

    auto UFAType2 = Mgr->MakeType<ExprUFAType>(RecType1);
    FieldVar = Mgr->MakeVar("IntField", UFAType2);
    FieldExp = Mgr->MakeExpr(Ops::OpField, UnionVar, FieldVar);
    
    cout << FieldExp << endl;

    // This should throw an error
    FieldVar2 = Mgr->MakeVar("RangeField", UFAType2);
    try {
        FieldExp = Mgr->MakeExpr(Ops::OpField, UnionVar, FieldVar2);
    } catch (const ExprTypeError& Ex) {
        cout << "Caught exception on badly typed union field access as expected" << endl;
        goto PastError1;
    }

    cout << "Error! Did not throw exception on badly typed union field access" << endl;

 PastError1:

    // Test for arrays of unions of records
    auto ArrayOfRecType = Mgr->MakeType<ExprArrayType>(RangeType, UnionType);
    auto ArrayVar = Mgr->MakeVar("ArrayOfRecVar", ArrayOfRecType);
    IndexExp = Mgr->MakeExpr(Ops::OpIndex, ArrayVar, Mgr->MakeVal("0", RangeType));
    FieldExp = Mgr->MakeExpr(Ops::OpField, IndexExp, FieldVar);
    cout << FieldExp << endl;

    // This should throw an error
    try {
        FieldExp = Mgr->MakeExpr(Ops::OpField, IndexExp, FieldVar2);
    } catch (const ExprTypeError& Ex) {
        cout << "Caught exception on badly typed union field access as expected" << endl;
        goto PastError2;
    }

    cout << "Error! Did not throw exception on badly typed union field access" << endl;

 PastError2:

    Mgr->GC();
    delete Mgr;
}

// 
// SemTest.cpp ends here
