// SimpTests.cpp ---
//
// Filename: SimpTests.cpp
// Author: Abhishek Udupa
// Created: Wed Jan 14 18:43:28 2015 (-0500)
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
#include "../../src/uflts/LTSDecls.hpp"

using namespace ESMC::LTS;
using namespace ESMC::Exprs;

int main()
{
    auto Mgr = MgrT::Make();

    auto IndexType = Mgr->MakeType<SymmetricType>("SymmType", 3);
    auto IntType = Mgr->MakeType<RangeType>(0, 100);
    auto ArrType = Mgr->MakeType<ArrayType>(IndexType, IntType);

    auto ArrayExp = Mgr->MakeVar("ArrVar", ArrType);
    auto OneExp = Mgr->MakeVal("1", IntType);
    auto ZeroExp = Mgr->MakeVal("0", IntType);
    auto TwoExp = Mgr->MakeVal("2", IntType);

    auto IndexZeroExp = Mgr->MakeVal("SymmType::0", IndexType);
    auto IndexOneExp = Mgr->MakeVal("SymmType::1", IndexType);
    auto IndexTwoExp = Mgr->MakeVal("SymmType::2", IndexType);

    auto StoreExp = Mgr->MakeExpr(LTSOps::OpStore, ArrayExp, IndexOneExp, OneExp);
    auto SelectExp = Mgr->MakeExpr(LTSOps::OpSelect, StoreExp, IndexOneExp);

    auto SimpExp = Mgr->Simplify(SelectExp);
    cout << SimpExp->ToString() << endl;

    auto BoolType = Mgr->MakeType<BooleanType>();
    auto BoolVarA = Mgr->MakeVar("BoolVarA", BoolType);
    auto BoolVarB = Mgr->MakeVar("BoolVarB", BoolType);
    auto BoolVarC = Mgr->MakeVar("BoolVarC", BoolType);

    auto AAndB = Mgr->MakeExpr(LTSOps::OpAND, BoolVarA, BoolVarB);
    auto AAndBAndC = Mgr->MakeExpr(LTSOps::OpAND, AAndB, BoolVarC);

    cout << AAndBAndC->ToString() << endl
         << Mgr->SimplifyFP(AAndBAndC) << endl;

    // Test case for array record simplifier
    auto CacheIDType = Mgr->MakeType<SymmetricType>("CacheIDType", 2);
    auto AddressType = Mgr->MakeType<SymmetricType>("AddressType", 1);
    auto DirIDType = Mgr->MakeType<SymmetricType>("DirIDType", 1);
    auto SharersType = Mgr->MakeType<ArrayType>(CacheIDType, BoolType);
    vector<pair<string, TypeRef>> DirMembers;
    DirMembers.push_back(make_pair("Sharers", SharersType));
    DirMembers.push_back(make_pair("ActiveID", CacheIDType));
    auto DirRecType = Mgr->MakeType<RecordType>("Directory", DirMembers);
    auto DirArrayType = Mgr->MakeType<ArrayType>(AddressType, DirRecType);
    DirArrayType = Mgr->MakeType<ArrayType>(DirIDType, DirArrayType);
    auto FAType = Mgr->MakeType<FieldAccessType>();

    auto DirID0 = Mgr->MakeVal("DirIDType::0", DirIDType);
    auto Address0 = Mgr->MakeVal("AddressType::0", AddressType);
    auto CacheID0 = Mgr->MakeVal("CacheIDType::0", CacheIDType);
    auto CacheID1 = Mgr->MakeVal("CacheIDType::1", CacheIDType);
    auto TrueExp = Mgr->MakeTrue();
    auto FalseExp = Mgr->MakeFalse();
    auto DirVar = Mgr->MakeVar("Directory", DirArrayType);
    auto DirExp = Mgr->MakeExpr(LTSOps::OpSelect, DirVar, DirID0);
    DirExp = Mgr->MakeExpr(LTSOps::OpSelect, DirExp, Address0);

    auto DirDotSharers = Mgr->MakeExpr(LTSOps::OpProject, DirExp,
                                       Mgr->MakeVar("Sharers", FAType));
    auto DirDotActiveID = Mgr->MakeExpr(LTSOps::OpProject, DirExp,
                                        Mgr->MakeVar("ActiveID", FAType));

    auto TestArrayExp = Mgr->MakeExpr(LTSOps::OpStore, DirDotSharers, CacheID0, FalseExp);
    TestArrayExp = Mgr->MakeExpr(LTSOps::OpStore, TestArrayExp, CacheID1, FalseExp);
    TestArrayExp = Mgr->MakeExpr(LTSOps::OpStore, TestArrayExp, DirDotActiveID, TrueExp);

    auto TestExp = Mgr->MakeExpr(LTSOps::OpSelect, TestArrayExp, CacheID1);

    cout << TestExp->ToString() << endl;

    SimpExp = Mgr->Simplify(TestExp);
    cout << SimpExp << endl;

}

//
// SimpTests.cpp ends here
