// ExprTests.cpp --- 
// 
// Filename: ExprTests.cpp
// Author: Abhishek Udupa
// Created: Tue Jul  8 05:21:10 2014 (-0400)
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

#include "../../src/exprmgr/Expressions.hpp"
#include "../../src/exprmgr/Z3Semanticizer.hpp"

using namespace ESMC;
using namespace Exprs;
using namespace Z3Sem::Z3SemOps;

int main()
{
    typedef ExprMgr<EmptyExtType, Z3Sem::Z3Semanticizer> MgrType;
    auto Mgr = MgrType::Make();
    auto X = Mgr->MakeVar("X", IntType);
    auto Y = Mgr->MakeVar("Y", IntType);
    auto Z = Mgr->MakeVar("Z", IntType);

    auto EVar = Mgr->MakeBoundVar(IntType, 0);

    auto XLTZ = Mgr->MakeExpr(OpLT, X, Z);
    auto ZLTY = Mgr->MakeExpr(OpLT, Z, Y);
    auto QBody = Mgr->MakeExpr(OpAND, XLTZ, ZLTY);

    cout << "Before Substitution: " << QBody << endl;

    MgrType::SubstMapT SubstMap = { { Z, EVar } };
    
    QBody = Mgr->Substitute(SubstMap, QBody);
    
    cout << "After Substitution: " << QBody << endl;

    vector<i64> QVarTypes = { IntType };
    auto QExpr = Mgr->MakeExists(QVarTypes, QBody);

    cout << "After Quantification: " << QExpr << endl;

    auto ElimExpr = Mgr->ElimQuantifiers(QExpr);
    cout << "After Quantifier Elimination: " << ElimExpr << endl;

    delete Mgr;
}

// 
// ExprTests.cpp ends here
