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

#include "../../src/expr/Expressions.hpp"
#include "../../src/expr/Z3Semanticizer.hpp"
#include "../../src/symexec/Analyses.hpp"

using namespace ESMC;
using namespace Exprs;
using namespace Analyses;

int main()
{
    typedef ExprMgr<EmptyExtType, Z3Sem::Z3Semanticizer> MgrType;
    typedef Assignment<EmptyExtType, Z3Sem::Z3Semanticizer> AssignT;
    typedef MgrType::SemT SemT;
    typedef SemT::Ops Ops;
    auto Mgr = MgrType::Make();

    auto X = Mgr->MakeVar("X", Ops::IntType);
    auto Y = Mgr->MakeVar("Y", Ops::IntType);
    auto Z = Mgr->MakeVar("Z", Ops::IntType);
    auto XPlus1 = Mgr->MakeExpr(Ops::OpADD, X, Mgr->MakeVal("1", Ops::IntType));
    auto XPlusY = Mgr->MakeExpr(Ops::OpADD, X, Y);
    auto YPlusZ = Mgr->MakeExpr(Ops::OpADD, Y, Z);

    auto Assign1 = AssignT(X, XPlus1);
    auto Assign2 = AssignT(Y, XPlusY);

    vector<AssignT> AsgnVec = { Assign1, Assign2 };

    auto Pred1 = Mgr->MakeExpr(Ops::OpAND, 
                               Mgr->MakeExpr(Ops::OpEQ, X, Mgr->MakeVal("0", Ops::IntType)),
                               Mgr->MakeExpr(Ops::OpEQ, Y, Mgr->MakeVal("1", Ops::IntType)));

    auto PC1 = StrongestPostSeq(Pred1, AsgnVec.begin(), AsgnVec.end(), true, true);
    cout << "Seq SP of " << Pred1 << endl;
    cout << "After statements: " << endl;
    cout << Assign1 << endl;
    cout << Assign2 << endl;
    cout << "is:" << endl;
    cout << PC1 << endl;

    auto PC2 = StrongestPostPar(Pred1, AsgnVec.begin(), AsgnVec.end(), true);
    cout << "Par SP of " << Pred1 << endl;
    cout << "After statements: " << endl;
    cout << Assign1 << endl;
    cout << Assign2 << endl;
    cout << "is:" << endl;
    cout << PC2 << endl;
    
    // Weakest preconditions
    auto PreCond = Mgr->MakeExpr(Ops::OpGT, X, Mgr->MakeExpr(Ops::OpADD, Mgr->MakeVal("10", Ops::IntType), Y));
    auto WP1 = WeakestPreSeq(PreCond, AsgnVec.rbegin(), AsgnVec.rend());

    cout << "Seq WP of " << PreCond << endl;
    cout << "wrt statements: " << endl;
    cout << Assign1 << endl;
    cout << Assign2 << endl;
    cout << "is:" << endl;
    cout << WP1 << endl;

    auto WP2 = WeakestPrePar(PreCond, AsgnVec.rbegin(), AsgnVec.rend());

    cout << "Par WP of " << PreCond << endl;
    cout << "wrt statements: " << endl;
    cout << Assign1 << endl;
    cout << Assign2 << endl;
    cout << "is:" << endl;
    cout << WP2 << endl;
}

// 
// ExprTests.cpp ends here
