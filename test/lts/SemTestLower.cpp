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

    auto RangeType = Mgr->MakeType<ExprRangeType>(0, 10);
    auto VarA = Mgr->MakeVar("a", RangeType);
    auto VarB = Mgr->MakeVar("b", RangeType);

    auto APlusB = Mgr->MakeExpr(LTSOps::OpADD, VarA, VarB);
    cout << "Original Expression: " << APlusB << endl;

    LTSLCRef LoweringCtx = new LTSLoweredContext();
    auto LoweredAPlusB = Mgr->LowerExpr(APlusB, LoweringCtx);
    cout << "Lowered Expression: " << LoweredAPlusB.ToString() << endl;

    // raise it back
    auto RaisedAPlusB = Mgr->RaiseExpr(LoweredAPlusB, LoweringCtx);
    cout << "Raised Expression: " << RaisedAPlusB << endl;

    auto const& AllAssumptions = LoweringCtx->GetAllAssumptions();
    cout << "Number of assumption scopes: " << AllAssumptions.size() << endl;

    
    u32 CurScope = 0;
    for (auto const& AssumptionScope : AllAssumptions) {
        cout << "Assumption Scope " << CurScope++ << endl;

        for (auto const& Assumption : AssumptionScope) {
            cout << Assumption.ToString() << endl;
        }
    }

    auto APlusBPlusA = Mgr->MakeExpr(LTSOps::OpADD, APlusB, VarA);
    auto LoweredAPlusBPlusA = Mgr->LowerExpr(APlusBPlusA, LoweringCtx);
    cout << "Lowered Expression: " << LoweredAPlusBPlusA.ToString() << endl;

    // raise it back
    auto RaisedAPlusBPlusA = Mgr->RaiseExpr(LoweredAPlusBPlusA, LoweringCtx);
    cout << "Raised Expression: " << RaisedAPlusBPlusA << endl;

    auto const& AllAssumptions2 = LoweringCtx->GetAllAssumptions();
    cout << "Number of assumption scopes: " << AllAssumptions2.size() << endl;
    
    CurScope = 0;
    for (auto const& AssumptionScope : AllAssumptions2) {
        cout << "Assumption Scope " << CurScope++ << endl;

        for (auto const& Assumption : AssumptionScope) {
            cout << Assumption.ToString() << endl;
        }
    }
    
}

// 
// SemTest.cpp ends here
