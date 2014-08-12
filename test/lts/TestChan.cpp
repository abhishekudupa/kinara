// TestChan.cpp --- 
// 
// Filename: TestChan.cpp
// Author: Abhishek Udupa
// Created: Tue Aug 12 00:23:05 2014 (-0400)
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

#include "../../src/uflts/LabelledTS.hpp"
#include "../../src/uflts/LTSEFSM.hpp"

using namespace ESMC;
using namespace LTS;
using namespace Exprs;

int main()
{
    auto TheLTS = new LabelledTS();
    auto Mgr = TheLTS->GetMgr();
    auto SymmType = TheLTS->MakeSymmType("IDType", 2);
    auto ChanSymmType = TheLTS->MakeSymmType("ChanIDType", 2);
    vector<ExpT> Params;
    Params.push_back(Mgr->MakeVar("ParamVar", SymmType));
    auto TrueExp = Mgr->MakeTrue();

    set<string> EnumMembers = { "EnumConst1", "EnumConst2", "EnumConst3" };
    auto EnumType = Mgr->MakeType<ExprEnumType>("EnumType", EnumMembers);

    vector<pair<string, ExprTypeRef>> Rec1Fields;
    Rec1Fields.push_back(make_pair("BoolField", Mgr->MakeType<ExprBoolType>()));
    Rec1Fields.push_back(make_pair("RangeField", Mgr->MakeType<ExprRangeType>(0, 100)));

    vector<pair<string, ExprTypeRef>> Rec2Fields;
    Rec2Fields.push_back(make_pair("RangeField", Mgr->MakeType<ExprRangeType>(0, 100)));
    Rec2Fields.push_back(make_pair("EnumField", EnumType));

    auto MType1 = TheLTS->MakeMsgTypes(Params, TrueExp, "MType1", Rec1Fields, true);
    auto MType2 = TheLTS->MakeMsgTypes(Params, TrueExp, "MType2", Rec2Fields, true);

    TheLTS->FreezeMsgs();
    
    vector<ExpT> ChanParams;
    ChanParams = { Mgr->MakeVar("ChanID", ChanSymmType) };

    auto Chan1 = TheLTS->MakeChannel("Chan1", ChanParams, TrueExp, 4, true, false, true, false, LTSFairnessType::Strong);
    Chan1->AddMsgs(Params, TrueExp, MType1, Params, LTSFairnessType::Strong, LossDupFairnessType::NotAlwaysLostOrDup);

    auto Chan2 = TheLTS->MakeChannel("Chan2", ChanParams, TrueExp, 3, true, true, true, false, LTSFairnessType::Strong);
    Chan2->AddMsgs(Params, TrueExp, MType2, Params, LTSFairnessType::Strong, LossDupFairnessType::NotAlwaysLostOrDup);
    
    cout << Chan1->ToString() << endl;
    cout << endl << endl << endl << endl;
    cout << Chan2->ToString() << endl;
    
    TheLTS->FreezeAutomata();

    delete TheLTS;
}

// 
// TestChan.cpp ends here
















