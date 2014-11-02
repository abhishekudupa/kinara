// LTSEFSMBase.cpp --- 
// 
// Filename: LTSEFSMBase.cpp
// Author: Abhishek Udupa
// Created: Fri Aug 15 12:10:58 2014 (-0400)
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

#include "LTSEFSMBase.hpp"
#include "LTSUtils.hpp"
#include "LabelledTS.hpp"
#include "LTSState.hpp"
#include "LTSTransitions.hpp"
#include "LTSFairnessSet.hpp"

namespace ESMC {
    namespace LTS {

        // Symmetric message decl implementation
        SymmetricMessageDecl::SymmetricMessageDecl(const ExprTypeRef& MessageType,
                                                   const vector<ExpT>& NewParams,
                                                   const ExpT& Constraint,
                                                   const vector<ExpT>& MessageParams,
                                                   bool Input)
            : MessageType(MessageType), NewParams(NewParams),
              Constraint(Constraint), MessageParams(MessageParams),
              Input(Input)
        {
            // Nothing here
        }

        SymmetricMessageDecl::~SymmetricMessageDecl()
        {
            // Nothing here
        }

        const ExprTypeRef& SymmetricMessageDecl::GetMessageType() const
        {
            return MessageType;
        }

        const vector<ExpT>& SymmetricMessageDecl::GetNewParams() const
        {
            return NewParams;
        }

        const ExpT& SymmetricMessageDecl::GetConstraint() const
        {
            return Constraint;
        }

        const vector<ExpT>& SymmetricMessageDecl::GetMessageParams() const
        {
            return MessageParams;
        }

        bool SymmetricMessageDecl::IsInput() const
        {
            return Input;
        }

        bool SymmetricMessageDecl::IsOutput() const
        {
            return (!Input);
        }

        string SymmetricMessageDecl::ToString() const
        {
            ostringstream sstr;
            if (Input) {
                sstr << "Input Message: ";
            } else {
                sstr << "Output Message: ";
            }
            if (NewParams.size() > 0) {
                sstr << "New Params: " << endl;
                for (auto const& NewParam : NewParams) {
                    sstr << "    " << NewParam->ToString() << endl;
                }
                sstr << "Constraint: " << Constraint->ToString() << endl;
            }
            sstr << "Message Type: " << MessageType->ToString() << endl;
            sstr << "Message Params: " << endl;
            for (auto const& MessageParam : MessageParams) {
                sstr << "    " << MessageParam->ToString() << endl;
            }
            sstr << endl;
            return sstr.str();
        }

        EFSMBase::EFSMBase(LabelledTS* TheLTS, const string& Name,
                           const vector<ExpT>& Params, const ExpT& Constraint,
                           LTSFairnessType Fairness)
            : AutomatonBase(TheLTS, Name, Params, Constraint),
              Fairness(Fairness), VarsFrozen(false), EFSMFrozen(false),
              Fairnesses(new LTSProcessFairnessGroup(this))
        {
            if (Fairness != LTSFairnessType::None) {
                auto FairsetFairness = Fairness == LTSFairnessType::Weak ? 
                    FairSetFairnessType::Weak : FairSetFairnessType::Strong;

                Fairnesses->AddFairnessSet("ProcessFairness", FairsetFairness);
            }
        }

        EFSMBase::~EFSMBase()
        {
            // Nothing here
        }

        set<ExprTypeRef> EFSMBase::GetInputs() const
        {
            set<ExprTypeRef> Retval;
            for (auto const& InstType : Inputs) {
                Retval.insert(InstType.second.begin(), InstType.second.end());
            }
            return Retval;
        }

        set<ExprTypeRef> EFSMBase::GetOutputs() const
        {
            set<ExprTypeRef> Retval;
            for (auto const& InstType : Outputs) {
                Retval.insert(InstType.second.begin(), InstType.second.end());
            }
            return Retval;
        }

        set<ExprTypeRef> EFSMBase::GetInputsForInstance(u32 InstanceID) const
        {
            auto it = Inputs.find(ParamInsts[InstanceID]);
            if (it == Inputs.end()) {
                return set<ExprTypeRef>();
            }
            return it->second;
        }

        set<ExprTypeRef> EFSMBase::GetOutputsForInstance(u32 InstanceID) const
        {
            auto it = Outputs.find(ParamInsts[InstanceID]);
            if (it == Outputs.end()) {
                return set<ExprTypeRef>();
            }
            return it->second;
        }

        vector<LTSTransRef> 
        EFSMBase::GetOutputTransitionsOnMsg(const ExprTypeRef& MsgType) const
        {
            vector<LTSTransRef> Retval;

            for (auto const& ParamInst : ParamInsts) {
                auto it = Outputs.find(ParamInst);
                if (it == Outputs.end()) {
                    continue;
                }
                auto const& OutputSet = it->second;
                auto it2 = OutputSet.find(MsgType);
                if (it2 == OutputSet.end()) {
                    continue;
                }

                auto it3 = Transitions.find(ParamInst);
                if (it3 == Transitions.end()) {
                    continue;
                }
                
                auto const& TransVec = it3->second;
                for (auto const& Trans : TransVec) {
                    if (Trans->Is<LTSTransitionOutput>()) {
                        auto TransAsOut = Trans->SAs<LTSTransitionOutput>();
                        auto const& TMsgType = TransAsOut->GetMessageType();
                        if (TMsgType == MsgType) {
                            Retval.push_back(Trans);
                        }
                    }
                }
            }
            return Retval;
        }

        vector<vector<LTSTransRef>> 
        EFSMBase::GetInputTransitionsOnMsg(const ExprTypeRef& MsgType) const
        {
            vector<vector<LTSTransRef>> Retval;
            
            for (auto const& ParamInst : ParamInsts) {

                vector<LTSTransRef> RetvalComp;

                auto it = Inputs.find(ParamInst);
                if (it == Inputs.end()) {
                    continue;
                }
                auto const& InputSet = it->second;
                auto it2 = InputSet.find(MsgType);
                if (it2 == InputSet.end()) {
                    continue;
                }
                auto it3 = Transitions.find(ParamInst);
                if (it3 == Transitions.end()) {
                    continue;
                }

                auto const& TransVec = it3->second;
                
                for (auto const& Trans : TransVec) {
                    if (Trans->Is<LTSTransitionInput>()) {
                        auto TransAsIn = Trans->SAs<LTSTransitionInput>();
                        auto const& TMsgType = TransAsIn->GetMessageType();
                        if (TMsgType == MsgType) {
                            RetvalComp.push_back(Trans);
                        }
                    }
                }

                if (RetvalComp.size() > 0) {
                    Retval.push_back(RetvalComp);
                }
            }
            
            return Retval;
        }


        vector<LTSTransRef> EFSMBase::GetInternalTransitions() const
        {
            vector <LTSTransRef> Retval;

            for (auto const& InstTrans : Transitions) {
                for (auto const& Trans : InstTrans.second) {
                    if (Trans->Is<LTSTransitionInternal>()) {
                        Retval.push_back(Trans);
                    }
                }
            }
            return Retval;
        }

        void EFSMBase::AddState(const string& StateName,
                                bool Initial, bool Final, 
                                bool Accepting, bool Error)
        {
            if (Initial || Accepting) {
                throw ESMCError((string)"EFSMs cannot have initial states or " + 
                                "accepting states");
            }
            AutomatonBase::AddState(StateName, Initial, Final, Accepting, Error);
        }

        void EFSMBase::AssertVarsFrozen() const
        {
            if (!VarsFrozen) {
                throw ESMCError((string)"The operation attempted needs variables to be frozen");
            }
        }

        void EFSMBase::AssertVarsNotFrozen() const
        {
            if (VarsFrozen) {
                throw ESMCError((string)"The operation attempted needs variables to be not frozen");
            }
        }

        void EFSMBase::AssertEFSMNotFrozen() const
        {
            if (EFSMFrozen) {
                throw ESMCError((string)"Cannot modify EFSM once it has been frozen");
            }
        }

        void EFSMBase::AssertEFSMFrozen() const
        {
            if (!EFSMFrozen) {
                throw ESMCError((string)"EFSM has not been frozen yet");
            }
        }

        void EFSMBase::AssertInput(const vector<ExpT>& ParamInst, 
                                   const ExprTypeRef& MessageType) const
        {
            auto it = Inputs.find(ParamInst);
            if (it == Inputs.end()) {
                throw InternalError((string)"Attempted instantation of msg with invalid " + 
                                    "parameters.\nAt: " + __FILE__ + ":" + to_string(__LINE__));
            }
            auto it2 = it->second.find(MessageType);
            if (it2 == it->second.end()) {
                throw ESMCError((string)"Message type \"" + MessageType->ToString() + "\" is not " + 
                                "an input for machine named \"" + Name + "\"");
            }
        }

        void EFSMBase::AssertOutput(const vector<ExpT>& ParamInst, 
                                    const ExprTypeRef& MessageType) const
        {
            auto it = Outputs.find(ParamInst);
            if (it == Outputs.end()) {
                throw InternalError((string)"Attempted instantation of msg with invalid " + 
                                    "parameters.\nAt: " + __FILE__ + ":" + to_string(__LINE__));
            }
            auto it2 = it->second.find(MessageType);
            if (it2 == it->second.end()) {
                throw ESMCError((string)"Message type \"" + MessageType->ToString() + "\" is not " + 
                                "an output for machine named \"" + Name + "\"");
            }
        }

        void EFSMBase::CheckMsgType(const ExprTypeRef& Type) const
        {
            auto TypeAsRec = Type->As<ExprRecordType>();
            if (TypeAsRec == nullptr) {
                auto TypeAsParam = Type->As<ExprParametricType>();
                if (TypeAsParam == nullptr) {
                    throw ESMCError((string)"Message type \"" + Type->ToString() + 
                                    "\" is not a record type or parametric type");
                }
                TypeAsRec = TypeAsParam->GetBaseType()->As<ExprRecordType>();
            }
            if (!TheLTS->CheckMessageType(Type)) {
                throw ESMCError((string)"Message type \"" + TypeAsRec->GetName() + 
                                "\" has not been registered with the LTS");
            }
        }

        void EFSMBase::CheckFairnessSets(const set<string>& FairnessSetNames) const
        {
            for (auto const& FairnessSetName : FairnessSetNames) {
                (void)Fairnesses->GetFairnessSet(FairnessSetName);
            }
        }

        ExprTypeRef EFSMBase::InstantiateMessageType(const vector<ExpT>& Params, 
                                                     const MgrT::SubstMapT& SubstMap,
                                                     const ExprTypeRef& MsgType)
        {
            auto Mgr = TheLTS->GetMgr();
            if (Params.size() > 0) {
                auto&& SubstVec = SubstAll(Params, SubstMap, Mgr);
                return InstantiateType(MsgType, SubstVec, Mgr);
            } else {
                if (!MsgType->Is<ExprRecordType>()) {
                    throw ESMCError((string)"Message type not plain record type, " + 
                                    "but attempted to instantiate with no params");
                }
                return MsgType;
            }
        }

        SymmMsgDeclRef EFSMBase::AddMsg(const ExprTypeRef& MsgType, 
                                        const vector<ExpT>& Params, 
                                        bool IsInput)
        {
            AssertStatesFrozen();
            AssertEFSMNotFrozen();

            CheckMsgType(MsgType);
            CheckParams(Params, SymTab);
            const u32 NumInsts = ParamInsts.size();

            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];
                
                auto ActMsgType = InstantiateMessageType(Params, SubstMap, MsgType);

                auto const& InpSet = Inputs[ParamInsts[i]];
                auto const& OutSet = Outputs[ParamInsts[i]];


                if (IsInput) {
                    if (OutSet.find(ActMsgType) != OutSet.end()) {
                        throw ESMCError((string)"Message type "  + ActMsgType->ToString()  + 
                                        " conflicts in input/output definition");
                    }

                    Inputs[ParamInsts[i]].insert(ActMsgType);
                } else {
                    if (InpSet.find(ActMsgType) != InpSet.end()) {
                        throw ESMCError((string)"Message type "  + ActMsgType->ToString()  + 
                                        " conflicts in input/output definition");
                    }

                    Outputs[ParamInsts[i]].insert(ActMsgType);
                }
            }
            
            SymmetricMessages.push_back(new SymmetricMessageDecl(MsgType, vector<ExpT>(),
                                                                 this->Constraint,
                                                                 Params, IsInput));
            return SymmetricMessages.back();
        }

        SymmMsgDeclRef EFSMBase::AddMsgs(const vector<ExpT>& NewParams, 
                                         const ExpT& Constraint, 
                                         const ExprTypeRef& MsgType, 
                                         const vector<ExpT>& Params, 
                                         bool IsInput)
        {
            AssertStatesFrozen();
            AssertEFSMNotFrozen();

            auto Mgr = TheLTS->GetMgr();
            CheckMsgType(MsgType);
            SymTab.Push();
            CheckParams(NewParams, Constraint, SymTab, Mgr, true);
            CheckParams(Params, SymTab);
            SymTab.Pop();
            
            const u32 NumInsts = ParamInsts.size();
            const u32 NumNewParams = NewParams.size();
            
            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];
                auto SubstConstraint = Mgr->Substitute(SubstMap, Constraint);
                
                auto&& NewInsts = InstantiateParams(NewParams, SubstConstraint, Mgr);
                const u32 NumNewInsts = NewInsts.size();
                
                for (u32 j = 0; j < NumNewInsts; ++j) {
                    MgrT::SubstMapT LocalSubstMap = SubstMap;
                    auto const& CurNewInst = NewInsts[j];

                    for (u32 k = 0; k < NumNewParams; ++k) {
                        LocalSubstMap[NewParams[k]] = CurNewInst[k];
                    }

                    auto const& InpSet = Inputs[ParamInsts[i]];
                    auto const& OutSet = Outputs[ParamInsts[i]];

                    auto ActMsgType = InstantiateMessageType(Params, LocalSubstMap, MsgType);

                    if (IsInput) {
                        if (OutSet.find(ActMsgType) != OutSet.end()) {
                            throw ESMCError((string)"Message type "  + ActMsgType->ToString()  + 
                                            " conflicts in input/output definition");
                        }
                        if (InpSet.find(ActMsgType) != InpSet.end()) {
                            throw ESMCError((string)"Message type "  + ActMsgType->ToString()  + 
                                            " already declared as input of machine \"" + 
                                            Name + "\"");
                        }
                        
                        Inputs[ParamInsts[i]].insert(ActMsgType);
                    } else {
                        if (InpSet.find(ActMsgType) != InpSet.end()) {
                            throw ESMCError((string)"Message type "  + ActMsgType->ToString()  + 
                                            " conflicts in input/output definition");
                        }
                        if (OutSet.find(ActMsgType) != OutSet.end()) {
                            throw ESMCError((string)"Message type "  + ActMsgType->ToString()  + 
                                            " already declared as output of machine \"" + 
                                            Name + "\"");
                        }

                        
                        Outputs[ParamInsts[i]].insert(ActMsgType);
                    }
                }
            }

            SymmetricMessages.push_back(new SymmetricMessageDecl(MsgType, NewParams, 
                                                                 Constraint, Params,
                                                                 IsInput));
            return SymmetricMessages.back();
        }

        vector<LTSAssignRef> EFSMBase::InstantiateUpdates(const MgrT::SubstMapT& ParamSubst,
                                                          const vector<LTSAssignRef>& Updates,
                                                          const string& MessageName,
                                                          const ExprTypeRef& MessageType,
                                                          const ExprTypeRef& ActMType)
        {
            auto Mgr = TheLTS->GetMgr();
            auto const& SubstMap = ParamSubst;
            
            vector<LTSAssignRef> Retval;

            for (auto const& Update : Updates) {
                auto PUpdate = Update->As<LTSAssignParam>();
                if (PUpdate == nullptr) {
                    // No need to instantiate, just subst out
                    auto LHS = Mgr->Substitute(ParamSubst, Update->GetLHS());
                    auto RHS = Mgr->Substitute(ParamSubst, Update->GetRHS());
                    Retval.push_back(new LTSAssignSimple(LHS, RHS));
                    // That's all folks!
                    continue;
                }

                auto const& UpdateParams = PUpdate->GetParams();
                auto const& UpdateConstraint = PUpdate->GetConstraint();

                auto SubstConstraint = Mgr->Substitute(SubstMap, UpdateConstraint);
                const u32 NumUpdateParams = UpdateParams.size();

                auto&& UpdateInsts = InstantiateParams(UpdateParams, SubstConstraint, Mgr);
                for (auto const& UpdateInst : UpdateInsts) {
                    MgrT::SubstMapT LocalSubstMap = SubstMap;
                    
                    for(u32 i = 0; i < NumUpdateParams; ++i) {
                        LocalSubstMap[UpdateParams[i]] = UpdateInst[i];
                    }
                    
                    auto NewAsgn = 
                        new LTSAssignSimple(Mgr->Substitute(LocalSubstMap, PUpdate->GetLHS()),
                                            Mgr->Substitute(LocalSubstMap, PUpdate->GetRHS()));
                    Retval.push_back(NewAsgn);
                }
            }

            if (MessageName == "") {
                return Retval;
            } else {
                // Need to subst out message types
                vector<LTSAssignRef> Retval2;
                MgrT::SubstMapT LocalSubstMap;
                LocalSubstMap[Mgr->MakeVar(MessageName, MessageType)] = 
                    Mgr->MakeVar(MessageName, ActMType);

                for (auto const& Update : Retval) {
                    
                    auto LHS = Mgr->Substitute(LocalSubstMap, Update->GetLHS());
                    auto RHS = Mgr->Substitute(LocalSubstMap, Update->GetRHS());
                    Retval2.push_back(new LTSAssignSimple(LHS, RHS));
                }
                return Retval2;
            }
        }

        vector<LTSAssignRef> EFSMBase::RebaseUpdates(const vector<ExpT>& ParamInst,
                                                     const vector<LTSAssignRef>& Updates)
        {
            auto Mgr = TheLTS->GetMgr();
            const u32 NumUpdates = Updates.size();
            vector<LTSAssignRef> Retval(NumUpdates);
            auto const& RebaseSubstMap = RebaseSubstMaps[ParamInst];
            
            for (u32 i = 0; i < NumUpdates; ++i) {
                auto RebasedLHS = Mgr->Substitute(RebaseSubstMap, Updates[i]->GetLHS());
                auto RebasedRHS = Mgr->Substitute(RebaseSubstMap, Updates[i]->GetRHS());
                Retval[i] = new LTSAssignSimple(RebasedLHS, RebasedRHS);
            }
            return Retval;
        }

        vector<LTSAssignRef> EFSMBase::MsgTransformUpdates(const vector<LTSAssignRef>& Updates,
                                                           const string& MessageName,
                                                           const ExprTypeRef& MessageType)
        {
            auto Mgr = TheLTS->GetMgr();
            const u32 NumUpdates = Updates.size();
            vector<LTSAssignRef> Retval(NumUpdates);

            for (u32 i = 0; i < NumUpdates; ++i) {
                auto const& LHS = Updates[i]->GetLHS();
                auto const& RHS = Updates[i]->GetRHS();
                auto NewLHS = 
                    Mgr->ApplyTransform<Detail::MsgTransformer>(LHS, MessageName,
                                                                MessageType, 
                                                                TheLTS->GetUnifiedMType());
                auto NewRHS = 
                    Mgr->ApplyTransform<Detail::MsgTransformer>(RHS, MessageName,
                                                                MessageType, 
                                                                TheLTS->GetUnifiedMType());
                Retval[i] = new LTSAssignSimple(NewLHS, NewRHS);
            }
            return Retval;
        }

        vector<LTSAssignRef> EFSMBase::SimplifyUpdates(const vector<LTSAssignRef>& Updates)
        {
            const u32 NumUpdates = Updates.size();
            auto Mgr = TheLTS->GetMgr();
            vector<LTSAssignRef> Retval;

            for (u32 i = 0; i < NumUpdates; ++i) {
                auto SimpLHS = Mgr->Simplify(Updates[i]->GetLHS());
                auto SimpRHS = Mgr->Simplify(Updates[i]->GetRHS());
                
                if (SimpLHS == SimpRHS) {
                    // identity, ignore
                    continue;
                }

                auto SimpUpdate = new LTSAssignSimple(SimpLHS, SimpRHS);
                Retval.push_back(SimpUpdate);
            }
            return Retval;
        }
        

        LTSFairnessType EFSMBase::GetFairnessType() const
        {
            return Fairness;
        }

        void EFSMBase::AddFairnessSet(const string& Name, FairSetFairnessType Fairness)
        {
            AssertStatesFrozen();
            AssertEFSMNotFrozen();
            Fairnesses->AddFairnessSet(Name, Fairness);
        }

        const LTSFairObjRef& EFSMBase::GetFairnessForInst(const string& FairnessName,
                                                          const vector<ExpT>& InstParams) const
        {
            return Fairnesses->GetFairnessObj(FairnessName, InstParams);
        }

        const LTSFairSetRef& 
        EFSMBase::GetFairnessSet(const string& FairnessName) const
        {
            return Fairnesses->GetFairnessSet(FairnessName);
        }

        const LTSPFGRef&
        EFSMBase::GetAllFairnessSets() const
        {
            return Fairnesses;
        }

        void EFSMBase::FreezeStates() 
        {
            AutomatonBase::FreezeStates();
            // Create a variable called "state"
            SymTab.Bind("state", new VarDecl("state", StateType));
        }

        void EFSMBase::FreezeVars()
        {
            AssertStatesFrozen();
            if (VarsFrozen) {
                return;
            }

            VarsFrozen = true;
            auto Mgr = TheLTS->GetMgr();
            // Create the variable type for this efsm
            auto const& DeclMap = SymTab.Bot()->GetDeclMap();
            vector<pair<string, ExprTypeRef>> RecordMembers;
            for (auto const& Decl : DeclMap) {
                if (Decl.second->Is<VarDecl>()) {
                    auto DeclAsVarDecl = Decl.second->SAs<VarDecl>();
                    RecordMembers.push_back(make_pair(DeclAsVarDecl->GetDeclName(),
                                                      DeclAsVarDecl->GetType()));
                }
            }

            auto ArrayType = Mgr->MakeType<ExprRecordType>(Name, RecordMembers);
            vector<ExpT> RevParams(Params.rbegin(), Params.rend());
            for (auto const& Param : RevParams) {
                ArrayType = Mgr->MakeType<ExprArrayType>(Param->GetType(), ArrayType);
            }
            StateVarType = ArrayType;

            // Create the RebaseSubstMaps
            const u32 NumInsts = ParamInsts.size();
            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& ParamInst = ParamInsts[i];
                auto const& ParamSubst = ParamSubsts[i];
                auto PrefixExp = Mgr->MakeVar(Name, StateVarType);
                for (auto const& Param : Params) {
                    PrefixExp = Mgr->MakeExpr(LTSOps::OpIndex, PrefixExp, 
                                              Mgr->Substitute(ParamSubst, Param));
                }

                auto FAType = Mgr->MakeType<ExprFieldAccessType>();
            
                // Create a substitution for each var
                for (auto const& Decl : DeclMap) {
                    if (Decl.second->Is<VarDecl>()) {
                        auto From = Mgr->MakeVar(Decl.first, Decl.second->GetType());
                        auto FieldVar = Mgr->MakeVar(Decl.first, FAType);
                        auto To = Mgr->MakeExpr(LTSOps::OpField, PrefixExp, FieldVar);
                        RebaseSubstMaps[ParamInst][From] = To;
                    }
                }
            }

            // Make the error and final conditions on states
            auto ErrorConditionOne = Mgr->MakeFalse();
            auto FinalConditionOne = Mgr->MakeFalse();
            
            for (auto const& StateDesc : States) {
                auto const& Desc = StateDesc.second;
                if (Desc.IsError()) {
                    auto ValExp = Mgr->MakeVal(Desc.GetName(), StateType);
                    auto ErrPred = Mgr->MakeExpr(LTSOps::OpEQ, Mgr->MakeVar("state", StateType),
                                                 ValExp);
                    ErrorConditionOne = Mgr->MakeExpr(LTSOps::OpOR, ErrorConditionOne, ErrPred);
                }
                if (Desc.IsFinal()) {
                    auto ValExp = Mgr->MakeVal(Desc.GetName(), StateType);
                    auto ErrPred = Mgr->MakeExpr(LTSOps::OpEQ, Mgr->MakeVar("state", StateType),
                                                 ValExp);
                    FinalConditionOne = Mgr->MakeExpr(LTSOps::OpOR, FinalConditionOne, ErrPred);
                }
            }

            vector<ExpT> ErrorConditions;
            vector<ExpT> FinalConditions;
            for (auto const& ParamInst : ParamInsts) {
                ErrorConditions.push_back(Mgr->Substitute(RebaseSubstMaps[ParamInst], 
                                                          ErrorConditionOne));
                FinalConditions.push_back(Mgr->Substitute(RebaseSubstMaps[ParamInst],
                                                          FinalConditionOne));
            }

            ErrorCondition = MakeDisjunction(ErrorConditions, Mgr);

            if (FinalConditions.size() == 0) {
                FinalCondition = Mgr->MakeFalse();
            } else {
                FinalCondition = MakeConjunction(FinalConditions, Mgr);
            }
        }

        void EFSMBase::Freeze()
        {
            AssertStatesFrozen();
            AssertVarsFrozen();
            if (EFSMFrozen) {
                return;
            }
            
            EFSMFrozen = true;
            return;
        }

        SymmMsgDeclRef EFSMBase::AddInputMsg(const ExprTypeRef& MsgType,
                                             const vector<ExpT>& Params)
        {
            return AddMsg(MsgType, Params, true);
        }

        SymmMsgDeclRef EFSMBase::AddInputMsgs(const vector<ExpT>& NewParams, 
                                              const ExpT& Constraint, 
                                              const ExprTypeRef& MessageType,
                                              const vector<ExpT>& MessageParams)
        {
            return AddMsgs(NewParams, Constraint, MessageType, MessageParams, true);
        }

        SymmMsgDeclRef EFSMBase::AddOutputMsg(const ExprTypeRef& MsgType,
                                              const vector<ExpT>& Params)
        {
            return AddMsg(MsgType, Params, false);
        }

        SymmMsgDeclRef EFSMBase::AddOutputMsgs(const vector<ExpT>& NewParams, 
                                               const ExpT& Constraint, 
                                               const ExprTypeRef& MessageType,
                                               const vector<ExpT>& MessageParams)
        {
            return AddMsgs(NewParams, Constraint, MessageType, MessageParams, false);
        }

        void EFSMBase::AddVariable(const string& VarName, const ExprTypeRef& VarType)
        {
            AssertStatesFrozen();
            AssertVarsNotFrozen();
            AssertEFSMNotFrozen();

            if (SymTab.Lookup(VarName) != DeclRef::NullPtr) {
                throw ESMCError((string)"Something named \"" + VarName + "\" already declared " + 
                                "in EFSM \"" + Name + "\"");
            }
            SymTab.Bind(VarName, new VarDecl(VarName, VarType));
        }

        void EFSMBase::AddInputTransForInstance(u32 InstanceID, 
                                                const MgrT::SubstMapT& SubstMap,
                                                const string& InitState, 
                                                const ExpT& Guard, 
                                                const vector<LTSAssignRef>& Updates, 
                                                const string& MessageName, 
                                                const ExprTypeRef& MessageType,
                                                const ExprTypeRef& ActMType,
                                                const LTSSymbTransRef& SymbTrans)
        {
            AssertStatesFrozen();
            AssertVarsFrozen();
            AssertEFSMNotFrozen();

            auto Mgr = TheLTS->GetMgr();
            CheckState(InitState);

            auto const& IS = States[InitState];

            auto const& ParamInst = ParamInsts[InstanceID];
            
            auto LocalUpdates = Updates;
            auto&& InstUpdates = InstantiateUpdates(SubstMap, LocalUpdates, MessageName,
                                                    MessageType, ActMType);
            auto&& RebasedUpdates = RebaseUpdates(ParamInst, InstUpdates);
            auto&& MsgTransformedUpdates = MsgTransformUpdates(RebasedUpdates,
                                                               MessageName, 
                                                               ActMType);
            auto&& SimpUpdates = SimplifyUpdates(MsgTransformedUpdates);
            SimpUpdates = ExpandUpdates(SimpUpdates);
            
            auto LocalGuard = 
                Mgr->MakeExpr(LTSOps::OpAND,
                              Mgr->MakeExpr(LTSOps::OpEQ,
                                            Mgr->MakeVar("state", StateType),
                                            Mgr->MakeVal(InitState, StateType)),
                              Guard);
            
            auto SubstGuard = Mgr->Substitute(SubstMap, LocalGuard);
            auto RebasedGuard = Mgr->Substitute(RebaseSubstMaps[ParamInst], 
                                                SubstGuard);
            auto MsgTransformedGuard = 
                Mgr->ApplyTransform<Detail::MsgTransformer>(RebasedGuard,
                                                            MessageName, MessageType,
                                                            TheLTS->GetUnifiedMType());

            // Finally, unroll quantifiers 
            auto ElimGuard = Mgr->UnrollQuantifiers(MsgTransformedGuard);
            auto SimpGuard = Mgr->Simplify(ElimGuard);

            auto CurTransition = new LTSTransitionInput(this, ParamInst, IS, SimpGuard,
                                                        SimpUpdates, MessageName, 
                                                        ActMType, SymbTrans);
            Transitions[ParamInst].push_back(CurTransition);
        }

        void EFSMBase::AddInputTransition(const string& InitState, const string& FinalState, 
                                          const ExpT& Guard, const vector<LTSAssignRef>& Updates, 
                                          const string& MessageName, const ExprTypeRef& MessageType, 
                                          const vector<ExpT>& MessageParams, bool Tentative)
        {
            auto Mgr = TheLTS->GetMgr();
            CheckState(FinalState);
            auto LocalUpdates = Updates;
            LocalUpdates.push_back(new LTSAssignSimple(Mgr->MakeVar("state", StateType),
                                                       Mgr->MakeVal(FinalState, StateType)));
            AddInputTransition(InitState, Guard, LocalUpdates, MessageName, 
                               MessageType, MessageParams, Tentative);
        }

        void EFSMBase::AddInputTransitions(const vector<ExpT>& TransParams, const ExpT& Constraint,
                                           const string& InitState, const string& FinalState,
                                           const ExpT& Guard, const vector<LTSAssignRef>& Updates,
                                           const string& MessageName, const ExprTypeRef& MessageType,
                                           const vector<ExpT>& MessageParams, bool Tentative)
        {
            auto Mgr = TheLTS->GetMgr();
            CheckState(FinalState);
            auto LocalUpdates = Updates;
            LocalUpdates.push_back(new LTSAssignSimple(Mgr->MakeVar("state", StateType),
                                                       Mgr->MakeVal(FinalState, StateType)));
            AddInputTransitions(TransParams, Constraint, InitState, Guard, 
                                LocalUpdates, MessageName, MessageType, 
                                MessageParams, Tentative);
        }

        void EFSMBase::AddInputTransition(const string& InitState, 
                                          const ExpT& Guard, 
                                          const vector<LTSAssignRef>& Updates, 
                                          const string& MessageName, 
                                          const ExprTypeRef& MessageType, 
                                          const vector<ExpT>& MessageParams,
                                          bool Tentative)
        {
            AssertStatesFrozen();
            AssertVarsFrozen();
            AssertEFSMNotFrozen();

            auto Mgr = TheLTS->GetMgr();

            CheckExpr(Guard, SymTab, Mgr);
            CheckParams(MessageParams, SymTab);
 
            ExprTypeRef ActMsgType = nullptr;
            if (MessageType->Is<ExprParametricType>()) {
                ActMsgType = MessageType->SAs<ExprParametricType>()->GetBaseType();
            }
            else if (MessageType->Is<ExprRecordType>()) {
                ActMsgType = MessageType;
            } else {
                throw ESMCError((string)"Message type \"" + MessageType->ToString() + 
                                "\" is not a record or parametric type");
            }

            SymTab.Push();
            if (SymTab.Lookup(MessageName) != DeclRef::NullPtr) {
                throw ESMCError((string)"Message Name \"" + MessageName + "\" shadows " + 
                                "earlier declaration in machine \"" + Name + "\"");
            }

            SymTab.Bind(MessageName, new InMsgDecl(MessageName, ActMsgType));
            CheckUpdates(Updates, SymTab, Mgr, true, MessageName);
            SymTab.Pop();

            auto SymbTrans = new LTSSymbInputTransition({}, this->Params, this->Constraint,
                                                        this, States[InitState],
                                                        Guard, Updates, MessageName,
                                                        MessageType, MessageParams, Tentative);
            SymbolicTransitions.push_back(SymbTrans);
            
            const u32 NumInsts = ParamInsts.size();
            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];
                auto ActMType = InstantiateMessageType(MessageParams, SubstMap, MessageType);
                CheckMsgType(ActMType);
                AssertInput(ParamInsts[i], ActMType);

                AddInputTransForInstance(i, SubstMap, InitState,
                                         Guard, Updates, MessageName, MessageType,
                                         ActMType, SymbTrans);
            }
        }

        void EFSMBase::AddInputTransitions(const vector<ExpT>& TransParams, 
                                           const ExpT& Constraint, 
                                           const string& InitState, 
                                           const ExpT& Guard, 
                                           const vector<LTSAssignRef>& Updates, 
                                           const string& MessageName, 
                                           const ExprTypeRef& MessageType, 
                                           const vector<ExpT>& MessageParams,
                                           bool Tentative)
        {
            AssertStatesFrozen();
            AssertVarsFrozen();
            AssertEFSMNotFrozen();

            auto Mgr = TheLTS->GetMgr();

            SymTab.Push();

            CheckParams(TransParams, Constraint, SymTab, Mgr, true);
            CheckParams(MessageParams, SymTab);
            CheckExpr(Guard, SymTab, Mgr);
            
            ExprTypeRef ActMsgType = nullptr;
            if (MessageType->Is<ExprParametricType>()) {
                ActMsgType = MessageType->SAs<ExprParametricType>()->GetBaseType();
            }
            else if (MessageType->Is<ExprRecordType>()) {
                ActMsgType = MessageType;
            } else {
                throw ESMCError((string)"Message type \"" + MessageType->ToString() + 
                                "\" is not a record or parametric type");
            }
            SymTab.Push();
            if (SymTab.Lookup(MessageName) != DeclRef::NullPtr) {
                throw ESMCError((string)"Message Name \"" + MessageName + "\" shadows " + 
                                "earlier declaration in machine \"" + Name + "\"");
            }
            SymTab.Bind(MessageName, new InMsgDecl(MessageName, ActMsgType));
            CheckUpdates(Updates, SymTab, Mgr, true, MessageName);
            SymTab.Pop();
            SymTab.Pop();

            vector<ExpT> CombinedParams = Params;
            CombinedParams.insert(CombinedParams.end(), TransParams.begin(),
                                  TransParams.end());
            auto CombinedConstraint = Mgr->MakeExpr(LTSOps::OpAND, this->Constraint,
                                                    Constraint);
            auto SymbTrans = new LTSSymbInputTransition(TransParams,
                                                        CombinedParams,
                                                        CombinedConstraint,
                                                        this, States[InitState],
                                                        Guard, Updates,
                                                        MessageName, MessageType,
                                                        MessageParams, Tentative);
            SymbolicTransitions.push_back(SymbTrans);

            const u32 NumInsts = ParamInsts.size();
            const u32 NumTransParams = TransParams.size();
            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];
                auto const& SubstConstraint = Mgr->Substitute(SubstMap, Constraint);

                auto&& TransParamInsts = InstantiateParams(TransParams, SubstConstraint, Mgr);

                for (auto const& TransParamInst : TransParamInsts) {
                    MgrT::SubstMapT LocalSubstMap = SubstMap;
                    for (u32 j = 0; j < NumTransParams; ++j) {
                        LocalSubstMap[TransParams[j]] = TransParamInst[j];
                    }

                    auto ActMType = 
                        InstantiateMessageType(MessageParams, LocalSubstMap, MessageType);
                    CheckMsgType(ActMType);
                    AssertInput(ParamInsts[i], ActMType);

                    AddInputTransForInstance(i, LocalSubstMap,
                                             InitState, Guard, Updates,
                                             MessageName, MessageType, ActMType,
                                             SymbTrans);
                }
            }
        }

        void EFSMBase::AddOutputTransForInstance(u32 InstanceID, 
                                                 const MgrT::SubstMapT& SubstMap,
                                                 const string& InitState,
                                                 const ExpT& Guard,
                                                 const vector<LTSAssignRef>& Updates,
                                                 const string& MessageName,
                                                 const ExprTypeRef& MessageType,
                                                 const ExprTypeRef& ActMType,
                                                 const set<string>& AddToFairnessSets,
                                                 const LTSSymbTransRef& SymbTrans)
        {
            AssertStatesFrozen();
            AssertVarsFrozen();
            AssertEFSMNotFrozen();

            auto Mgr = TheLTS->GetMgr();
            CheckState(InitState);
            CheckFairnessSets(AddToFairnessSets);

            auto const& IS = States[InitState];
            auto const& ParamInst = ParamInsts[InstanceID];

            auto UMType = TheLTS->GetUnifiedMType();
            auto UMTypeAsUnion = UMType->As<Exprs::ExprUnionType>();
            auto const& TypeIDFieldName = UMTypeAsUnion->GetTypeIDFieldName();
            auto const& TypeIDFieldType = UMTypeAsUnion->GetTypeIDFieldType();
            auto FieldVar = Mgr->MakeVar(TypeIDFieldName, Mgr->MakeType<ExprFieldAccessType>());
            auto FieldExp = Mgr->MakeExpr(LTSOps::OpField, 
                                          Mgr->MakeVar(MessageName, UMType), FieldVar);
            auto TypeIDExp = 
                Mgr->MakeVal(to_string(UMTypeAsUnion->GetTypeIDForMemberType(ActMType)),
                             TypeIDFieldType);

            
            auto LocalUpdates = Updates;
            LocalUpdates.push_back(new LTSAssignSimple(FieldExp, TypeIDExp));
                                                           
            auto&& InstUpdates = InstantiateUpdates(SubstMap, LocalUpdates,
                                                    MessageName, MessageType, ActMType);
            auto&& RebasedUpdates = RebaseUpdates(ParamInst, InstUpdates);
            auto&& MsgTransformedUpdates = MsgTransformUpdates(RebasedUpdates, 
                                                               MessageName, 
                                                               ActMType);
            auto&& SimpUpdates = SimplifyUpdates(MsgTransformedUpdates);
            SimpUpdates = ExpandUpdates(SimpUpdates);

            auto LocalGuard = Mgr->MakeExpr(LTSOps::OpAND, Guard,
                                            Mgr->MakeExpr(LTSOps::OpEQ, 
                                                          Mgr->MakeVar("state", StateType),
                                                          Mgr->MakeVal(InitState, StateType)));

            auto SubstGuard = Mgr->Substitute(SubstMap, LocalGuard);
            auto RebasedGuard = Mgr->Substitute(RebaseSubstMaps[ParamInst], 
                                                SubstGuard);
            auto MsgTransformedGuard = 
                Mgr->ApplyTransform<Detail::MsgTransformer>(RebasedGuard, MessageName, 
                                                            MessageType, 
                                                            TheLTS->GetUnifiedMType());
            auto ElimGuard = Mgr->UnrollQuantifiers(MsgTransformedGuard);
            auto SimpGuard = Mgr->Simplify(ElimGuard);

            // Add to appropriate fairness set if process fairness is set
            auto LocalFairnessSets = AddToFairnessSets;
            if (Fairness != LTSFairnessType::None) {
                LocalFairnessSets.insert("ProcessFairness");
            }

            auto CurTransition = new LTSTransitionOutput(this, ParamInst, IS, SimpGuard,
                                                         SimpUpdates, MessageName,
                                                         ActMType, LocalFairnessSets,
                                                         SymbTrans);


            Transitions[ParamInst].push_back(CurTransition);
        }

        void EFSMBase::AddOutputTransition(const string& InitState, 
                                           const string& FinalState, 
                                           const ExpT& Guard, 
                                           const vector<LTSAssignRef>& Updates, 
                                           const string& MessageName, 
                                           const ExprTypeRef& MessageType, 
                                           const vector<ExpT>& MessageParams, 
                                           const set<string> &AddToFairnessSets,
                                           bool Tentative)
        {
            auto Mgr = TheLTS->GetMgr();
            CheckState(FinalState);
            auto LocalUpdates = Updates;
            LocalUpdates.push_back(new LTSAssignSimple(Mgr->MakeVar("state", StateType),
                                                       Mgr->MakeVal(FinalState, StateType)));
            AddOutputTransition(InitState, Guard, LocalUpdates, MessageName,
                                MessageType, MessageParams, AddToFairnessSets,
                                Tentative);
        }

        void EFSMBase::AddOutputTransition(const string& InitState, 
                                           const ExpT& Guard, 
                                           const vector<LTSAssignRef>& Updates, 
                                           const string& MessageName, 
                                           const ExprTypeRef& MessageType, 
                                           const vector<ExpT>& MessageParams,
                                           const set<string>& AddToFairnessSets,
                                           bool Tentative)
        {
            AssertStatesFrozen();
            AssertVarsFrozen();
            AssertEFSMNotFrozen();

            auto Mgr = TheLTS->GetMgr();

            CheckFairnessSets(AddToFairnessSets);
            CheckExpr(Guard, SymTab, Mgr);
            CheckParams(MessageParams, SymTab);
            
            ExprTypeRef ActMsgType = nullptr;
            if (MessageType->Is<ExprParametricType>()) {
                ActMsgType = MessageType->SAs<ExprParametricType>()->GetBaseType();
            } else if (MessageType->Is<ExprRecordType>()) {
                ActMsgType = MessageType;
            } else {
                throw ESMCError((string)"Message type \"" + MessageType->ToString() + 
                                "\" is not a record or parametric type");
            }
            
            SymTab.Push();
            if (SymTab.Lookup(MessageName) != DeclRef::NullPtr) {
                throw ESMCError((string)"Message Name \"" + MessageName + "\" shadows " + 
                                "earlier declaration in machine \"" + Name + "\"");
            }

            SymTab.Bind(MessageName, new OutMsgDecl(MessageName, ActMsgType));
            CheckUpdates(Updates, SymTab, Mgr, false, MessageName);
            SymTab.Pop();

            const u32 NumInsts = ParamInsts.size();

            auto SymbTrans = new LTSSymbOutputTransition({}, this->Params, this->Constraint,
                                                         this, States[InitState],
                                                         Guard, Updates, MessageName,
                                                         MessageType, MessageParams,
                                                         Tentative);
            SymbolicTransitions.push_back(SymbTrans);

            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];

                auto ActMType = InstantiateMessageType(MessageParams, SubstMap, MessageType);
                CheckMsgType(ActMType);
                AssertOutput(ParamInsts[i], ActMType);

                AddOutputTransForInstance(i, SubstMap, InitState,
                                          Guard, Updates, MessageName, 
                                          MessageType, ActMType, 
                                          AddToFairnessSets, SymbTrans);
            }
        }


        void EFSMBase::AddOutputTransitions(const vector<ExpT>& TransParams, 
                                            const ExpT& Constraint, 
                                            const string& InitState, 
                                            const string& FinalState, 
                                            const ExpT& Guard, 
                                            const vector<LTSAssignRef>& Updates, 
                                            const string& MessageName, 
                                            const ExprTypeRef& MessageType, 
                                            const vector<ExpT>& MessageParams,
                                            LTSFairnessType FairnessKind,
                                            SplatFairnessType SplatFairness,
                                            const string& SplatFairnessName,
                                            bool Tentative)
        {
            auto Mgr = TheLTS->GetMgr();
            CheckState(FinalState);
            auto LocalUpdates = Updates;
            LocalUpdates.push_back(new LTSAssignSimple(Mgr->MakeVar("state", StateType),
                                                       Mgr->MakeVal(FinalState, StateType)));
            AddOutputTransitions(TransParams, Constraint, InitState,
                                 Guard, LocalUpdates, MessageName, MessageType, 
                                 MessageParams, FairnessKind, SplatFairness, 
                                 SplatFairnessName, Tentative);
        }

        void EFSMBase::AddOutputTransitions(const vector<ExpT>& TransParams, 
                                            const ExpT& Constraint, 
                                            const string& InitState, 
                                            const ExpT& Guard, 
                                            const vector<LTSAssignRef>& Updates, 
                                            const string& MessageName, 
                                            const ExprTypeRef& MessageType, 
                                            const vector<ExpT>& MessageParams,
                                            LTSFairnessType FairnessKind,
                                            SplatFairnessType SplatFairness,
                                            const string& SplatFairnessName,
                                            bool Tentative)
        {
            AssertStatesFrozen();
            AssertVarsFrozen();
            AssertEFSMNotFrozen();

            FairSetFairnessType SetFairnessType = FairSetFairnessType::Weak;
            string SplatPrefix;
            if (SplatFairness != SplatFairnessType::None) {
                if (FairnessKind == LTSFairnessType::None) {
                    throw ESMCError((string)"Cannot create splat fairness with FairnessKind " + 
                                    "specified as None");
                }
                SetFairnessType = FairnessKind == LTSFairnessType::Weak ? 
                    FairSetFairnessType::Weak : FairSetFairnessType::Strong;

                if (SplatFairnessName == "") {
                    SplatPrefix = ((string)"SplatFairness_" + 
                                   to_string(FairnessUIDGenerator.GetUID()));
                } else {
                    SplatPrefix = SplatFairnessName;
                }
            }

            auto Mgr = TheLTS->GetMgr();

            SymTab.Push();

            CheckParams(TransParams, Constraint, SymTab, Mgr, true);
            CheckParams(MessageParams, SymTab);
            CheckExpr(Guard, SymTab, Mgr);

            ExprTypeRef ActMsgType = nullptr;
            if (MessageType->Is<ExprParametricType>()) {
                ActMsgType = MessageType->SAs<ExprParametricType>()->GetBaseType();
            }
            else if (MessageType->Is<ExprRecordType>()) {
                ActMsgType = MessageType;
            } else {
                throw ESMCError((string)"Message type \"" + MessageType->ToString() + 
                                "\" is not a record or parametric type");
            }
            SymTab.Push();
            if (SymTab.Lookup(MessageName) != DeclRef::NullPtr) {
                throw ESMCError((string)"Message Name \"" + MessageName + "\" shadows " + 
                                "earlier declaration in machine \"" + Name + "\"");
            }
            
            SymTab.Bind(MessageName, new OutMsgDecl(MessageName, ActMsgType));
            CheckUpdates(Updates, SymTab, Mgr, false, MessageName);
            SymTab.Pop();
            SymTab.Pop();

            const u32 NumInsts = ParamInsts.size();
            const u32 NumTransParams = TransParams.size();

            if (SplatFairness == SplatFairnessType::Group) {
                Fairnesses->AddFairnessSet(SplatPrefix, SetFairnessType);
                UserToInternalFairness[SplatPrefix].insert(SplatPrefix);
            }

            vector<ExpT> CombinedParams = Params;
            CombinedParams.insert(CombinedParams.end(), TransParams.begin(),
                                  TransParams.end());
            auto CombinedConstraint = Mgr->MakeExpr(LTSOps::OpAND, this->Constraint,
                                                    Constraint);
            auto SymbTrans = new LTSSymbOutputTransition(TransParams,
                                                         CombinedParams,
                                                         CombinedConstraint,
                                                         this, States[InitState],
                                                         Guard, Updates,
                                                         MessageName, MessageType,
                                                         MessageParams, Tentative);
            SymbolicTransitions.push_back(SymbTrans);

            bool FirstInstance = true;

            for (u32 i = 0; i < NumInsts; ++i) {

                auto const& SubstMap = ParamSubsts[i];
                auto SubstConstraint = Mgr->Substitute(SubstMap, Constraint);
                
                auto&& TransParamInsts = InstantiateParams(TransParams, SubstConstraint, Mgr);
                UIDGenerator LocalFairnessUIDGenerator;

                // Since we're being symmetric, the number of instances
                // must be the same for each parameter
                for (auto const& TransParamInst : TransParamInsts) {

                    MgrT::SubstMapT LocalSubstMap = SubstMap;
                    for (u32 j = 0; j < NumTransParams; ++j) {
                        LocalSubstMap[TransParams[j]] = TransParamInst[j];
                    }

                    auto ActMType = 
                        InstantiateMessageType(MessageParams, LocalSubstMap, MessageType);

                    CheckMsgType(ActMType);
                    AssertOutput(ParamInsts[i], ActMType);

                    set<string> LocalFairnessSets;
                    string SplatFairnessSetName;
                    
                    if (SplatFairness == SplatFairnessType::Group) {
                        SplatFairnessSetName = SplatPrefix;
                        LocalFairnessSets.insert(SplatFairnessSetName);
                    }
                    if (SplatFairness == SplatFairnessType::Individual) {
                        SplatFairnessSetName = SplatPrefix + "_" + 
                            to_string(LocalFairnessUIDGenerator.GetUID());
                        LocalFairnessSets.insert(SplatFairnessSetName);
                        
                        if (FirstInstance) {
                            Fairnesses->AddFairnessSet(SplatFairnessSetName, SetFairnessType);
                            UserToInternalFairness[SplatPrefix].insert(SplatFairnessSetName);
                        }
                    }
                    
                    AddOutputTransForInstance(i, LocalSubstMap, InitState,
                                              Guard, Updates, MessageName, 
                                              MessageType, ActMType, 
                                              LocalFairnessSets, SymbTrans);
                }

                FirstInstance = false;
            }
        }

        void EFSMBase::AddInternalTransForInstance(u32 InstanceID,
                                                   const MgrT::SubstMapT& SubstMap,
                                                   const string& InitState,
                                                   const ExpT& Guard,
                                                   const vector<LTSAssignRef>& Updates,
                                                   const set<string>& AddToFairnessSets,
                                                   const LTSSymbTransRef& SymbTrans)
        {
            AssertStatesFrozen();
            AssertVarsFrozen();
            AssertEFSMNotFrozen();

            auto Mgr = TheLTS->GetMgr();

            CheckState(InitState);
            CheckFairnessSets(AddToFairnessSets);

            auto const& IS = States[InitState];
            auto const& ParamInst = ParamInsts[InstanceID];
            
            auto LocalUpdates = Updates;
            auto&& InstUpdates = InstantiateUpdates(SubstMap, LocalUpdates,
                                                    "", ExprTypeRef::NullPtr, ExprTypeRef::NullPtr);
            auto&& RebasedUpdates = RebaseUpdates(ParamInst, InstUpdates);
            auto&& SimpUpdates = SimplifyUpdates(RebasedUpdates);
            SimpUpdates = ExpandUpdates(SimpUpdates);
             
            auto LocalGuard = 
                Mgr->MakeExpr(LTSOps::OpAND, 
                              Mgr->MakeExpr(LTSOps::OpEQ,
                                            Mgr->MakeVar("state", StateType),
                                            Mgr->MakeVal(InitState, StateType)),
                              Guard);

            auto SubstGuard = Mgr->Substitute(SubstMap, LocalGuard);
            auto RebasedGuard = Mgr->Substitute(RebaseSubstMaps[ParamInst], SubstGuard);
            auto ElimGuard = Mgr->UnrollQuantifiers(RebasedGuard);
            auto SimpGuard = Mgr->Simplify(ElimGuard);

            auto LocalFairnessSets = AddToFairnessSets;
            if (Fairness != LTSFairnessType::None) {
                LocalFairnessSets.insert("ProcessFairness");
            }

            auto CurTransition = new LTSTransitionInternal(this, ParamInst, IS, SimpGuard,
                                                           SimpUpdates, LocalFairnessSets,
                                                           SymbTrans);

            Transitions[ParamInst].push_back(CurTransition);
        }

        void EFSMBase::AddInternalTransition(const string& InitState, 
                                             const string& FinalState, 
                                             const ExpT& Guard, 
                                             const vector<LTSAssignRef>& Updates,
                                             const set<string>& AddToFairnessSets,
                                             bool Tentative)
        {
            auto Mgr = TheLTS->GetMgr();
            CheckState(FinalState);
            auto LocalUpdates = Updates;
            LocalUpdates.push_back(new LTSAssignSimple(Mgr->MakeVar("state", StateType),
                                                       Mgr->MakeVal(FinalState, StateType)));
            AddInternalTransition(InitState, Guard, LocalUpdates, 
                                  AddToFairnessSets, Tentative);
        }

        void EFSMBase::AddInternalTransition(const string& InitState, 
                                             const ExpT& Guard, 
                                             const vector<LTSAssignRef> &Updates,
                                             const set<string>& AddToFairnessSets,
                                             bool Tentative)
        {
            AssertStatesFrozen();
            AssertVarsFrozen();
            AssertEFSMNotFrozen();

            auto Mgr = TheLTS->GetMgr();

            CheckFairnessSets(AddToFairnessSets);
            CheckExpr(Guard, SymTab, Mgr);

            CheckUpdates(Updates, SymTab, Mgr, false, "");

            auto SymbTrans = new LTSSymbInternalTransition({}, this->Params, this->Constraint,
                                                           this, States[InitState],
                                                           Guard, Updates, Tentative);
            SymbolicTransitions.push_back(SymbTrans);

            const u32 NumInsts = ParamInsts.size();

            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];
                AddInternalTransForInstance(i, SubstMap, InitState, Guard, 
                                            Updates, AddToFairnessSets, SymbTrans);
            }
        }

        void EFSMBase::AddInternalTransitions(const vector<ExpT>& TransParams, 
                                              const ExpT& Constraint, 
                                              const string& InitState, 
                                              const string& FinalState, 
                                              const ExpT& Guard, 
                                              const vector<LTSAssignRef>& Updates, 
                                              LTSFairnessType FairnessKind, 
                                              SplatFairnessType SplatFairness,
                                              const string& SplatFairnessName,
                                              bool Tentative)
        {
            auto Mgr = TheLTS->GetMgr();
            CheckState(FinalState);
            auto LocalUpdates = Updates;
            LocalUpdates.push_back(new LTSAssignSimple(Mgr->MakeVar("state", StateType),
                                                       Mgr->MakeVal(FinalState, StateType)));
            AddInternalTransitions(TransParams, Constraint, InitState, 
                                   Guard, LocalUpdates, FairnessKind, SplatFairness, 
                                   SplatFairnessName, Tentative);
        }

        void EFSMBase::AddInternalTransitions(const vector<ExpT>& TransParams, 
                                              const ExpT& Constraint, 
                                              const string& InitState, 
                                              const ExpT& Guard, 
                                              const vector<LTSAssignRef>& Updates, 
                                              LTSFairnessType FairnessKind, 
                                              SplatFairnessType SplatFairness,
                                              const string& SplatFairnessName,
                                              bool Tentative)
        {
            AssertStatesFrozen();
            AssertVarsFrozen();
            AssertEFSMNotFrozen();

            FairSetFairnessType SetFairnessType = FairSetFairnessType::Weak;
            string SplatPrefix;
            if (SplatFairness != SplatFairnessType::None) {
                if (FairnessKind == LTSFairnessType::None) {
                    throw ESMCError((string)"Cannot create splat fairness with FairnessKind " + 
                                    "specified as None");
                }
                SetFairnessType = FairnessKind == LTSFairnessType::Weak ? 
                    FairSetFairnessType::Weak : FairSetFairnessType::Strong;

                if (SplatFairnessName == "") {
                    SplatPrefix = ((string)"SplatFairness_" + 
                                   to_string(FairnessUIDGenerator.GetUID()));
                } else {
                    SplatPrefix = SplatFairnessName;
                }
            }

            auto Mgr = TheLTS->GetMgr();

            SymTab.Push();

            CheckParams(TransParams, Constraint, SymTab, Mgr, true);
            CheckExpr(Guard, SymTab, Mgr);
            CheckUpdates(Updates, SymTab, Mgr, false, "");

            SymTab.Pop();

            vector<ExpT> CombinedParams = Params;
            CombinedParams.insert(CombinedParams.end(), TransParams.begin(),
                                  TransParams.end());
            auto CombinedConstraint = Mgr->MakeExpr(LTSOps::OpAND, this->Constraint,
                                                    Constraint);
            auto SymbTrans = new LTSSymbInternalTransition(TransParams,
                                                           CombinedParams,
                                                           CombinedConstraint,
                                                           this, States[InitState],
                                                           Guard, Updates, Tentative);
            SymbolicTransitions.push_back(SymbTrans);

            const u32 NumInsts = ParamInsts.size();
            const u32 NumTransParams = TransParams.size();

            if (SplatFairness == SplatFairnessType::Group) {
                Fairnesses->AddFairnessSet(SplatPrefix, SetFairnessType);
                UserToInternalFairness[SplatPrefix].insert(SplatPrefix);
            }

            bool FirstInstance = true;
            
            for (u32 i = 0; i < NumInsts; ++i) {

                auto const& SubstMap = ParamSubsts[i];
                auto SubstConstraint = Mgr->Substitute(SubstMap, Constraint);

                auto&& TransParamInsts = InstantiateParams(TransParams, SubstConstraint, Mgr);
                UIDGenerator LocalFairnessUIDGenerator;

                for (auto const& TransParamInst : TransParamInsts) {
                    MgrT::SubstMapT LocalSubstMap = SubstMap;

                    for (u32 j = 0; j < NumTransParams; ++j) {
                        LocalSubstMap[TransParams[j]] = TransParamInst[j];
                    }

                    set<string> LocalFairnessSets;
                    string SplatFairnessSetName;
                    
                    if (Fairness != LTSFairnessType::None) {
                        LocalFairnessSets.insert("ProcessFairness");
                    }
                    if (SplatFairness == SplatFairnessType::Group) {
                        SplatFairnessSetName = SplatPrefix;
                        LocalFairnessSets.insert(SplatFairnessSetName);
                    }
                    if (SplatFairness == SplatFairnessType::Individual) {
                        SplatFairnessSetName = SplatPrefix + "_" + 
                            to_string(LocalFairnessUIDGenerator.GetUID());
                        LocalFairnessSets.insert(SplatFairnessSetName);

                        if (FirstInstance) {
                            Fairnesses->AddFairnessSet(SplatFairnessSetName, SetFairnessType);
                            UserToInternalFairness[SplatPrefix].insert(SplatFairnessSetName);
                        }
                    }
                    
                    AddInternalTransForInstance(i, LocalSubstMap, InitState,
                                                Guard, Updates, LocalFairnessSets,
                                                SymbTrans);
                }

                FirstInstance = false;
            }
        }

        string EFSMBase::ToString() const
        {
            ostringstream sstr;

            for (auto const& ParamInst : ParamInsts) {
                sstr << "efsm " << Name;
                for (auto const& ParamVal : ParamInst) {
                    sstr << "[" << ParamVal->ToString() << "]";
                }
                sstr << " {" << endl;

                auto it1 = Inputs.find(ParamInst);
                if (it1 != Inputs.end()) {
                    sstr << "    inputs {" << endl;
                    for (auto const& MType : it1->second) {
                        sstr << "        " << MType->As<ExprRecordType>()->GetName() << endl;
                    }
                    sstr << "    }" << endl;
                }

                auto it2 = Outputs.find(ParamInst);
                if (it2 != Outputs.end()) {
                    sstr << "    outputs {" << endl;
                    for (auto const& MType : it2->second) {
                        sstr << "        " << MType->As<ExprRecordType>()->GetName() << endl;
                    }
                    sstr << "    }" << endl;
                }
                
                sstr << "    vars {" << endl;
                for (auto const& NameDecl : SymTab.Bot()->GetDeclMap()) {
                    if (NameDecl.second->Is<VarDecl>()) {
                        sstr << "        " << NameDecl.first << " : " 
                             << NameDecl.second->GetType()->ToString() << endl;
                    }
                }
                sstr << "    }" << endl;

                auto it3 = Transitions.find(ParamInst);
                if (it3 != Transitions.end()) {
                    sstr << "    transitions {" << endl;
                    for (auto const& Trans : it3->second) {
                        sstr << Trans->ToString(8) << endl;
                    }
                    sstr << "    }" << endl;
                }
                sstr << "}" << endl << endl;
            }

            sstr << "Symbolic Transitions:" << endl;
            for (auto const& SymbTrans : SymbolicTransitions) {
                sstr << SymbTrans->ToString() << endl << endl;
            }

            return sstr.str();
        }

        const MgrT::SubstMapT& EFSMBase::GetRebaseSubstMap(const vector<ExpT>& ParamInst) const
        {
            auto it = RebaseSubstMaps.find(ParamInst);
            if (it == RebaseSubstMaps.end()) {
                throw InternalError((string)"Requested rebase substitution map of non-existent " + 
                                    "instance of EFSM \"" + Name + "\"\nAt: " + __FILE__ + 
                                    ":" + to_string(__LINE__));
            }
            return it->second;
        }

        const vector<LTSSymbTransRef>& EFSMBase::GetSymbolicTransitions() const
        {
            return SymbolicTransitions;
        }

        vector<LTSSymbTransRef> 
        EFSMBase::GetSymbolicTransitions(const function<bool(const LTSSymbTransRef&)>& 
                                         MatchPred) const
        {
            vector<LTSSymbTransRef> Retval;
            for (auto const& SymbTrans : SymbolicTransitions) {
                if (MatchPred(SymbTrans)) {
                    Retval.push_back(SymbTrans);
                }
            }
            return Retval;
        }

        const vector<SymmMsgDeclRef>& EFSMBase::GetSymmetricMessages() const
        {
            return SymmetricMessages;
        }

        vector<SymmMsgDeclRef> 
        EFSMBase::GetSymmetricMessages(const function<bool(const SymmMsgDeclRef&)>& 
                                       MatchPred) const
        {
            vector<SymmMsgDeclRef> Retval;
            for (auto const& SymmMsgDecl : SymmetricMessages) {
                if (MatchPred(SymmMsgDecl)) {
                    Retval.push_back(SymmMsgDecl);
                }
            }
            return Retval;
        }
            

    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// LTSEFSMBase.cpp ends here



