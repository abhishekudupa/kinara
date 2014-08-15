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

        EFSMBase::EFSMBase(LabelledTS* TheLTS, const string& Name,
                           const vector<ExpT>& Params, const ExpT& Constraint,
                           LTSFairnessType Fairness)
            : AutomatonBase(TheLTS, Name, Params, Constraint),
              Fairness(Fairness), VarsFrozen(false), EFSMFrozen(false)
        {
            if (Fairness != LTSFairnessType::None) {
                auto FairsetFairness = Fairness == LTSFairnessType::Weak ? 
                    FairSetFairnessType::Weak : FairSetFairnessType::Strong;

                for (auto const& ParamInst : ParamInsts) {
                    Fairnesses["ProcessFairness"][ParamInst] = new LTSFairnessSet(this,
                                                                                  "ProcessFairness",
                                                                                  FairsetFairness);
                }
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

        vector<LTSTransRef> EFSMBase::GetTransitionsOnMsg(const ExprTypeRef& MsgType) const
        {
            if (!MsgType->Is<ExprRecordType>()) {
                throw ESMCError((string)"EFSMBase::GetTransitionsOnMsg() must be called " + 
                                "on actual message types, not parametric types");
            }

            vector<LTSTransRef> Retval;

            for (auto const& InstTrans : Transitions) {
                for (auto const& Trans : InstTrans.second) {
                    if (Trans->Is<LTSTransitionIOBase>()) {
                        auto AsIO = Trans->SAs<LTSTransitionIOBase>();
                        if (AsIO->GetMessageType() == MsgType) {
                            Retval.push_back(Trans);
                        }
                    }
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
                auto it = Fairnesses.find(FairnessSetName);
                if (it == Fairnesses.end()) {
                    throw ESMCError((string)"No fairness set called \"" + FairnessSetName + 
                                    "\" has been declared yet in EFSM \"" + Name + "\"");
                }
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

        void EFSMBase::AddMsg(const ExprTypeRef& MsgType, const vector<ExpT>& Params, bool IsInput)
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
        }

        void EFSMBase::AddMsgs(const vector<ExpT>& NewParams, const ExpT& Constraint, 
                               const ExprTypeRef& MsgType, const vector<ExpT>& Params, 
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
                        
                        Inputs[ParamInsts[i]].insert(ActMsgType);
                    } else {
                        if (InpSet.find(ActMsgType) != InpSet.end()) {
                            throw ESMCError((string)"Message type "  + ActMsgType->ToString()  + 
                                            " conflicts in input/output definition");
                        }
                        
                        Outputs[ParamInsts[i]].insert(ActMsgType);
                    }
                }
            }
        }

        vector<LTSAssignRef> EFSMBase::InstantiateUpdates(const MgrT::SubstMapT& ParamSubst,
                                                          const vector<LTSAssignRef>& Updates)
        {
            auto Mgr = TheLTS->GetMgr();
            auto const& SubstMap = ParamSubst;
            vector<LTSAssignRef> Retval;

            for (auto const& Update : Updates) {
                auto PUpdate = Update->As<LTSAssignParam>();
                if (PUpdate == nullptr) {
                    Retval.push_back(Update);
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
                        new LTSAssignSimple(Mgr->Substitute(SubstMap, PUpdate->GetLHS()),
                                            Mgr->Substitute(SubstMap, PUpdate->GetRHS()));
                    Retval.push_back(NewAsgn);
                }
            }
            return Retval;
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

            if (Fairnesses.find(Name) != Fairnesses.end()) {
                throw ESMCError((string)"Fairness Set \"" + Name + "\" has already " + 
                                "been declared in EFSM \"" + this->Name + "\"");
            }
            // Add to every instance
            Fairnesses[Name] = map<vector<ExpT>, LTSFairSetRef>();
            for (auto const& ParamInst : ParamInsts) {
                Fairnesses[Name][ParamInst] = new LTSFairnessSet(this, Name, Fairness);
            }
        }

        const LTSFairSetRef& EFSMBase::GetFairnessForInst(const string& FairnessName,
                                                          const vector<ExpT>& InstParams) const
        {
            auto it = Fairnesses.find(FairnessName);
            if (it == Fairnesses.end()) {
                throw ESMCError((string)"No fairness set called \"" + FairnessName + 
                                "\" has been declared yet in EFSM \"" + Name + "\"");
            }
            auto it2 = it->second.find(InstParams);
            if (it2 == it->second.end()) {
                throw ESMCError((string)"No fairness set called \"" + FairnessName + 
                                "\" available for the given instance parameters");
            }
            return it2->second;
        }

        const map<vector<ExpT>, LTSFairSetRef>& 
        EFSMBase::GetFairnessSet(const string& FairnessName) const
        {
            auto it = Fairnesses.find(FairnessName);
            if (it == Fairnesses.end()) {
                throw ESMCError((string)"No fairness set called \"" + FairnessName + 
                                "\" has been declared yet in EFSM \"" + Name + "\"");
            }
            return it->second;
        }

        const map<string, map<vector<ExpT>, LTSFairSetRef> >&
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
            for (auto const& Param : Params) {
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

            if (ErrorConditions.size() == 0) {
                ErrorCondition = Mgr->MakeFalse();
            } else if (ErrorConditions.size() == 1) {
                ErrorCondition = ErrorConditions[1];
            } else {
                ErrorCondition = Mgr->MakeExpr(LTSOps::OpOR, ErrorConditions);
            }

            if (FinalConditions.size() == 0) {
                FinalCondition = Mgr->MakeFalse();
            } else if (FinalConditions.size() == 1) {
                FinalCondition = FinalConditions[0];
            } else {
                FinalCondition = Mgr->MakeExpr(LTSOps::OpAND, FinalConditions);
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

        void EFSMBase::AddInputMsg(const ExprTypeRef& MsgType,
                                   const vector<ExpT>& Params)
        {
            AddMsg(MsgType, Params, true);
        }

        void EFSMBase::AddInputMsgs(const vector<ExpT>& NewParams, const ExpT& Constraint, 
                                    const ExprTypeRef& MessageType,
                                    const vector<ExpT>& MessageParams)
        {
            AddMsgs(NewParams, Constraint, MessageType, MessageParams, true);
        }

        void EFSMBase::AddOutputMsg(const ExprTypeRef& MsgType,
                                    const vector<ExpT>& Params)
        {
            AddMsg(MsgType, Params, false);
        }

        void EFSMBase::AddOutputMsgs(const vector<ExpT>& NewParams, const ExpT& Constraint, 
                                     const ExprTypeRef& MessageType,
                                     const vector<ExpT>& MessageParams)
        {
            AddMsgs(NewParams, Constraint, MessageType, MessageParams, false);
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
                                                const string& FinalState, 
                                                const ExpT& Guard, 
                                                const vector<LTSAssignRef>& Updates, 
                                                const string& MessageName, 
                                                const ExprTypeRef& MessageType,
                                                const ExprTypeRef& ActMType)
        {
            AssertStatesFrozen();
            AssertVarsFrozen();
            AssertEFSMNotFrozen();

            auto Mgr = TheLTS->GetMgr();
            CheckState(InitState);
            CheckState(FinalState);

            auto const& IS = States[InitState];
            auto const& FS = States[FinalState];

            auto const& ParamInst = ParamInsts[InstanceID];
            
            auto LocalUpdates = Updates;
            // Update state var
            LocalUpdates.push_back(new LTSAssignSimple(Mgr->MakeVar("state", StateType),
                                                       Mgr->MakeVal(FinalState, StateType)));
            auto&& InstUpdates = InstantiateUpdates(SubstMap, LocalUpdates);
            auto&& RebasedUpdates = RebaseUpdates(ParamInst, InstUpdates);
            auto&& MsgTransformedUpdates = MsgTransformUpdates(RebasedUpdates,
                                                               MessageName, 
                                                               MessageType);
            auto&& SimpUpdates = SimplifyUpdates(MsgTransformedUpdates);
            
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
            auto SimpGuard = Mgr->Simplify(MsgTransformedGuard);

            auto CurTransition = new LTSTransitionInput(this, IS, FS, SimpGuard,
                                                        SimpUpdates, MessageName, 
                                                        ActMType);
            Transitions[ParamInst].push_back(CurTransition);
        }

        void EFSMBase::AddInputTransition(const string& InitState, const string& FinalState, 
                                          const ExpT& Guard, const vector<LTSAssignRef>& Updates, 
                                          const string& MessageName, const ExprTypeRef& MessageType, 
                                          const vector<ExpT>& MessageParams)
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
            
            const u32 NumInsts = ParamInsts.size();
            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];
                auto ActMType = InstantiateMessageType(MessageParams, SubstMap, MessageType);
                CheckMsgType(ActMType);
                AssertInput(ParamInsts[i], ActMType);

                AddInputTransForInstance(i, SubstMap, InitState, FinalState,
                                         Guard, Updates, MessageName, MessageType,
                                         ActMType);
            }
        }

        void EFSMBase::AddInputTransitions(const vector<ExpT>& TransParams, const ExpT& Constraint,
                                           const string& InitState, const string& FinalState,
                                           const ExpT& Guard, const vector<LTSAssignRef>& Updates,
                                           const string& MessageName, const ExprTypeRef& MessageType,
                                           const vector<ExpT>& MessageParams)
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

                    AddInputTransForInstance(i, LocalSubstMap, InitState,
                                             FinalState, Guard, Updates,
                                             MessageName, MessageType, ActMType);
                }
            }
        }

        void EFSMBase::AddOutputTransForInstance(u32 InstanceID, 
                                                 const MgrT::SubstMapT& SubstMap,
                                                 const string& InitState,
                                                 const string& FinalState,
                                                 const ExpT& Guard,
                                                 const vector<LTSAssignRef>& Updates,
                                                 const string& MessageName,
                                                 const ExprTypeRef& MessageType,
                                                 const ExprTypeRef& ActMType,
                                                 const set<string>& AddToFairnessSets)
        {
            AssertStatesFrozen();
            AssertVarsFrozen();
            AssertEFSMNotFrozen();

            auto Mgr = TheLTS->GetMgr();
            CheckState(InitState);
            CheckState(FinalState);
            CheckFairnessSets(AddToFairnessSets);

            auto const& IS = States[InitState];
            auto const& FS = States[FinalState];
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
            // Add the next state update
            LocalUpdates.push_back(new LTSAssignSimple(Mgr->MakeVar("state", StateType),
                                                       Mgr->MakeVal(FinalState, StateType)));
                                                           
            auto&& InstUpdates = InstantiateUpdates(SubstMap, LocalUpdates);
            auto&& RebasedUpdates = RebaseUpdates(ParamInst, InstUpdates);
            auto&& MsgTransformedUpdates = MsgTransformUpdates(RebasedUpdates, 
                                                               MessageName, 
                                                               MessageType);
            auto&& SimpUpdates = SimplifyUpdates(MsgTransformedUpdates);

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
            auto SimpGuard = Mgr->Simplify(MsgTransformedGuard);

            // Add to appropriate fairness set if process fairness is set
            auto LocalFairnessSets = AddToFairnessSets;
            if (Fairness != LTSFairnessType::None) {
                LocalFairnessSets.insert("ProcessFairness");
            }

            auto CurTransition = new LTSTransitionOutput(this, IS, FS, SimpGuard,
                                                         SimpUpdates, MessageName,
                                                         ActMType, LocalFairnessSets);

            for (auto const& FairnessSet : LocalFairnessSets) {
                Fairnesses[FairnessSet][ParamInst]->AddTransition(CurTransition);
            }

            Transitions[ParamInst].push_back(CurTransition);
        }

        void EFSMBase::AddOutputTransition(const string& InitState, const string& FinalState, 
                                           const ExpT& Guard, const vector<LTSAssignRef> &Updates, 
                                           const string &MessageName, 
                                           const ExprTypeRef &MessageType, 
                                           const vector<ExpT> &MessageParams,
                                           const set<string>& AddToFairnessSets)
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

            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];

                auto ActMType = InstantiateMessageType(MessageParams, SubstMap, MessageType);
                CheckMsgType(ActMType);
                AssertOutput(ParamInsts[i], ActMType);

                AddOutputTransForInstance(i, SubstMap, InitState, FinalState,
                                          Guard, Updates, MessageName, 
                                          MessageType, ActMType, AddToFairnessSets);
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
                                            const string& SplatFairnessName)
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

            for (u32 i = 0; i < NumInsts; ++i) {

                if (SplatFairness == SplatFairnessType::Group) {
                    Fairnesses[SplatPrefix][ParamInsts[i]] = 
                        new LTSFairnessSet(this, SplatPrefix, SetFairnessType);
                    UserToInternalFairness[SplatPrefix].insert(SplatPrefix);
                }

                auto const& SubstMap = ParamSubsts[i];
                auto SubstConstraint = Mgr->Substitute(SubstMap, Constraint);
                
                auto&& TransParamInsts = InstantiateParams(TransParams, SubstConstraint, Mgr);
                UIDGenerator LocalFairnessUIDGenerator;

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
                        // This gets repeated as many times as there are 
                        // instances, but that's okay!
                        UserToInternalFairness[SplatPrefix].insert(SplatFairnessSetName);
                        LocalFairnessSets.insert(SplatFairnessSetName);
                        auto NewFairnessSet = new LTSFairnessSet(this, SplatFairnessSetName,
                                                                 SetFairnessType);
                        Fairnesses[SplatFairnessSetName][ParamInsts[i]] = NewFairnessSet;
                    }
                    
                    AddOutputTransForInstance(i, LocalSubstMap, InitState,
                                              FinalState, Guard, Updates, 
                                              MessageName, MessageType, ActMType,
                                              LocalFairnessSets);
                }
            }
        }

        void EFSMBase::AddInternalTransForInstance(u32 InstanceID,
                                                   const MgrT::SubstMapT& SubstMap,
                                                   const string& InitState,
                                                   const string& FinalState,
                                                   const ExpT& Guard,
                                                   const vector<LTSAssignRef>& Updates,
                                                   const set<string>& AddToFairnessSets)
        {
            AssertStatesFrozen();
            AssertVarsFrozen();
            AssertEFSMNotFrozen();

            auto Mgr = TheLTS->GetMgr();

            CheckState(InitState);
            CheckState(FinalState);
            CheckFairnessSets(AddToFairnessSets);

            auto const& IS = States[InitState];
            auto const& FS = States[FinalState];
            auto const& ParamInst = ParamInsts[InstanceID];
            
            auto LocalUpdates = Updates;
            LocalUpdates.push_back(new LTSAssignSimple(Mgr->MakeVar("state", StateType),
                                                       Mgr->MakeVal(FinalState, StateType)));
            auto&& InstUpdates = InstantiateUpdates(SubstMap, Updates);
            auto&& RebasedUpdates = RebaseUpdates(ParamInst, InstUpdates);
            auto&& SimpUpdates = SimplifyUpdates(RebasedUpdates);
             
            auto LocalGuard = 
                Mgr->MakeExpr(LTSOps::OpAND, 
                              Mgr->MakeExpr(LTSOps::OpEQ,
                                            Mgr->MakeVar("state", StateType),
                                            Mgr->MakeVal(InitState, StateType)),
                              Guard);

            auto SubstGuard = Mgr->Substitute(SubstMap, LocalGuard);
            auto RebasedGuard = Mgr->Substitute(RebaseSubstMaps[ParamInst], SubstGuard);
            auto SimpGuard = Mgr->Simplify(RebasedGuard);

            auto LocalFairnessSets = AddToFairnessSets;
            if (Fairness != LTSFairnessType::None) {
                LocalFairnessSets.insert("ProcessFairness");
            }

            auto CurTransition = new LTSTransitionInternal(this, IS, FS, SimpGuard,
                                                           SimpUpdates, LocalFairnessSets);

            for (auto const& FairnessSet : LocalFairnessSets) {
                Fairnesses[FairnessSet][ParamInst]->AddTransition(CurTransition);
            }

            Transitions[ParamInst].push_back(CurTransition);
        }

        void EFSMBase::AddInternalTransition(const string& InitState, const string& FinalState, 
                                             const ExpT& Guard, const vector<LTSAssignRef> &Updates,
                                             const set<string>& AddToFairnessSets)
        {
            AssertStatesFrozen();
            AssertVarsFrozen();
            AssertEFSMNotFrozen();

            auto Mgr = TheLTS->GetMgr();

            CheckFairnessSets(AddToFairnessSets);
            CheckExpr(Guard, SymTab, Mgr);

            CheckUpdates(Updates, SymTab, Mgr, false, "");

            const u32 NumInsts = ParamInsts.size();

            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];
                AddInternalTransForInstance(i, SubstMap, InitState, FinalState,
                                            Guard, Updates, AddToFairnessSets);
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
                                              const string& SplatFairnessName)
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

            const u32 NumInsts = ParamInsts.size();
            const u32 NumTransParams = TransParams.size();

            for (u32 i = 0; i < NumInsts; ++i) {

                if (SplatFairness == SplatFairnessType::Group) {
                    Fairnesses[SplatPrefix][ParamInsts[i]] = 
                        new LTSFairnessSet(this, SplatPrefix, SetFairnessType);
                    UserToInternalFairness[SplatPrefix].insert(SplatPrefix);
                }

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
                        UserToInternalFairness[SplatPrefix].insert(SplatFairnessSetName);
                        auto NewFairnessSet = new LTSFairnessSet(this, SplatFairnessSetName,
                                                                 SetFairnessType);
                        Fairnesses[SplatFairnessSetName][ParamInsts[i]] = NewFairnessSet;
                    }
                    
                    AddInternalTransForInstance(i, LocalSubstMap, InitState,
                                                FinalState, Guard, Updates,
                                                LocalFairnessSets);
                }
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
                    sstr << "        " << NameDecl.first << " : " 
                         << NameDecl.second->GetType()->ToString() << endl;
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

            return sstr.str();
        }

    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// LTSEFSMBase.cpp ends here
