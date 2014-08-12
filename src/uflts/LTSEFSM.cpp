// LTSEFSM.cpp --- 
// 
// Filename: LTSEFSM.cpp
// Author: Abhishek Udupa
// Created: Fri Aug  8 20:32:45 2014 (-0400)
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

#include "LTSEFSM.hpp"
#include "LTSUtils.hpp"
#include "LabelledTS.hpp"
#include "LTSState.hpp"
#include "LTSTransitions.hpp"
#include "LTSFairnessSet.hpp"

namespace ESMC {
    namespace LTS {

        namespace Detail {

            MsgTransformer::MsgTransformer(MgrT* Mgr, const string& MsgVarName,
                                           const ExprTypeRef& MsgRecType, 
                                           const ExprTypeRef& UnifiedMType)
                : VisitorBaseT("MessageTransformer"),
                  Mgr(Mgr), MsgVarName(MsgVarName), MsgRecType(MsgRecType),
                  UnifiedMType(UnifiedMType)
            {
                // Nothing here
            }

            MsgTransformer::~MsgTransformer()
            {
                // Nothing here
            }

            void MsgTransformer::VisitVarExpression(const VarExpT* Exp)
            {
                if (Exp->GetVarName() == MsgVarName &&
                    Exp->GetVarType() == MsgRecType) {
                    ExpStack.push_back(Mgr->MakeVar(Exp->GetVarName(), UnifiedMType));
                } else {
                    ExpStack.push_back(Exp);
                }
            }

            void MsgTransformer::VisitBoundVarExpression(const BoundVarExpT* Exp)
            {
                ExpStack.push_back(Exp);
            }

            void MsgTransformer::VisitConstExpression(const ConstExpT* Exp)
            {
                ExpStack.push_back(Exp);
            }

            void MsgTransformer::VisitOpExpression(const OpExpT* Exp)
            {
                VisitorBaseT::VisitOpExpression(Exp);

                auto const& OldChildren = Exp->GetChildren();
                const u32 NumChildren = OldChildren.size();
                vector<ExpT> NewChildren(NumChildren);
                for (u32 i = 0; i < NumChildren; ++i) {
                    NewChildren[NumChildren - i - 1] = ExpStack.back();
                    ExpStack.pop_back();
                }

                auto OpCode = Exp->GetOpCode();
                if (OpCode == LTSOps::OpField && 
                    OldChildren[0]->Is<Exprs::VarExpression>() &&
                    OldChildren[0]->As<Exprs::VarExpression>()->GetVarName() == MsgVarName &&
                    OldChildren[0]->As<Exprs::VarExpression>()->GetVarType() == MsgRecType) {
                    
                    ExprTypeRef ActMsgRecType = nullptr;
                    if (MsgRecType->Is<ExprRecordType>()) {
                        ActMsgRecType = MsgRecType;
                    } else if (MsgRecType->Is<ExprParametricType>()) {
                        ActMsgRecType = MsgRecType->SAs<ExprParametricType>()->GetBaseType();
                    } else {
                        throw ESMCError((string)"MsgTransformer: Message type \"" + 
                                        MsgRecType->ToString() + "\" is not a parametric " + 
                                        "type or a record type");
                    }

                    auto MTypeAsUnion = UnifiedMType->As<ExprUnionType>();
                    auto FieldVarExp = OldChildren[0]->As<VarExpression>();
                    auto const& OldFieldName = FieldVarExp->GetVarName();
                    auto const& NewFieldName = MTypeAsUnion->MapFromMemberField(ActMsgRecType, 
                                                                                OldFieldName);
                    auto FAType = Mgr->MakeType<Exprs::ExprFieldAccessType>();
                    auto NewFieldVar = Mgr->MakeVar(NewFieldName, FAType);
                    ExpStack.push_back(Mgr->MakeExpr(LTSOps::OpField, NewChildren[0], NewFieldVar));
                } else {
                    ExpStack.push_back(Mgr->MakeExpr(OpCode, NewChildren));
                }
            }

            void MsgTransformer::VisitEQuantifiedExpression(const EQExpT* Exp)
            {
                Exp->GetQExpression()->Accept(this);
                auto NewQExpr = ExpStack.back();
                ExpStack.pop_back();
                ExpStack.push_back(Mgr->MakeExists(Exp->GetQVarTypes(), NewQExpr));
            }

            void MsgTransformer::VisitAQuantifiedExpression(const AQExpT* Exp)
            {
                Exp->GetQExpression()->Accept(this);
                auto NewQExpr = ExpStack.back();
                ExpStack.pop_back();
                ExpStack.push_back(Mgr->MakeForAll(Exp->GetQVarTypes(), NewQExpr));
            }

            ExpT MsgTransformer::Do(MgrT* Mgr, 
                                    const ExpT& Exp,
                                    const string& MsgVarName,
                                    const ExprTypeRef& MsgRecType,
                                    const ExprTypeRef& UnifiedMType)
            {
                MsgTransformer TheTransformer(Mgr, MsgVarName, MsgRecType, UnifiedMType);
                Exp->Accept(&TheTransformer);
                return TheTransformer.ExpStack[0];
            }
        } /* end namespace Detail */

        AutomatonBase::AutomatonBase(LabelledTS* TheLTS, const string& Name,
                                     const vector<ExpT>& Params, const ExpT& Constraint)
            : TheLTS(TheLTS), Name(Name), Params(Params), Constraint(Constraint),
              StatesFrozen(false)
        {
            auto Mgr = TheLTS->GetMgr();
            CheckParams(Params, Constraint, SymTab, Mgr, true);

            // Instantiate the parameters
            const u32 NumParams = Params.size();
            ParamInsts = InstantiateParams(Params, Constraint, Mgr);

            for (auto const& ParamInst : ParamInsts) {
                MgrT::SubstMapT SubstMap;
                for (u32 i = 0; i < NumParams; ++i) {
                    SubstMap[Params[i]] = ParamInst[i];
                }
                ParamSubsts.push_back(SubstMap);
            }
        }

        AutomatonBase::~AutomatonBase()
        {
            // Nothing here
        }

        void AutomatonBase::AssertStatesFrozen() const
        {
            if (!StatesFrozen) {
                throw ESMCError((string)"An operation was attempted before states of the " + 
                                "automaton \"" + Name + "\" were frozen. This operation " + 
                                "can only be performed after freezing states");
            }
        }

        void AutomatonBase::AssertStatesNotFrozen() const
        {
            if (StatesFrozen) {
                throw ESMCError((string)"An operation was attempted after states of the " + 
                                "automaton \"" + Name + "\" were frozen. This operation " + 
                                "can only be performed before freezing states");
            }
        }

        void AutomatonBase::CheckState(const string& StateName) const
        {
            if (States.find(StateName) == States.end()) {
                throw ESMCError((string)"Automaton named \"" + Name + "\" has no state named \"" + 
                                StateName + "\"");
            }
        }

        void AutomatonBase::FreezeStates()
        {
            auto Mgr = TheLTS->GetMgr();
            if (StatesFrozen) {
                return;
            }

            set<string> StateNames;
            for (auto const& State : States) {
                StateNames.insert(State.first);
            }

            StateType = Mgr->MakeType<ExprEnumType>(Name + "_StateT", StateNames);
            StatesFrozen = true;
        }

        void AutomatonBase::AddState(const string& StateName,
                                     bool Initial, bool Final,
                                     bool Accepting, bool Error)
        {
            if (States.find(StateName) != States.end()) {
                throw ESMCError((string)"Error, state \"" + StateName + "\" already " + 
                                "declared for automaton \"" + Name + "\"");
            }

            States[StateName] = LTSState(StateName, Initial, Final, Accepting, Error);
        }

        vector<LTSState> AutomatonBase::GetStates() const
        {
            vector<LTSState> Retval;
            for (auto const& State : States) {
                Retval.push_back(State.second);
            }
            return Retval;
        }

        const ExprTypeRef& AutomatonBase::GetStateType() const
        {
            if (!StatesFrozen) {
                throw ESMCError((string)"Cannot call AutomatonBase::GetStateType() before " + 
                                "freezing states of automaton using Automaton::FreezeStates()");
            }
            return StateType;
        }

        const vector<vector<ExpT>>& AutomatonBase::GetParamInsts() const
        {
            return ParamInsts;
        }

        const vector<MgrT::SubstMapT>& AutomatonBase::GetParamSubsts() const
        {
            return ParamSubsts;
        }

        u32 AutomatonBase::GetNumInstances() const
        {
            return ParamInsts.size();
        }

        u32 AutomatonBase::GetNumInstancesUnconstrained() const
        {
            u32 Retval = 1;
            for (auto const& Param : Params) {
                Retval *= Param->GetType()->GetCardinality();
            }
            return Retval;
        }

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
                                            SplatFairnessType SplatFairness)
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

                SplatPrefix = ((string)"SplatFairness_" + 
                               to_string(FairnessUIDGenerator.GetUID()));
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
                                              SplatFairnessType SplatFairness)
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

                SplatPrefix = ((string)"SplatFairness_" + 
                               to_string(FairnessUIDGenerator.GetUID()));
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

        GeneralEFSM::GeneralEFSM(LabelledTS* TheLTS, const string& Name,
                                 const vector<ExpT>& Params, 
                                 const ExpT& Constraint,
                                 LTSFairnessType Fairness)
            : AutomatonBase(TheLTS, Name, Params, Constraint),
              EFSMBase(TheLTS, Name, Params, Constraint, Fairness)
        {
            // Nothing here
        }

        GeneralEFSM::~GeneralEFSM()
        {
            // Nothing here
        }
            
        DetEFSM::DetEFSM(LabelledTS* TheLTS, const string& Name,
                         const vector<ExpT>& Params, const ExpT& Constraint,
                         LTSFairnessType Fairness)
            : AutomatonBase(TheLTS, Name, Params, Constraint),
              EFSMBase(TheLTS, Name, Params, Constraint, Fairness)
        {
            // Nothing here
        }


        DetEFSM::~DetEFSM()
        {
            // Nothing here
        }

        void DetEFSM::AddInputTransition(const string& InitState,
                                         const string& FinalState,
                                         const ExpT& Guard,
                                         const vector<LTSAssignRef>& Updates,
                                         const string& MessageName,
                                         const ExprTypeRef& MessageType,
                                         const vector<ExpT>& MessageParams)
        {
            // TODO: Check for determinism
            EFSMBase::AddInputTransition(InitState, FinalState, Guard, 
                                         Updates, MessageName, MessageType, 
                                         MessageParams);
        }

        void DetEFSM::AddInputTransitions(const vector<ExpT>& TransParams,
                                          const ExpT& Constraint,
                                          const string& InitState,
                                          const string& FinalState,
                                          const ExpT& Guard,
                                          const vector<LTSAssignRef>& Updates,
                                          const string& MessageName,
                                          const ExprTypeRef& MessageType,
                                          const vector<ExpT>& MessageParams)
        {
            // TODO: Check for determinism
            EFSMBase::AddInputTransitions(TransParams, Constraint,
                                          InitState, FinalState, Guard,
                                          Updates, MessageName, MessageType,
                                          MessageParams);
        }

        void DetEFSM::AddOutputTransition(const string& InitState,
                                          const string& FinalState,
                                          const ExpT& Guard,
                                          const vector<LTSAssignRef>& Updates,
                                          const string& MessageName,
                                          const ExprTypeRef& MessageType,
                                          const vector<ExpT>& MessageParams,
                                          const set<string>& AddToFairnessSets)
        {
            // TODO: Check for determinism
            EFSMBase::AddOutputTransition(InitState, FinalState, Guard,
                                          Updates, MessageName, MessageType,
                                          MessageParams, AddToFairnessSets);
        }

        void DetEFSM::AddOutputTransitions(const vector<ExpT>& TransParams, 
                                           const ExpT& Constraint, 
                                           const string& InitState, 
                                           const string& FinalState, 
                                           const ExpT &Guard, 
                                           const vector<LTSAssignRef>& Updates, 
                                           const string& MessageName, 
                                           const ExprTypeRef& MessageType, 
                                           const vector<ExpT>& MessageParams, 
                                           LTSFairnessType FairnessKind,
                                           SplatFairnessType SplatFairness)
        {
            // TODO: Check for determinism
            EFSMBase::AddOutputTransitions(TransParams, Constraint, InitState, 
                                           FinalState, Guard, Updates, MessageName, 
                                           MessageType, MessageParams, 
                                           FairnessKind, SplatFairness);
        }

        void DetEFSM::AddInternalTransition(const string& InitState,
                                            const string& FinalState,
                                            const ExpT& Guard,
                                            const vector<LTSAssignRef>& Updates,
                                            const set<string>& AddToFairnessSets)
        {
            // TODO: Check for determinism
            EFSMBase::AddInternalTransition(InitState, FinalState, Guard, 
                                            Updates, AddToFairnessSets);
        }

        void DetEFSM::AddInternalTransitions(const vector<ExpT>& TransParams,
                                             const ExpT& Constraint,
                                             const string& InitState,
                                             const string& FinalState,
                                             const ExpT& Guard,
                                             const vector<LTSAssignRef>& Updates,
                                             LTSFairnessType FairnessKind,
                                             SplatFairnessType SplatFairness)
        {
            // TODO: Check for determinism
            EFSMBase::AddInternalTransitions(TransParams, Constraint, InitState, 
                                             FinalState, Guard, Updates, 
                                             FairnessKind, SplatFairness);
        }

        ChannelEFSM::ChannelEFSM(LabelledTS* TheLTS, const string& Name,
                                 const vector<ExpT>& Params, const ExpT& Constraint,
                                 u32 Capacity, bool Lossy, bool Ordered, bool Duplicating,
                                 bool Blocking, LTSFairnessType Fairness)
              : AutomatonBase(TheLTS, Name, Params, Constraint), 
                EFSMBase(TheLTS, Name, Params, Constraint, Fairness),
                Capacity(Capacity), Lossy(Lossy), Ordered(Ordered),
                Duplicating(Duplicating), Blocking(Blocking)
        {
            if (Blocking && !Lossy) {
                throw ESMCError((string)"Non lossy channels cannot be declared blocking");
            }
            
            AddState("ChanInitState");
            if (Lossy) {
                AddState("LossDecideState");
            }

            EFSMBase::FreezeStates();
            auto Mgr = TheLTS->GetMgr();
            IndexType = Mgr->MakeType<ExprRangeType>(0, Capacity - 1);
            ValType = TheLTS->GetUnifiedMType();
            ArrayType = Mgr->MakeType<ExprArrayType>(IndexType, ValType);
            CountType = Mgr->MakeType<ExprRangeType>(0, Capacity);
            
            EFSMBase::AddVariable("MsgBuffer", ArrayType);
            EFSMBase::AddVariable("MsgCount", CountType);
            
            if (Lossy) {
                EFSMBase::AddVariable("LastMsg", ValType);
            }

            EFSMBase::FreezeVars();

            CountExp = Mgr->MakeVar("MsgCount", CountType);
            ArrayExp = Mgr->MakeVar("MsgBuffer", ArrayType);
            IndexExp = Mgr->MakeExpr(LTSOps::OpIndex, ArrayExp, CountExp);
            LastMsgExp = Mgr->MakeVar("LastMsg", ValType);
            MaxChanExp = Mgr->MakeVal(to_string(Capacity), CountType);
            OneExp = Mgr->MakeVal("1", CountType);
            ZeroExp = Mgr->MakeVal("0", CountType);
        }

        ChannelEFSM::~ChannelEFSM()
        {
            // Nothing here
        }

        void ChannelEFSM::FreezeStates()
        {
            throw ESMCError((string)"ChannelEFSM::FreezeStates() should not be called");
        }

        void ChannelEFSM::FreezeVars()
        {
            throw ESMCError((string)"ChannelEFSM::FreezeVars() should not be called");
        }

        void ChannelEFSM::AddFairnessSet(const string& Name, FairSetFairnessType Fairness)
        {
            throw ESMCError((string)"ChannelEFSM::AddFairnessSet() should not be called");
        }

        inline void ChannelEFSM::MakeInputTransition(u32 InstanceID,
                                                     const MgrT::SubstMapT& SubstMap,
                                                     const ExprTypeRef& MessageType,
                                                     LossDupFairnessType LossDupFairness)
        {
            auto Mgr = TheLTS->GetMgr();
            auto const& UMType = TheLTS->GetUnifiedMType();

            auto RecType = MessageType->As<ExprRecordType>();
            if (RecType == nullptr) {
                throw ESMCError((string)"Expected Record type as message type " + 
                                "in ChannelEFSM::MakeInputTransition()");
            }
            
            ExpT TargetExp = nullptr;
            if (Lossy) {
                TargetExp = LastMsgExp;
            } else {
                TargetExp = IndexExp;
            }

            auto FAType = Mgr->MakeType<ExprFieldAccessType>();
            vector<LTSAssignRef> Updates;
            vector<LTSAssignRef> NoCountUpdates;
            string InMsgName = "__inmsg__";
            auto InMsgVar = Mgr->MakeVar(InMsgName, UMType);
            auto TrueExp = Mgr->MakeTrue();
            
            auto Guard = Mgr->MakeExpr(LTSOps::OpLT, CountExp, MaxChanExp);
            NoCountUpdates.push_back(new LTSAssignSimple(TargetExp, InMsgVar));
            Updates = NoCountUpdates;
            Updates.push_back(new LTSAssignSimple(CountExp, Mgr->MakeExpr(LTSOps::OpADD,
                                                                          CountExp, OneExp)));
            if (!Lossy) {
                EFSMBase::AddInputTransForInstance(InstanceID,
                                                   SubstMap,
                                                   "ChanInitState", 
                                                   "ChanInitState", 
                                                   Guard, 
                                                   Updates, 
                                                   InMsgName, 
                                                   MessageType, 
                                                   MessageType);
            } else {
                vector<LTSAssignRef> Step2Updates;
                Step2Updates.push_back(new LTSAssignSimple(LastMsgExp, IndexExp));
                Step2Updates.push_back(new LTSAssignSimple(CountExp,
                                                           Mgr->MakeExpr(LTSOps::OpADD,
                                                                         CountExp, OneExp)));
                Step2Updates.push_back(new LTSAssignSimple(LastMsgExp, 
                                                           Mgr->MakeVal("clear", UMType)));
                vector<LTSAssignRef> LossUpdates;
                LossUpdates.push_back(new LTSAssignSimple(LastMsgExp, 
                                                          Mgr->MakeVal("clear", UMType)));

                ExpT Guard2 = nullptr;
                if (Blocking) {
                    EFSMBase::AddInputTransForInstance(InstanceID,
                                                       SubstMap,
                                                       "ChanInitState", 
                                                       "LossDecideState",
                                                       Guard, 
                                                       Updates, 
                                                       InMsgName, 
                                                       MessageType,
                                                       MessageType);
                    Guard2 = TrueExp;
                } else {
                    EFSMBase::AddInputTransForInstance(InstanceID,
                                                       SubstMap,
                                                       "ChanInitState", 
                                                       "LossDecideState",
                                                       TrueExp, 
                                                       Updates, 
                                                       InMsgName, 
                                                       MessageType,
                                                       MessageType);
                    Guard2 = Guard;
                }

                set<string> AddToFairnessSets;
                if (LossDupFairness == LossDupFairnessType::NotAlwaysLost || 
                    LossDupFairness == LossDupFairnessType::NotAlwaysLostOrDup) {
                    auto LDFairID = LossDupFairnessUIDGen.GetUID();
                    string FairName = "LossFairness_ " + to_string(LDFairID);
                    EFSMBase::AddFairnessSet(FairName, FairSetFairnessType::Strong);
                    AddToFairnessSets.insert(FairName);
                }

                // Lossy transition
                EFSMBase::AddInternalTransForInstance(InstanceID,
                                                      SubstMap,
                                                      "LossDecideState", 
                                                      "ChanInitState", 
                                                      TrueExp, 
                                                      LossUpdates, 
                                                      set<string>());
                // Non lossy
                EFSMBase::AddInternalTransForInstance(InstanceID,
                                                      SubstMap,
                                                      "LossDecideState", 
                                                      "ChanInitState",
                                                      Guard2, 
                                                      Step2Updates, 
                                                      AddToFairnessSets);
            }
        }

        void ChannelEFSM::MakeOutputTransition(u32 InstanceID,
                                               const MgrT::SubstMapT& SubstMap,
                                               const ExprTypeRef& MessageType, 
                                               LTSFairnessType MessageFairness, 
                                               ESMC::LTS::LossDupFairnessType LossDupFairness)
        {
            auto Mgr = TheLTS->GetMgr();
            auto PMessageType = TheLTS->GetPrimedType(MessageType);
            auto UMType = TheLTS->GetUnifiedMType()->As<ExprUnionType>();

            vector<ExpT> ChooseExps;
            if (!Ordered) {
                for (u32 i = 0; i < Capacity; ++i) {
                    ChooseExps.push_back(Mgr->MakeVal(to_string(i), IndexType));
                }
            } else {
                ChooseExps.push_back(ZeroExp);
            }

            set<string> AddToFairnessSets;
            bool AddFairnessToDup = false;
            if (!Duplicating) {
                if (MessageFairness != LTSFairnessType::None) {
                    string FairnessSetName = "MessageFairness_" + 
                        to_string(MessageFairnessUIDGen.GetUID());
                    EFSMBase::AddFairnessSet(FairnessSetName,
                                             MessageFairness == LTSFairnessType::Strong ? 
                                             FairSetFairnessType::Strong : 
                                             FairSetFairnessType::Weak);
                    AddToFairnessSets.insert(FairnessSetName);
                }
            } else {
                if (LossDupFairness == LossDupFairnessType::NotAlwaysDup ||
                    LossDupFairness == LossDupFairnessType::NotAlwaysLostOrDup) {
                    string FairnessSetName = "DupFairness_" + 
                        to_string(LossDupFairnessUIDGen.GetUID());
                    EFSMBase::AddFairnessSet(FairnessSetName,
                                             FairSetFairnessType::Strong);
                    AddToFairnessSets.insert(FairnessSetName);
                } else if (MessageFairness != LTSFairnessType::None) {
                    string FairnessSetName = "MessageFairness_" + 
                        to_string(MessageFairnessUIDGen.GetUID());
                    EFSMBase::AddFairnessSet(FairnessSetName,
                                             MessageFairness == LTSFairnessType::Strong ? 
                                             FairSetFairnessType::Strong : 
                                             FairSetFairnessType::Weak);
                    AddToFairnessSets.insert(FairnessSetName);
                    AddFairnessToDup = true;
                }
            }

            for (auto const& ChooseExp : ChooseExps) {
                auto ChooseIndexExp = Mgr->MakeExpr(LTSOps::OpIndex, ArrayExp, ChooseExp);
                auto TrueExp = Mgr->MakeTrue();
                auto FAType = Mgr->MakeType<ExprFieldAccessType>();
                auto FieldVar = Mgr->MakeVar(UMType->GetTypeIDFieldName(), FAType);
                auto FieldExp = Mgr->MakeExpr(LTSOps::OpField, ChooseIndexExp, FieldVar);
                
                auto NonEmptyExp = Mgr->MakeExpr(LTSOps::OpGT, CountExp, ZeroExp);
                auto TypeID = UMType->GetTypeIDForMemberType(MessageType);
                auto MatchExp = Mgr->MakeExpr(LTSOps::OpEQ, FieldExp, 
                                              Mgr->MakeVal(to_string(TypeID), 
                                                           UMType->GetTypeIDFieldType()));
                auto ChooseOkExp = Mgr->MakeExpr(LTSOps::OpLT, ChooseExp, CountExp);
                auto Guard = Mgr->MakeExpr(LTSOps::OpAND, NonEmptyExp, ChooseOkExp, MatchExp);
                string OutMsgName = "__outmsg__";
                auto OutMsgExp = Mgr->MakeVar(OutMsgName, ValType);
                auto CntSubExp = Mgr->MakeExpr(LTSOps::OpSUB, CountExp, OneExp);
                
                vector<LTSAssignRef> Updates;
                Updates.push_back(new LTSAssignSimple(OutMsgExp, ChooseIndexExp));
                vector<LTSAssignRef> MsgUpdates = Updates;

                // Clear the last message
                Updates.push_back(new LTSAssignSimple(Mgr->MakeExpr(LTSOps::OpIndex,
                                                                    ArrayExp, CntSubExp),
                                                      Mgr->MakeVal("clear", ValType)));

                Updates.push_back(new LTSAssignSimple(CountExp, CntSubExp));
                // Push all the elements past the chosen element back
                for (u32 i = 0; i < Capacity - 1; ++i) {
                    auto Cond = Mgr->MakeExpr(LTSOps::OpGE, Mgr->MakeVal(to_string(i),
                                                                         IndexType),
                                              ChooseExp);
                    auto IfBranch = Mgr->MakeExpr(LTSOps::OpIndex, 
                                                  ArrayExp, 
                                                  Mgr->MakeVal(to_string(i+1), 
                                                               IndexType));
                    auto ElseBranch = Mgr->MakeExpr(LTSOps::OpIndex, 
                                                    ArrayExp, 
                                                    Mgr->MakeVal(to_string(i), 
                                                                 IndexType));
                    auto ITEExp = Mgr->MakeExpr(LTSOps::OpITE, Cond, IfBranch, ElseBranch);
                    Updates.push_back(new LTSAssignSimple(ElseBranch, ITEExp));
                }

                EFSMBase::AddOutputTransForInstance(InstanceID,
                                                    SubstMap,
                                                    "ChanInitState", 
                                                    "ChanInitState", 
                                                    Guard, 
                                                    Updates, 
                                                    OutMsgName, 
                                                    PMessageType, 
                                                    PMessageType,
                                                    AddToFairnessSets);
                if (Duplicating) {
                    EFSMBase::AddOutputTransForInstance(InstanceID,
                                                        SubstMap,
                                                        "ChanInitState", 
                                                        "ChanInitState",
                                                        Guard,
                                                        MsgUpdates, 
                                                        OutMsgName,
                                                        PMessageType, 
                                                        PMessageType,
                                                        (AddFairnessToDup ? 
                                                         AddToFairnessSets : 
                                                         set<string>()));
                }
            }
        }

        void ChannelEFSM::AddMsg(const ExprTypeRef& MessageType,
                                 const vector<ExpT>& Params,
                                 LTSFairnessType MessageFairness,
                                 LossDupFairnessType LossDupFairness)
        {
            auto PMessageType = TheLTS->GetPrimedType(MessageType);
            auto Mgr = TheLTS->GetMgr();
            EFSMBase::AddInputMsg(MessageType, Params);
            EFSMBase::AddOutputMsg(PMessageType, Params);

            const u32 NumInsts = ParamInsts.size();
            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& ParamInst = ParamInsts[i];
                auto const& SubstMap = ParamSubsts[i];
                auto&& SubstParams = SubstAll(Params, SubstMap, Mgr);
                
                auto ActMType = InstantiateType(MessageType, SubstParams, Mgr);
                MakeInputTransition(i, SubstMap, ActMType, LossDupFairness);
                MakeOutputTransition(i, SubstMap, ActMType, MessageFairness, LossDupFairness);
            }
        }

        void ChannelEFSM::AddMsgs(const vector<ExpT> NewParams,
                                  const ExpT& Constraint,    
                                  const ExprTypeRef& MessageType,
                                  const vector<ExpT>& MessageParams,
                                  LTSFairnessType MessageFairness,
                                  LossDupFairnessType LossDupFairness)
        {
            // We need to instantiate these ourselves 
            // because we need more fine grained control
            auto Mgr = TheLTS->GetMgr();
            auto PMessageType = TheLTS->GetPrimedType(MessageType);

            SymTab.Push();
            CheckParams(NewParams, Constraint, SymTab, Mgr, true);
            CheckParams(MessageParams, SymTab);
            SymTab.Pop();

            EFSMBase::AddInputMsgs(NewParams, Constraint, MessageType, MessageParams);
            EFSMBase::AddOutputMsgs(NewParams, Constraint, PMessageType, MessageParams);

            const u32 NumNewParams = NewParams.size();
            const u32 NumInsts = ParamInsts.size();

            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];
                auto SubstConstraint = Mgr->Substitute(SubstMap, Constraint);
                auto const&& NewInsts = InstantiateParams(NewParams, SubstConstraint, Mgr);
                for (auto const& NewInst : NewInsts) {
                    auto LocalSubstMap = SubstMap;
                    for (u32 j = 0; j < NumNewParams; ++j) {
                        LocalSubstMap[NewParams[j]] = NewInst[j];
                    }
                    auto&& SubstParams = SubstAll(MessageParams, LocalSubstMap, Mgr);
                    auto MType = InstantiateType(MessageType, SubstParams, Mgr);
                    auto PMType = TheLTS->GetPrimedType(MType);

                    MakeInputTransition(i, LocalSubstMap, MType, LossDupFairness);
                    MakeOutputTransition(i, LocalSubstMap, MType, MessageFairness, 
                                         LossDupFairness);
                }
            }
        }

        void ChannelEFSM::AddInputMsg(const ExprTypeRef& MessageType,
                                      const vector<ExpT>& Params)
        {
            throw ESMCError((string)"ChannelEFSM::AddInputMsg() should not be called");
        }

        void ChannelEFSM::AddInputMsgs(const vector<ExpT>& NewParams, 
                                       const ExpT& Constraint,
                                       const ExprTypeRef& MessageType,
                                       const vector<ExpT>& MessageParams)
        {
            throw ESMCError((string)"ChannelEFSM::AddInputMsgs() should not be called");
        }

        void ChannelEFSM::AddOutputMsg(const ExprTypeRef& MessageType,
                                       const vector<ExpT>& Params)
        {
            throw ESMCError((string)"ChannelEFSM::AddOutputMsg() should not be called");
        }

        void ChannelEFSM::AddOutputMsgs(const vector<ExpT>& NewParams, const ExpT& Constraint,
                                        const ExprTypeRef& MessageType,
                                        const vector<ExpT>& MessageParams)
        {
            throw ESMCError((string)"ChannelEFSM::AddOutputMsgs() should not be called");
        }


        void ChannelEFSM::AddVariable(const string& VarName, const ExprTypeRef& VarType)
        {
            throw ESMCError((string)"ChannelEFSM::AddVariable() should not be called");
        }

        void ChannelEFSM::AddInputTransition(const string& InitState,
                                             const string& FinalState,
                                             const ExpT& Guard,
                                             const vector<LTSAssignRef>& Updates,
                                             const string& MessageName,
                                             const ExprTypeRef& MessageType,
                                             const vector<ExpT>& MessageParams)
        {
            throw ESMCError((string)"ChannelEFSM::AddInputTransition() should not be called");
        }

        void ChannelEFSM::AddInputTransitions(const vector<ExpT>& TransParams,
                                              const ExpT& Constraint,
                                              const string& InitState,
                                              const string& FinalState,
                                              const ExpT& Guard,
                                              const vector<LTSAssignRef>& Updates,
                                              const string& MessageName,
                                              const ExprTypeRef& MessageType,
                                              const vector<ExpT>& MessageParams)
        {
            throw ESMCError((string)"ChannelEFSM::AddInputTransitions() should not be called");
        }
            
        void ChannelEFSM::AddOutputTransition(const string& InitState,
                                              const string& FinalState,
                                              const ExpT& Guard,
                                              const vector<LTSAssignRef>& Updates,
                                              const string& MessageName,
                                              const ExprTypeRef& MessageType,
                                              const vector<ExpT>& MessageParams,
                                              const set<string>& AddToFairnessSets)
        {
            throw ESMCError((string)"ChannelEFSM::AddOutputTransition() should not be called");
        }

        void ChannelEFSM::AddOutputTransitions(const vector<ExpT>& TransParams,
                                               const ExpT& Constraint,
                                               const string& InitState,
                                               const string& FinalState,
                                               const ExpT& Guard,
                                               const vector<LTSAssignRef>& Updates,
                                               const string& MessageName,
                                               const ExprTypeRef& MessageType,
                                               const vector<ExpT>& MessageParams,
                                               LTSFairnessType MessageFairnes,
                                               SplatFairnessType SplatFairness)
        {
            throw ESMCError((string)"ChannelEFSM::AddOutputTransitions() should not be called");
        }

        
        void ChannelEFSM::AddInternalTransition(const string& InitState,
                                                const string& FinalState,
                                                const ExpT& Guard,
                                                const vector<LTSAssignRef>& Updates,
                                                const set<string>& AddToFairnessSets)
        {
            throw ESMCError((string)"ChannelEFSM::AddInternalTransition() should not be called");
        }

        void ChannelEFSM::AddInternalTransitions(const vector<ExpT>& TransParams,
                                                 const ExpT& Constraint,
                                                 const string& InitState,
                                                 const string& FinalState,
                                                 const ExpT& Guard,
                                                 const vector<LTSAssignRef>& Updates,
                                                 LTSFairnessType MessageFairness,
                                                 SplatFairnessType SplatFairness)
        {
            throw ESMCError((string)"ChannelEFSM::AddInternalTransitions() should not be called");
        }

        MonitorBase::MonitorBase(LabelledTS* TheLTS, const string& Name,
                                 const vector<ExpT>& Params, const ExpT& Constraint)
            : AutomatonBase(TheLTS, Name, Params, Constraint)
        {
            // Nothing here
        }

        MonitorBase::~MonitorBase()
        {
            // Nothing here
        }


        SafetyMonitor::SafetyMonitor(LabelledTS* TheLTS, const string& Name,
                                     const vector<ExpT>& Params, const ExpT& Constraint)
            : AutomatonBase(TheLTS, Name, Params, Constraint),
              MonitorBase(TheLTS, Name, Params, Constraint),
              EFSMBase(TheLTS, Name, Params, Constraint)
        {
            // Nothing here
        }

        SafetyMonitor::~SafetyMonitor()
        {
            // Nothing here
        }

        void SafetyMonitor::AddFairnessSet(const string& Name, FairSetFairnessType Fairness)
        {
            throw ESMCError((string)"Cannot add fairness to monitors");
        }

        void SafetyMonitor::AddOutputMsg(const ExprTypeRef& MessageType,
                                         const vector<ExpT>& Params)
        {
            throw ESMCError((string)"Cannot add output message to monitors");
        }

         void SafetyMonitor::AddOutputMsgs(const vector<ExpT>& NewParams,
                                           const ExpT& Constraint,
                                           const ExprTypeRef& MessageType,
                                           const vector<ExpT>& MessageParams)
         {
             throw ESMCError((string)"Cannot add output messages to monitors");
         }


        void SafetyMonitor::AddOutputTransition(const string& InitState,
                                                const string& FinalState,
                                                const ExpT& Guard,
                                                const vector<LTSAssignRef>& Updates,
                                                const string& MessageName,
                                                const ExprTypeRef& MessageType,
                                                const vector<ExpT>& MessageParams,
                                                const set<string>& AddToFairnessSets)
        {
            throw ESMCError((string)"Cannot add output transitions to monitors");
        }

        void SafetyMonitor::AddOutputTransitions(const vector<ExpT>& TransParams,
                                                 const ExpT& Constraint,
                                                 const string& InitState,
                                                 const string& FinalState,
                                                 const ExpT& Guard,
                                                 const vector<LTSAssignRef>& Updates,
                                                 const string& MessageName,
                                                 const ExprTypeRef& MessageType,
                                                 const vector<ExpT>& MessageParams,
                                                 LTSFairnessType MessageFairness,
                                                 SplatFairnessType SplatFairness)
        {
            throw ESMCError((string)"Cannot add output transitions to monitors");
        }

        void SafetyMonitor::AddInternalTransition(const string& InitState,
                                           const string& FinalState,
                                           const ExpT& Guard,
                                           const vector<LTSAssignRef>& Updates,
                                           const set<string>& AddToFairnessSets)
        {
            throw ESMCError((string)"Cannot add internal transitions to monitors");
        }

        void SafetyMonitor::AddInternalTransitions(const vector<ExpT>& TransParams,
                                                   const ExpT& Constraint,
                                                   const string& InitState,
                                                   const string& FinalState,
                                                   const ExpT& Guard,
                                                   const vector<LTSAssignRef>& Updates,
                                                   LTSFairnessType MessageFairness,
                                                   SplatFairnessType SplatFairness)
        {
            throw ESMCError((string)"Cannot add internal transitions to monitors");
        }

        BuchiMonitor::BuchiMonitor(LabelledTS* TheLTS, const string& Name,
                                   const vector<ExpT>& Params, const ExpT& Constraint)
            : AutomatonBase(TheLTS, Name, Params, Constraint),
              MonitorBase(TheLTS, Name, Params, Constraint)
        {
            for (auto const& Param : Params) {
                if (!Param->GetType()->Is<ExprSymmetricType>()) {
                    throw ESMCError((string)"Buchi monitors can only be parametrized by " + 
                                    "symmetric types");
                }
            }
        }

        BuchiMonitor::~BuchiMonitor()
        {
            // Nothing here
        }

        void BuchiMonitor::AddTransition(const string& InitState,
                                         const string& FinalState,
                                         const ExpT& Guard)
        {
            CheckState(InitState);
            CheckState(FinalState);
            TheLTS->CheckExpr(Guard);
            auto IS = States[InitState];
            auto FS = States[FinalState];

            Transitions.push_back(new BuchiMonitorTransition(this, IS, FS, Guard));
        }

        const vector<BuchiTransRef>& BuchiMonitor::GetTransitions() const
        {
            return Transitions;
        }

        string BuchiMonitor::ToString() const 
        {
            ostringstream sstr;

            sstr << "buchimonitor {" << endl;
            for (auto const& Trans : Transitions) {
                sstr << Trans->ToString(4) << endl;
            }
            sstr << "}" << endl;

            return sstr.str();
        }
        
    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// LTSEFSM.cpp ends here
