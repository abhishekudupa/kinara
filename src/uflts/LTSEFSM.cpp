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
            : TheLTS(TheLTS), Name(Name), Params(Params), Constraint(Constraint)
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
              Fairness(Fairness)
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

        void EFSMBase::AssertInput(const vector<ExpT>& ParamInst, const ExprTypeRef& MessageType) const
        {
            auto it = Inputs.find(ParamInst);
            if (it == Inputs.end()) {
                throw InternalError((string)"Attempted instantation of msg with invalid parameters.\nAt: " + 
                                    __FILE__ + ":" + to_string(__LINE__));
            }
            auto it2 = it->second.find(MessageType);
            if (it2 == it->second.end()) {
                throw ESMCError((string)"Message type \"" + MessageType->ToString() + "\" is not " + 
                                "an input for machine named \"" + Name + "\"");
            }
        }

        void EFSMBase::AssertOutput(const vector<ExpT>& ParamInst, const ExprTypeRef& MessageType) const
        {
            auto it = Outputs.find(ParamInst);
            if (it == Outputs.end()) {
                throw InternalError((string)"Attempted instantation of msg with invalid parameters.\nAt: " + 
                                    __FILE__ + ":" + to_string(__LINE__));
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
                if (IsInput) {
                    Inputs[ParamInsts[i]].insert(ActMsgType);
                } else {
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

                    auto ActMsgType = InstantiateMessageType(Params, LocalSubstMap, MsgType);
                    if (IsInput) {
                        Inputs[ParamInsts[i]].insert(ActMsgType);
                    } else {
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

                    Retval.push_back(new LTSAssignSimple(Mgr->Substitute(SubstMap, PUpdate->GetLHS()),
                                                         Mgr->Substitute(SubstMap, PUpdate->GetRHS())));
                }
            }
            return Retval;
        }

        vector<LTSAssignRef> EFSMBase::RebaseUpdates(const vector<LTSAssignRef>& Updates)
        {
            auto Mgr = TheLTS->GetMgr();
            const u32 NumUpdates = Updates.size();
            vector<LTSAssignRef> Retval(NumUpdates);
            
            for (u32 i = 0; i < NumUpdates; ++i) {
                auto RebasedLHS = Mgr->Substitute(RebaseSubstMap, Updates[i]->GetLHS());
                auto RebasedRHS = Mgr->Substitute(RebaseSubstMap, Updates[i]->GetRHS());
                Retval.push_back(new LTSAssignSimple(RebasedLHS, RebasedRHS));
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
                auto NewLHS = Mgr->ApplyTransform<Detail::MsgTransformer>(LHS, MessageName,
                                                                          MessageType, 
                                                                          TheLTS->GetUnifiedMType());
                auto NewRHS = Mgr->ApplyTransform<Detail::MsgTransformer>(RHS, MessageName,
                                                                          MessageType, 
                                                                          TheLTS->GetUnifiedMType());
                Retval[i] = new LTSAssignSimple(NewLHS, NewRHS);
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
            StateType = ArrayType;

            // Create the RebaseSubstMap
            auto PrefixExp = Mgr->MakeVar(Name, StateType);
            for (auto const& Param : Params) {
                PrefixExp = Mgr->MakeExpr(LTSOps::OpIndex, PrefixExp, Param);
            }

            auto FAType = Mgr->MakeType<ExprFieldAccessType>();
            
            // Create a substitution for each var
            for (auto const& Decl : DeclMap) {
                if (Decl.second->Is<VarDecl>()) {
                    auto From = Mgr->MakeVar(Decl.first, Decl.second->GetType());
                    auto FieldVar = Mgr->MakeVar(Decl.first, FAType);
                    auto To = Mgr->MakeExpr(LTSOps::OpField, PrefixExp, FieldVar);
                    RebaseSubstMap[From] = To;
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

        void EFSMBase::AddInputTransition(const string& InitState, const string& FinalState, 
                                          const ExpT& Guard, const vector<LTSAssignRef>& Updates, 
                                          const string& MessageName, const ExprTypeRef& MessageType, 
                                          const vector<ExpT>& MessageParams)
        {
            AssertStatesFrozen();
            AssertVarsFrozen();
            AssertEFSMNotFrozen();

            auto Mgr = TheLTS->GetMgr();

            CheckState(InitState);
            CheckState(FinalState);
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

            auto IS = States[InitState];
            auto FS = States[FinalState];
            
            const u32 NumInsts = ParamInsts.size();
            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];
                auto ActMType = InstantiateMessageType(MessageParams, SubstMap, MessageType);
                CheckMsgType(ActMType);
                AssertInput(ParamInsts[i], ActMType);
                
                auto&& InstUpdates = InstantiateUpdates(SubstMap, Updates);
                auto&& RebasedUpdates = RebaseUpdates(InstUpdates);
                auto&& MsgTransformedUpdates = MsgTransformUpdates(RebasedUpdates,
                                                                   MessageName, 
                                                                   MessageType);

                auto SubstGuard = Mgr->Substitute(SubstMap, Guard);
                auto RebasedGuard = Mgr->Substitute(RebaseSubstMap, SubstGuard);
                auto MsgTransformedGuard = 
                    Mgr->ApplyTransform<Detail::MsgTransformer>(RebasedGuard,
                                                                MessageName, MessageType,
                                                                TheLTS->GetUnifiedMType());

                auto CurTransition = new LTSTransitionInput(this, IS, FS, MsgTransformedGuard,
                                                            MsgTransformedUpdates, MessageName, 
                                                            ActMType);

                Transitions[ParamInsts[i]].push_back(CurTransition);
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

            CheckState(InitState);
            CheckState(FinalState);
            
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

            auto IS = States[InitState];
            auto FS = States[FinalState];

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

                    auto ActMType = InstantiateMessageType(MessageParams, LocalSubstMap, MessageType);
                    CheckMsgType(ActMType);
                    AssertInput(ParamInsts[i], ActMType);

                    auto&& InstUpdates = InstantiateUpdates(LocalSubstMap, Updates);
                    auto&& RebasedUpdates = RebaseUpdates(InstUpdates);
                    auto&& MsgTransformedUpdates = MsgTransformUpdates(RebasedUpdates,
                                                                       MessageName, 
                                                                       MessageType);

                    auto SubstGuard = Mgr->Substitute(LocalSubstMap, Guard);
                    auto RebasedGuard = Mgr->Substitute(RebaseSubstMap, SubstGuard);
                    auto MsgTransformedGuard = 
                        Mgr->ApplyTransform<Detail::MsgTransformer>(RebasedGuard,
                                                                    MessageName, MessageType,
                                                                    TheLTS->GetUnifiedMType());

                    auto CurTransition = new LTSTransitionInput(this, IS, FS, MsgTransformedGuard,
                                                                MsgTransformedUpdates, MessageName,
                                                                ActMType);
                                                                
                    Transitions[ParamInsts[i]].push_back(CurTransition);
                }
            }
        }

        void EFSMBase::AddOutputTransition(const string& InitState, const string& FinalState, 
                                           const ExpT& Guard, const vector<LTSAssignRef> &Updates, 
                                           const string &MessageName, const ExprTypeRef &MessageType, 
                                           const vector<ExpT> &MessageParams,
                                           const set<string>& AddToFairnessSets)
        {
            AssertStatesFrozen();
            AssertVarsFrozen();
            AssertEFSMNotFrozen();

            auto Mgr = TheLTS->GetMgr();

            CheckState(InitState);
            CheckState(FinalState);
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

            auto IS = States[InitState];
            auto FS = States[FinalState];

            const u32 NumInsts = ParamInsts.size();

            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];

                auto ActMType = InstantiateMessageType(MessageParams, SubstMap, MessageType);
                CheckMsgType(ActMType);
                AssertOutput(ParamInsts[i], ActMType);

                auto&& InstUpdates = InstantiateUpdates(SubstMap, Updates);
                auto&& RebasedUpdates = RebaseUpdates(InstUpdates);
                auto&& MsgTransformedUpdates = MsgTransformUpdates(RebasedUpdates, 
                                                                   MessageName, 
                                                                   MessageType);

                auto SubstGuard = Mgr->Substitute(SubstMap, Guard);
                auto RebasedGuard = Mgr->Substitute(RebaseSubstMap, SubstGuard);
                auto MsgTransformedGuard = 
                    Mgr->ApplyTransform<Detail::MsgTransformer>(RebasedGuard, MessageName, 
                                                                MessageType, 
                                                                TheLTS->GetUnifiedMType());

                // Add to appropriate fairness set if process fairness is set
                auto LocalFairnessSets = AddToFairnessSets;
                if (Fairness != LTSFairnessType::None) {
                    LocalFairnessSets.insert("ProcessFairness");
                }

                auto CurTransition = new LTSTransitionOutput(this, IS, FS, MsgTransformedGuard,
                                                             MsgTransformedUpdates, MessageName,
                                                             ActMType, LocalFairnessSets);
                for (auto const& FairnessSet : LocalFairnessSets) {
                    Fairnesses[FairnessSet][ParamInsts[i]]->AddTransition(CurTransition);
                }
                
                Transitions[ParamInsts[i]].push_back(CurTransition);
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

            CheckState(InitState);
            CheckState(FinalState);

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

            auto IS = States[InitState];
            auto FS = States[FinalState];

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

                    auto ActMType = InstantiateMessageType(MessageParams, LocalSubstMap, MessageType);
                    CheckMsgType(ActMType);
                    AssertOutput(ParamInsts[i], ActMType);

                    auto&& InstUpdates = InstantiateUpdates(LocalSubstMap, Updates);
                    auto&& RebasedUpdates = RebaseUpdates(InstUpdates);
                    auto&& MsgTransformedUpdates = MsgTransformUpdates(RebasedUpdates,
                                                                       MessageName, 
                                                                       MessageType);

                    auto SubstGuard = Mgr->Substitute(LocalSubstMap, Guard);
                    auto RebasedGuard = Mgr->Substitute(RebaseSubstMap, SubstGuard);
                    auto MsgTransformedGuard = 
                        Mgr->ApplyTransform<Detail::MsgTransformer>(RebasedGuard,
                                                                    MessageName, MessageType,
                                                                    TheLTS->GetUnifiedMType());
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
                    }

                    auto CurTransition = new LTSTransitionOutput(this, IS, FS, MsgTransformedGuard,
                                                                 MsgTransformedUpdates, MessageName,
                                                                 ActMType, LocalFairnessSets);
                    
                    if (Fairness != LTSFairnessType::None) {
                        Fairnesses["ProcessFairness"][ParamInsts[i]]->AddTransition(CurTransition);
                    }
                    if (SplatFairness == SplatFairnessType::Group) {
                        Fairnesses[SplatFairnessSetName][ParamInsts[i]]->AddTransition(CurTransition);
                    } else if (SplatFairness == SplatFairnessType::Individual) {
                        auto NewFairnessSet = new LTSFairnessSet(this, SplatFairnessSetName,
                                                                 SetFairnessType);
                        NewFairnessSet->AddTransition(CurTransition);
                        Fairnesses[SplatFairnessSetName][ParamInsts[i]] = NewFairnessSet;
                    }
                                                                
                    Transitions[ParamInsts[i]].push_back(CurTransition);
                }
            }
        }

        void EFSMBase::AddInternalTransition(const string& InitState, const string& FinalState, 
                                             const ExpT& Guard, const vector<LTSAssignRef> &Updates,
                                             const set<string>& AddToFairnessSets)
        {
            AssertStatesFrozen();
            AssertVarsFrozen();
            AssertEFSMFrozen();

            auto Mgr = TheLTS->GetMgr();

            CheckState(InitState);
            CheckState(FinalState);
            CheckFairnessSets(AddToFairnessSets);
            CheckExpr(Guard, SymTab, Mgr);

            CheckUpdates(Updates, SymTab, Mgr, false, "");

            auto IS = States[InitState];
            auto FS = States[FinalState];

            const u32 NumInsts = ParamInsts.size();

            for (u32 i = 0; i < NumInsts; ++i) {
                auto const& SubstMap = ParamSubsts[i];
                auto&& InstUpdates = InstantiateUpdates(SubstMap, Updates);
                auto&& RebasedUpdates = RebaseUpdates(InstUpdates);
             
                auto SubstGuard = Mgr->Substitute(SubstMap, Guard);
                auto RebasedGuard = Mgr->Substitute(RebaseSubstMap, SubstGuard);

                auto LocalFairnessSets = AddToFairnessSets;
                if (Fairness != LTSFairnessType::None) {
                    LocalFairnessSets.insert("ProcessFairness");
                }

                auto CurTransition = new LTSTransitionInternal(this, IS, FS, RebasedGuard,
                                                               RebasedUpdates, LocalFairnessSets);

                for (auto const& FairnessSet : LocalFairnessSets) {
                    Fairnesses[FairnessSet][ParamInsts[i]]->AddTransition(CurTransition);
                }

                Transitions[ParamInsts[i]].push_back(CurTransition);
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

            CheckState(InitState);
            CheckState(FinalState);

            SymTab.Push();

            CheckParams(TransParams, Constraint, SymTab, Mgr, true);
            CheckExpr(Guard, SymTab, Mgr);
            CheckUpdates(Updates, SymTab, Mgr, false, "");

            SymTab.Pop();

            auto IS = States[InitState];
            auto FS = States[FinalState];

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

                    auto&& InstUpdates = InstantiateUpdates(LocalSubstMap, Updates);
                    auto&& RebasedUpdates = RebaseUpdates(InstUpdates);

                    auto SubstGuard = Mgr->Substitute(LocalSubstMap, Guard);
                    auto RebasedGuard = Mgr->Substitute(RebaseSubstMap, Guard);

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
                    }

                    auto CurTransition = new LTSTransitionInternal(this, IS, FS, 
                                                                   RebasedGuard, 
                                                                   RebasedUpdates,
                                                                   LocalFairnessSets);

                    if (Fairness != LTSFairnessType::None) {
                        Fairnesses["ProcessFairness"][ParamInsts[i]]->AddTransition(CurTransition);
                    }
                    if (SplatFairness == SplatFairnessType::Group) {
                        Fairnesses[SplatFairnessSetName][ParamInsts[i]]->AddTransition(CurTransition);
                    } else if (SplatFairness == SplatFairnessType::Individual) {
                        auto NewFairnessSet = new LTSFairnessSet(this, SplatFairnessSetName,
                                                                 SetFairnessType);
                        NewFairnessSet->AddTransition(CurTransition);
                        Fairnesses[SplatFairnessSetName][ParamInsts[i]] = NewFairnessSet;
                    }

                    Transitions[ParamInsts[i]].push_back(CurTransition);
                }
            }
        }

    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// LTSEFSM.cpp ends here













