// LabelledTS.cpp --- 
// 
// Filename: LabelledTS.cpp
// Author: Abhishek Udupa
// Created: Mon Aug 11 11:11:33 2014 (-0400)
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

#include "../utils/CombUtils.hpp"

#include "LabelledTS.hpp"
#include "LTSUtils.hpp"
#include "LTSEFSM.hpp"
#include "LTSTransitions.hpp"


namespace ESMC {
    namespace LTS {

        namespace Detail {
            
            QuantifierUnroller::QuantifierUnroller(MgrT* Mgr)
                : VisitorBaseT("QuantifierUnroller"), Mgr(Mgr)
            {
                // Nothing here
            }

            QuantifierUnroller::~QuantifierUnroller()
            {
                // Nothing here
            }

            void QuantifierUnroller::VisitVarExpression(const VarExpT* Exp) 
            {
                ExpStack.push_back(Exp);
            }

            void QuantifierUnroller::VisitBoundVarExpression(const BoundVarExpT* Exp)
            {
                ExpStack.push_back(Exp);
            }

            void QuantifierUnroller::VisitConstExpression(const ConstExpT* Exp) 
            {
                ExpStack.push_back(Exp);
            }

            void QuantifierUnroller::VisitOpExpression(const OpExpT* Exp)
            {
                VisitorBaseT::VisitOpExpression(Exp);
                auto const& Children = Exp->GetChildren();
                const u32 NumChildren = Children.size();

                vector<ExpT> NewChildren(NumChildren);
                for (u32 i = 0; i < NumChildren; --i) {
                    NewChildren[NumChildren - i - 1] = ExpStack.back();
                    ExpStack.pop_back();
                }
                
                ExpStack.push_back(Mgr->MakeExpr(Exp->GetOpCode(), NewChildren));
            }

            inline vector<ExpT>
            QuantifierUnroller::UnrollQuantifier(const QExpT* Exp)
            {
                vector<ExpT> Retval;
                auto const& QVarTypes = Exp->GetQVarTypes();
                auto const& QBody = Exp->GetQExpression();
                const u32 NumQVars = QVarTypes.size();

                vector<vector<string>> QVarElems;
                for (auto const& QVarType : QVarTypes) {
                    QVarElems.push_back(QVarType->GetElements());
                }
                auto&& CP = CrossProduct<string>(QVarElems.begin(), 
                                                  QVarElems.end());
                for (auto const& CPElem : CP) {
                    MgrT::SubstMapT SubstMap;
                    for (u32 i = 0; i < NumQVars; ++i) {
                        auto const& BoundVarExp = Mgr->MakeBoundVar(QVarTypes[i], i);
                        auto const& ValExp = Mgr->MakeVal(CPElem[i], QVarTypes[i]);
                        SubstMap[BoundVarExp] = ValExp;
                    }

                    Retval.push_back(Mgr->Substitute(SubstMap, QBody));
                }
                return Retval;
            }

            void QuantifierUnroller::VisitEQuantifiedExpression(const EQExpT* Exp)
            {
                Exp->GetQExpression()->Accept(this);
                auto const& NewExp = ExpStack.back();
                ExpStack.pop_back();
                auto NewQExp = Mgr->MakeExists(Exp->GetQVarTypes(), NewExp);
                auto&& UnrolledExps = 
                    UnrollQuantifier(NewQExp->SAs<Exprs::QuantifiedExpressionBase>());
                ExpStack.push_back(Mgr->MakeExpr(LTSOps::OpOR, UnrolledExps));
            }

            void QuantifierUnroller::VisitAQuantifiedExpression(const AQExpT* Exp)
            {
                Exp->GetQExpression()->Accept(this);
                auto const& NewExp = ExpStack.back();
                ExpStack.pop_back();
                auto NewQExp = Mgr->MakeForAll(Exp->GetQVarTypes(), NewExp);
                auto&& UnrolledExps = 
                    UnrollQuantifier(NewQExp->SAs<Exprs::QuantifiedExpressionBase>());
                ExpStack.push_back(Mgr->MakeExpr(LTSOps::OpAND, UnrolledExps));
            }

            ExpT QuantifierUnroller::Do(MgrT* Mgr, const ExpT& Exp)
            {
                QuantifierUnroller Unroller(Mgr);
                Exp->Accept(&Unroller);
                return Unroller.ExpStack[0];
            }

        } /* end namespace Detail */

        LabelledTS::LabelledTS()
            : Mgr(new MgrT()),
              Frozen(false), MsgsFrozen(false), AutomataFrozen(false)
        {
            // Nothing here
        }

        LabelledTS::~LabelledTS()
        {
            delete Mgr;
            for (auto const& NameEFSM : AllEFSMs) {
                delete NameEFSM.second;
            }

            for (auto const& NameMon : SafetyMonitors) {
                delete NameMon.second;
            }
            for (auto const& BuchiMon : BuchiMonitors) {
                delete BuchiMon.second;
            }
        }

        // helpers
        inline void LabelledTS::AssertFrozen() const
        {
            if (!Frozen) {
                throw ESMCError((string)"Operation cannot be performed before freezing " + 
                                "the Labelled Transition System");
            }
        }

        inline void LabelledTS::AssertNotFrozen() const
        {
            if (Frozen) {
                throw ESMCError((string)"Operation cannot be performed after freezing " + 
                                "the Labelled Transition System");
            }

        }

        inline void LabelledTS::AssertMsgsFrozen() const
        {
            if (!MsgsFrozen) {
                throw ESMCError((string)"Operation cannot be performed before freezing " + 
                                "messages of the Labelled Transition System");                
            }
        }

        inline void LabelledTS::AssertMsgsNotFrozen() const
        {
            if (MsgsFrozen) {
                throw ESMCError((string)"Operation cannot be performed after freezing " + 
                                "messages of the Labelled Transition System");                
            }
        }

        inline void LabelledTS::AssertAutomataFrozen() const
        {
            if (!AutomataFrozen) {
                throw ESMCError((string)"Operation cannot be performed before freezing " + 
                                "the automata of the Labelled Transition System");                
            }
        }

        inline void LabelledTS::AssertAutomataNotFrozen() const
        {
            if (AutomataFrozen) {
                throw ESMCError((string)"Operation cannot be performed after freezing " + 
                                "the automata of the Labelled Transition System");                
            }
        }

        inline void LabelledTS::MarkExpr(ExpT& Exp) const
        {
            ExpT->ExtensionData.CreatedViaLTS = true;
        }

        inline void LabelledTS::MarkType(ExprTypeRef& Type) const
        {
            Type->AddExtension(new UFLTSTypeExtensionT());
        }

        void LabelledTS::Freeze()
        {
            // TODO: implement me
        }

        void LabelledTS::FreezeAutomata()
        {
            AssertMsgsFrozen();
            if (AutomataFrozen) {
                return;
            }

            AutomataFrozen = true;
            
            StateVectorSize = 0;
            // Create the state variables
            for (auto& NameEFSM : AllEFSMs) {
                auto EFSM = NameEFSM.second;
                EFSM->Freeze();
                auto const& Name = EFSM->Name;
                auto const& StateVarType = EFSM->StateVarType;

                StateVectorVars[Mgr->MakeVar(Name, StateVarType)] = 
                    set<vector<ExpT>>(EFSM->ParamInsts.begin(),
                                      EFSM->ParamInsts.end());

                StateVectorSize += StateVarType->GetByteSize();
                ValidAutomata[EFSM->Name] = 
                    set<vector<ExpT>>(EFSM->ParamInsts.begin(),
                                      EFSM->ParamInsts.end());
            }
        }
        
        void LabelledTS::FreezeMsgs()
        {
            if (MsgsFrozen) {
                return;
            }

            MsgsFrozen = true;

            // create the union type
            set<ExprTypeRef> UnionMembers;
            for (auto const& MsgType : MsgTypes) {
                UnionMembers.insert(MsgType.second);
            }

            auto TypeIDFieldType = Mgr->MakeType<ExprRangeType>(0, 65000);
            UnifiedMsgType = Mgr->MakeType<ExprUnionType>("UnifiedMsgType",
                                                          UnionMembers, 
                                                          TypeIDFieldType);
        }

        // Expression and type creation in LTS
        ExprTypeRef LabelledTS::MakeBoolType()
        {
            Ret
        }

        MgrT* LabelledTS::GetMgr() const
        {
            return Mgr;
        }

        bool LabelledTS::CheckMessageType(const ExprTypeRef& MsgType) const
        {
            AssertMsgsFrozen();

            if (MsgType->Is<ExprParametricType>()) {
                auto TypeAsPar = MsgType->SAs<ExprParametricType>();
                auto BaseRecType = TypeAsPar->GetBaseType()->As<ExprRecordType>();
                auto it = ParametricMsgTypes.find(BaseRecType->GetName());
                if (it == ParametricMsgTypes.end()) {
                    return false;
                } else {
                    return true;
                }
            } else if (MsgType->Is<ExprRecordType>()) {
                auto TypeAsRec = MsgType->SAs<ExprRecordType>();
                auto it = MsgTypes.find(TypeAsRec->GetName());
                if (it == MsgTypes.end()) {
                    return false;
                } else {
                    return true;
                }
            } else {
                return false;
            }
        }

        const ExprTypeRef& LabelledTS::GetUnifiedMType() const
        {
            AssertMsgsFrozen();
            return UnifiedMsgType;
        }

        const ExprTypeRef& LabelledTS::MakeSymmType(const string& Name,
                                                    u32 Size)
        {
            if (SymmTypes.find(Name) != SymmTypes.end()) {
                throw ESMCError((string)"Symmetric type name \"" + Name + "\" has already " + 
                                "been created");
            }

            SymmTypes[Name] = Mgr->MakeType<ExprSymmetricType>(Name, Size);
            return SymmTypes[Name];
        }

        const ExprTypeRef& LabelledTS::MakeMsgType(const string& Name, 
                                                   const vector<pair<string, ExprTypeRef>> &Members,
                                                   bool IncludePrimed)
        {
            AssertMsgsNotFrozen();

            if (MsgTypes.find(Name) != MsgTypes.end() || 
                ParametricMsgTypes.find(Name) != ParametricMsgTypes.end()) {
                throw ESMCError((string)"Message type name \"" + Name + "\" has already been " + 
                                "created");
            }

            MsgTypes[Name] = Mgr->MakeType<ExprRecordType>(Name, Members);
            if (IncludePrimed) {
                auto PrimedName = Name + "'";
                MsgTypes[PrimedName] = Mgr->MakeType<ExprRecordType>(Name, Members);
                TypeToPrimed[MsgTypes[Name]] = MsgTypes[PrimedName];
            }

            return MsgTypes[Name];
        }

        const ExprTypeRef& LabelledTS::MakeMsgTypes(const vector<ExpT>& Params,
                                                    const ExpT& Constraint,
                                                    const string& Name,
                                                    const vector<pair<string, ExprTypeRef>>& Members,
                                                    bool IncludePrimed)
        {
            AssertMsgsNotFrozen();

            SymTab.Push();
            CheckParams(Params, Constraint, SymTab, Mgr, true);
            SymTab.Pop();

            auto&& ParamInsts = InstantiateParams(Params, Constraint, Mgr);
            if (ParamInsts.size() == 0) {
                throw ESMCError((string)"Instantiation of parametric type \"" + Name + 
                                "\" is empty");
            }

            if (ParametricMsgTypes.find(Name) != ParametricMsgTypes.end() || 
                MsgTypes.find(Name) != MsgTypes.end()) {
                throw ESMCError((string)"Message type name \"" + Name + "\" has already been " + 
                                "created");
            }

            auto BaseRecType = Mgr->MakeType<ExprRecordType>(Name, Members);
            vector<ExprTypeRef> ParamTypes;
            for (auto const& Param : Params) {
                ParamTypes.push_back(Param->GetType());
            }

            auto PType = Mgr->MakeType<ExprParametricType>(BaseRecType, ParamTypes);
            ParametricMsgTypes[Name] = PType;

            ExprTypeRef PBaseRecType;
            ExprTypeRef PPType = nullptr;
            string PName;
            if (IncludePrimed) {
                PName = Name + "'";
                PBaseRecType = Mgr->MakeType<ExprRecordType>(PName, Members);
                PPType = Mgr->MakeType<ExprParametricType>(PBaseRecType, ParamTypes);
                ParametricMsgTypes[PName] = PPType;
                TypeToPrimed[ParametricMsgTypes[Name]] = ParametricMsgTypes[PName];
            }

            for (auto const& ParamInst : ParamInsts) {
                auto InstType = InstantiateType(PType, ParamInst, Mgr);
                MsgTypes[InstType->As<ExprRecordType>()->GetName()] = InstType;

                if (IncludePrimed) {
                    auto PInstType = InstantiateType(PPType, ParamInst, Mgr);
                    MsgTypes[PInstType->As<ExprRecordType>()->GetName()] = PInstType;
                    TypeToPrimed[MsgTypes[InstType->As<ExprRecordType>()->GetName()]] = 
                        MsgTypes[PInstType->As<ExprRecordType>()->GetName()];
                }
            }

            return ParametricMsgTypes[Name];
        }

        const ExprTypeRef& LabelledTS::GetPrimedType(const ExprTypeRef& Type) const
        {
            auto it = TypeToPrimed.find(Type);
            if (it == TypeToPrimed.end()) {
                throw ESMCError((string)"Could not find primed type for type:\n" + 
                                Type->ToString());
            }
            return it->second;
        }

        void LabelledTS::CheckExpr(const ExpT& Expr) const
        {
            
        }

        EFSMBase* LabelledTS::MakeGenEFSM(const string& Name, const vector<ExpT>& Params,
                                          const ExpT& Constraint, LTSFairnessType Fairness)
        {
            AssertMsgsFrozen();
            AssertAutomataNotFrozen();

            if (AllEFSMs.find(Name) != AllEFSMs.end()) {
                throw ESMCError((string)"A machine named \"" + Name + "\" has already " + 
                                "been created in the LTS");
            }

            auto Retval = new GeneralEFSM(this, Name, Params, Constraint, Fairness);
            AllEFSMs[Name] = Retval;
            ActualEFSMs[Name] = Retval;
            return Retval;
        }

        EFSMBase* LabelledTS::MakeDetEFSM(const string& Name, const vector<ExpT>& Params,
                                          const ExpT& Constraint, LTSFairnessType Fairness)
        {
            AssertMsgsFrozen();
            AssertAutomataNotFrozen();

            if (AllEFSMs.find(Name) != AllEFSMs.end()) {
                throw ESMCError((string)"A machine named \"" + Name + "\" has already " + 
                                "been created in the LTS");
            }

            auto Retval = new DetEFSM(this, Name, Params, Constraint, Fairness);
            AllEFSMs[Name] = Retval;
            ActualEFSMs[Name] = Retval;
            return Retval;            
        }

        ChannelEFSM* LabelledTS::MakeChannel(const string& Name, const vector<ExpT> &Params, 
                                             const ExpT& Constraint, u32 Capacity, 
                                             bool Lossy, bool Ordered, bool Duplicating, 
                                             bool Blocking, LTSFairnessType Fairness)
        {
            AssertMsgsFrozen();
            AssertAutomataNotFrozen();

            if (AllEFSMs.find(Name) != AllEFSMs.end()) {
                throw ESMCError((string)"A machine named \"" + Name + "\" has already " + 
                                "been created in the LTS");
            }

            auto Retval = new ChannelEFSM(this, Name, Params, Constraint, Capacity, 
                                          Lossy, Ordered, Duplicating, Blocking, Fairness);
            AllEFSMs[Name] = Retval;
            ChannelEFSMs[Name] = Retval;
            return Retval;
        }

        void LabelledTS::AddInitStates(const vector<InitStateRef>& InitStates)
        {
            AssertAutomataFrozen();

            for (auto const& InitState : InitStates) {
                auto const& Params = InitState->GetParams();
                auto const& Constraint = InitState->GetConstraint();
                
                
            }
        }

    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// LabelledTS.cpp ends here











