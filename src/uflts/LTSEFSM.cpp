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

#include "../tpinterface/TheoremProver.hpp"
#include "../utils/CombUtils.hpp"
#include "../symmetry/Permutations.hpp"

#include "LTSEFSM.hpp"
#include "LTSUtils.hpp"
#include "LabelledTS.hpp"
#include "LTSState.hpp"
#include "LTSTransitions.hpp"
#include "LTSFairnessSet.hpp"

namespace ESMC {
    namespace LTS {

        using namespace ESMC::TP;

        GeneralEFSM::GeneralEFSM(LabelledTS* TheLTS, const string& Name,
                                 const vector<ExpT>& Params, 
                                 const ExpT& Constraint,
                                 LTSFairnessType Fairness)
              : EFSMBase(TheLTS, Name, Params, Constraint, Fairness)
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
            : EFSMBase(TheLTS, Name, Params, Constraint, Fairness)
        {
            // Nothing here
        }


        DetEFSM::~DetEFSM()
        {
            // Nothing here
        }

        static inline bool CheckDisjoint(const ExpT& Exp1, const ExpT& Exp2,
                                         const TPRef& TP)
        {
            auto Mgr = Exp1->GetMgr();
            auto Conjunction = Mgr->MakeExpr(LTSOps::OpAND, Exp1, Exp2);

            cout << "Checking for unsat:" << endl << Conjunction->ToString() 
                 << endl << endl;
            auto Res = TP->CheckSat(Conjunction);
            if (Res == TPResult::UNKNOWN) {
                throw InternalError((string)"Got unknown result from Z3 while " +
                                    "checking determinism of guards.\n" + 
                                    "Guards being checked:\n" + Exp1->ToString() + 
                                    "\n\n" + Exp2->ToString() + "\nAt: " + 
                                    __FILE__ + ":" + to_string(__LINE__));
            } else if (Res == TPResult::SATISFIABLE) {
                return false;
            } else {
                return true;
            }
        }

        // Rules:
        // 1. An input transition t on a message m
        //    - Must have disjoint guards from all other input transitions on m
        //    - Must have disjoint guards from all output transitions
        //    - Must have disjoint guards from all internal transitions
        // 2. An output transition t with a message m
        //    - Must have disjoint guards from all other output transitions
        //    - Must have disjoint guards from all internal transitions
        // 3. An internal transition t
        //    - Must have disjoint guards from all other internal transitions

        inline void DetEFSM::CheckTransition(const TPRef& TP, u32 TransIndex,
                                             const vector<LTSSymbTransRef>& CandTrans) const
        {
            auto const Trans = CandTrans[TransIndex];
            auto TransAsInput = Trans->As<LTSSymbInputTransition>();
            ExprTypeRef MType = ExprTypeRef::NullPtr;

            if (TransAsInput != nullptr) {
                MType = TransAsInput->GetMessageType();
            }

            for (u32 j = TransIndex + 1; j < CandTrans.size(); ++j) {
                auto const& OtherTrans = CandTrans[j];
                auto OtherAsInput = OtherTrans->As<LTSSymbInputTransition>();
                
                if (MType != ExprTypeRef::NullPtr && OtherAsInput != nullptr) {
                    if (OtherAsInput->GetMessageType() != MType) {
                        continue;
                    }
                }
                
                auto Disjoint = 
                    CheckDisjoint(Trans->GetGuard(), OtherTrans->GetGuard(), TP);
                if (!Disjoint) {
                    throw ESMCError((string)"Determinism Check failed on EFSM \"" + 
                                    Name + "\"\nOn Transitions:\n" + Trans->ToString() + 
                                    "\n\nand:\n\n" + OtherTrans->ToString());
                }
            }
        }


        // Overriden to check determinism
        void DetEFSM::Freeze()
        {
            EFSMBase::Freeze();

            // Check for determinism
            auto TP = TheoremProver::MakeProver<Z3TheoremProver>();

            for (auto const& NameState : States) {
                auto const& StateName = NameState.first;
                auto&& CandTransitions = 
                    GetSymbolicTransitions([&] (const LTSSymbTransRef& Trans) -> bool
                                           {
                                               return (Trans->GetInitState().GetName() ==
                                                       StateName);
                                           });
                for (u32 i = 0; i < CandTransitions.size(); ++i) {
                    CheckTransition(TP, i, CandTransitions);
                }
            }
        }

        // IncompleteEFSM implementation
        IncompleteEFSM::IncompleteEFSM(LabelledTS* TheLTS, const string& Name,
                                       const vector<ExpT>& Params, 
                                       const ExpT& Constraint,
                                       LTSFairnessType Fairness)
            : DetEFSM(TheLTS, Name, Params, Constraint, Fairness)
        {
            // Nothing here
        }

        IncompleteEFSM::~IncompleteEFSM()
        {
            // Nothing here
        }

        void IncompleteEFSM::AddVariable(const string& VarName, const ExprTypeRef& VarType)
        {
            DetEFSM::AddVariable(VarName, VarType);
            UpdateableVariables[VarName] = VarType;
            AllVariables[VarName] = VarType;
        }

        void IncompleteEFSM::IgnoreMsgOnState(const SymmMsgDeclRef& MsgDecl,
                                              const string& StateName)
        {
            if (States.find(StateName) == States.end()) {
                throw ESMCError((string)"State named \"" + StateName + "\" is " + 
                                "not a valid state for EFSM \"" + Name + "\"" + 
                                " in call to IncompleteEFSM::IgnoreMsgOnState()");
            }
            if (find(SymmetricMessages.begin(), SymmetricMessages.end(), MsgDecl) ==
                SymmetricMessages.end()) {
                throw ESMCError((string)"Symmetric message undeclared as input " + 
                                "or output to EFSM in call to " + 
                                "IncompleteEFSM::IgnoreMsgOnState()");
            }
            BlockedCompletions[StateName].insert(MsgDecl);
        }

        void IncompleteEFSM::MarkStateComplete(const string& StateName)
        {
            if (States.find(StateName) == States.end()) {
                throw ESMCError((string)"State named \"" + StateName + "\" is " + 
                                "not a valid state for EFSM \"" + Name + "\"" + 
                                " in call to IncompleteEFSM::IgnoreMsgOnState()");
            }
            CompleteStates.insert(StateName);
        }

        void IncompleteEFSM::MarkVariableReadOnly(const string& VarName)
        {
            auto const& STEntry = SymTab.Lookup(VarName);
            if (STEntry == DeclRef::NullPtr || !STEntry->Is<VarDecl>()) {
                throw ESMCError((string)"Object named \"" + VarName + "\" is not " +
                                "a variable of the EFSM named \"" + Name + "\"" + 
                                " in call to IncompleteEFSM::MarkVariableReadOnly()");
            }
            ReadOnlyVars.insert(VarName);
            UpdateableVariables.erase(VarName);
        }

        void IncompleteEFSM::ExpandExpression(const ExpT& Exp, set<ExpT>& Expansions)
        {
            auto VarType = Exp->GetType();
            auto Mgr = TheLTS->GetMgr();
            if (VarType->Is<Exprs::ExprScalarType>()) {
                Expansions.insert(Exp);
                return;
            }

            if (VarType->Is<Exprs::ExprRecordType>()) {
                auto TypeAsRec = VarType->SAs<Exprs::ExprRecordType>();
                auto const& Fields = TypeAsRec->GetMemberVec();
                auto FAType = TheLTS->MakeFieldAccessType();

                for (auto const& Field : Fields) {
                    auto FAVar = TheLTS->MakeVar(Field.first, FAType);
                    auto CurExpansion = TheLTS->MakeOp(LTSOps::OpField,
                                                       Exp, FAVar);
                    ExpandExpression(CurExpansion, Expansions);
                }
                return;
            }
            
            if (VarType->Is<Exprs::ExprArrayType>()) {
                auto TypeAsArray = VarType->SAs<Exprs::ExprArrayType>();
                auto const& IndexType = TypeAsArray->GetIndexType();
                auto const& IndexElems = IndexType->GetElements();
                
                for (auto const& IndexElem : IndexElems) {
                    auto IndexExp = Mgr->MakeVal(IndexElem, IndexType);
                    auto CurExpansion = TheLTS->MakeOp(LTSOps::OpIndex,
                                                       Exp, IndexExp);
                    ExpandExpression(CurExpansion, Expansions);
                }
                return;
            }

            // Eh?
            return;
        }

        set<ExpT> IncompleteEFSM::GetDomainTerms(const map<string, ExprTypeRef>& DomainVars)
        {
            set<ExpT> Retval;
            for (auto const& Var : DomainVars) {
                auto VarExp = TheLTS->MakeVar(Var.first, Var.second);
                ExpandExpression(VarExp, Retval);
            }
            return Retval;
        }

        // Find the negation of the disjunction of the guards
        // which is also in the region defined by RegionConstraint
        inline ExpT 
        IncompleteEFSM::FindNegDisjunction(const vector<LTSSymbTransRef>& Transitions, 
                                           const TPRef& TP, 
                                           const ExpT& RegionConstraint)
        {
            vector<ExpT> Guards = { TheLTS->MakeFalse() };
            ExpT CoveredGuard = ExpT::NullPtr;

            for (auto const& Trans : Transitions) {
                Guards.push_back(Trans->GetGuard());
            }

            if (Guards.size() > 1) {
                CoveredGuard = TheLTS->MakeOp(LTSOps::OpOR, Guards);
            } else {
                CoveredGuard = Guards[0];
            }
            auto NegCovered = TheLTS->MakeOp(LTSOps::OpNOT, CoveredGuard);
            NegCovered = TheLTS->MakeOp(LTSOps::OpAND, NegCovered, RegionConstraint);
            auto Res = TP->CheckSat(NegCovered);
            if (Res == TPResult::SATISFIABLE) {
                return NegCovered;
            } else if (Res == TPResult::UNSATISFIABLE) {
                return TheLTS->MakeFalse();
            } else {
                throw IncompleteTheoryException(NegCovered);
            }
        }

        // Find the predicate left uncovered by 
        // input transition on MsgType
        inline ExpT
        IncompleteEFSM::FindInputUncoveredRegion(const vector<LTSSymbTransRef>& Transitions,
                                                 const TPRef& TP, 
                                                 const ExprTypeRef& MsgType,
                                                 const ExpT& RegionConstraint)
        {
            auto&& RelTransitions = 
                Filter<LTSSymbTransRef>(Transitions.begin(), 
                                        Transitions.end(),
                                        [&] (const LTSSymbTransRef& Trans) -> bool 
                                        {
                                            auto TransAsInput = 
                                                Trans->As<LTSSymbInputTransition>();
                                            return (TransAsInput != nullptr &&
                                                    TransAsInput->GetMessageType() == MsgType);
                                        });

            return FindNegDisjunction(RelTransitions, TP, RegionConstraint);
        }

        // Find the predicate left uncovered
        // globally, i.e., by the output and internal
        // transitions
        inline ExpT 
        IncompleteEFSM::FindGlobalUncoveredRegion(const vector<LTSSymbTransRef>& Transitions,
                                                  const TPRef& TP)
        {
            auto&& RelTransitions = 
                Filter<LTSSymbTransRef>(Transitions.begin(),
                                        Transitions.end(),
                                        [&] (const LTSSymbTransRef& Trans) -> bool
                                        {
                                            return (Trans->Is<LTSSymbInternalTransition>() || 
                                                    Trans->Is<LTSSymbOutputTransition>());
                                        });

            return FindNegDisjunction(RelTransitions, TP, TheLTS->MakeTrue());
        }

        inline vector<ExpT> IncompleteEFSM::GetSymmetryConstraints(const ExpT& Exp)
        {
            using namespace Symm;

            auto ExpAsOp = Exp->As<OpExpression>();
            if (ExpAsOp == nullptr) {
                throw InternalError((string)"Expected an uninterpreted function " + 
                                    "application as expression argument in " + 
                                    "IncompleteEFSM::GetSymmetryConstraints()\n" + 
                                    "At: " + __FILE__ + ":" + to_string(__LINE__));
            }
            auto const& AppArgs = ExpAsOp->GetChildren();
            vector<ExprTypeRef> ArgTypes;
            for_each(AppArgs.begin(), AppArgs.end()
                     [&] (const ExpT& Arg) -> void
                     {
                         ArgTypes.push_back(AppArgs->GetType());
                     });

            vector<ExpT> Retval;
            auto const IsRangeSymmetric = Exp->GetType()->Is<ExprSymmetricType>();

            auto Mgr = TheLTS->GetMgr();

            // Get all the terms which are of symmetric type
            auto&& SymmTerms = 
                Mgr->Gather(Exp,
                            [&] (const ExpT& Exp) -> bool
                            {
                                auto const& ExpType = Exp->GetType();
                                return (ExpType->Is<ExprSymmetricType>());
                            });

            set<ExprTypeRef> SymmTypeSet;
            for_each(SymmTerms.begin(), SymmTerms.end(),
                     [&] (const ExpT& Term) -> void
                     {
                         SymmTypeSet.insert(Term->GetType());
                     });

            if (IsRangeSymmetric) {
                SymmTypeSet.insert(Exp->GetType());
            }

            map<ExprTypeRef, u32> TypeOffsets;
            vector<u32> DomainSizes;
            
            // Run offset, run!!
            u32 RunningOffset = 0;
            for (auto const& SymmType : SymmTypeSet) {
                auto const CurSize = SymmType->GetCardinality();
                DomainSizes.push_back(CurSize);
                TypeOffsets[SymmType] = RunningOffset;
                RunningOffset += CurSize;
            }

            PermutationSet PermSet(DomainSizes, false);

            

            return Retval;
        }

        inline ExpT IncompleteEFSM::MakeGuard(const set<ExpT>& DomainTerms,
                                              const ExpT& UncoveredPred, 
                                              const vector<ExpT>& GuardExps)
        {
            using namespace ESMC::Symm;

            auto Mgr = TheLTS->GetMgr();
            vector<ExpT> DomainTermVec(DomainTerms.begin(), DomainTerms.end());
            vector<ExprTypeRef> DomainTypes;

            for_each(DomainTerms.begin(), DomainTerms.end(),
                     [&] (const ExpT& DomainTerm) -> void
                     {
                         DomainTypes.push_back(DomainTerm->GetType());
                     });

            // Register a new uninterpreted function
            string GuardUFName = 
                (string)"SynGuard_" + this->Name + "_" + to_string(UFUIDGen.GetUID());
            auto GuardOp = Mgr->MakeUninterpretedFunction(GuardUFName, 
                                                          DomainTypes,
                                                          Mgr->MakeType<ExprBoolType>());
            auto GuardExp = Mgr->MakeExpr(GuardOp, DomainTermVec);
            // Make the constraints for symmetry on the guard expression
            auto&& SymmConstraints = GetSymmetryConstraints(GuardExp);
            
        }

        inline vector<LTSAssignRef> 
        IncompleteEFSM::MakeUpdates(const set<ExpT>& DomainTerms)
        {
            vector<LTSAssignRef> Retval;
            return Retval;
        }

        inline void 
        IncompleteEFSM::CompleteOneInputTransition(const string& InitStateName, 
                                                   const string& FinalStateName,
                                                   const SymmMsgDeclRef& MsgDecl,
                                                   const map<string, ExprTypeRef>& DomainVars, 
                                                   vector<ExpT>& GuardExps, 
                                                   const ExpT& UncoveredPred)
        {
            auto LocalDomVars = DomainVars;
            auto const& NewParams = MsgDecl->GetNewParams();
            for (auto const& Param : NewParams) {
                auto ParamAsVar = Param->As<VarExpression>();
                auto const& ParamName = ParamAsVar->GetVarName();
                LocalDomVars[ParamName] = ParamAsVar->GetVarType();
            }
            // Add the input message
            auto const& MsgType = MsgDecl->GetMessageType();
            ExprTypeRef ActMsgType = ExprTypeRef::NullPtr;
            if (MsgType->Is<ExprParametricType>()) {
                ActMsgType = MsgType->SAs<ExprParametricType>()->GetBaseType();
            } else {
                ActMsgType = MsgType;
            }

            LocalDomVars["InMsg"] = MsgDecl->GetMessageType();

            // Get the domain terms
            auto&& DomainTerms = GetDomainTerms(LocalDomVars);

            auto GuardExp = MakeGuard(DomainTerms, UncoveredPred, GuardExps);
            auto&& Updates = MakeUpdates(DomainTerms);

            if (NewParams.size() == 0) {
                AddInputTransition(InitStateName, FinalStateName, GuardExp, 
                                   Updates, "InMsg", MsgDecl->GetMessageType(),
                                   MsgDecl->GetMessageParams());
            } else {
                AddInputTransitions(NewParams, MsgDecl->GetConstraint(), 
                                    InitStateName, FinalStateName, 
                                    GuardExp, Updates, "InMsg", 
                                    MsgDecl->GetMessageType(), 
                                    MsgDecl->GetMessageParams());
            }
        }

        inline void 
        IncompleteEFSM::CompleteInputTransitions(const string& StateName,
                                                 const vector<LTSSymbTransRef>& TransFromState,
                                                 const ExpT& RegionConstraint, const TPRef& TP)
        {
            for (auto const& MsgDecl : SymmetricMessages) {
                if (!MsgDecl->IsInput()) {
                    continue;
                }
                if (CompleteStates.find(StateName) != CompleteStates.end()) {
                    continue;
                }
                auto it = BlockedCompletions.find(StateName);
                if (it != BlockedCompletions.end() &&
                    it->second.find(MsgDecl) != it->second.end()) {
                    continue;
                }

                // We need to add transitions
                // find the predicate already covered
                // for this state on this input message
                auto UncoveredMsgPred = FindInputUncoveredRegion(TransFromState, TP, 
                                                                 MsgDecl->GetMessageType(),
                                                                 RegionConstraint);
                if (UncoveredMsgPred == TheLTS->MakeFalse()) {
                    continue;
                }

                // We're okay to add one or more transitions
                // Gather the list of expresions that could be 
                // arguments to the uninterpreted functions
                map<string, ExprTypeRef> DomainVariables;

                for (auto const& Var : AllVariables) {
                    DomainVariables.insert(Var);
                }
                // add in the message params to the domain vars
                auto const& NewMsgParams = MsgDecl->GetNewParams();
                for (auto const& NewMsgParam : NewMsgParams) {
                    auto ParamAsVar = NewMsgParam->As<Exprs::VarExpression>();
                    assert(ParamAsVar != nullptr);
                    DomainVariables[ParamAsVar->GetVarName()] = ParamAsVar->GetVarType();
                }

                vector<ExpT> GuardExps;

                // The target can be any state
                for (auto const& NameState : States) {
                    cout << "Completing input transition on state \"" 
                         << StateName << "\" on message type:" << endl
                         << MsgDecl->ToString() << endl
                         << "to state \"" << NameState.first << "\" on EFSM "
                         << this->Name << endl;
                        
                    CompleteOneInputTransition(StateName, NameState.first, 
                                               MsgDecl, DomainVariables, 
                                               GuardExps, UncoveredMsgPred);
                }
            }
        }

        void IncompleteEFSM::Freeze()
        {
            if (EFSMFrozen) {
                return;
            }
            // Check for determinism first
            DetEFSM::Freeze();
            // Thaw it
            EFSMFrozen = false;

            auto TP = TheoremProver::MakeProver<Z3TheoremProver>();
            vector<ExpT> AddedGuards;
            
            // for each state
            for (auto const& NameState : States) {
                auto const& StateName = NameState.first;
                auto&& TransFromState = 
                    GetSymbolicTransitions([&] (const LTSSymbTransRef& Trans) -> bool
                                           {
                                               return (Trans->GetInitState().GetName() ==
                                                       StateName);
                                           });
                auto UncoveredPred = FindGlobalUncoveredRegion(TransFromState, TP);
                if (UncoveredPred == TheLTS->MakeFalse()) {
                    continue;
                }
                
                // Add transitions
                CompleteInputTransitions(StateName, TransFromState, UncoveredPred, TP);
            }
            
            EFSMFrozen = true;
        }
        
    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// LTSEFSM.cpp ends here
