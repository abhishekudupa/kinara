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
                                              const string &StateName)
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

        void IncompleteEFSM::ExtendDomainTerms(map<ExprTypeRef, set<ExpT>>& DomainTerms)
        {
            // first attempt to extend record types
            auto Mgr = TheLTS->GetMgr();
            bool Modified = false;
            for (auto const& DomTerm : DomainTerms) {
                auto const& TermType = DomTerm.first;
                if (!TermType->Is<Exprs::ExprRecordType>()) {
                    continue;
                }
                
                Modified = true;
                auto TypeAsRec = TermType->SAs<Exprs::ExprRecordType>();
                auto const& Fields = TypeAsRec->GetMemberVec();

                for (auto const& Field : Fields) {
                    auto FAType = TheLTS->MakeFieldAccessType();
                    auto FAVar = TheLTS->MakeVar(Field.first, FAType);
                    for (auto const& Exp : DomTerm.second) {
                        auto NewExp = TheLTS->MakeOp(LTSOps::OpField, Exp, FAVar);
                        DomainTerms[Field.second].insert(NewExp);
                    }
                }

                DomainTerms.erase(DomTerm.first);
                break;
            }

            if (Modified) {
                return;
            }

            // Now attempt to extend array types 
            // which are indexed by non-symmetric values
            for (auto const& DomTerm : DomainTerms) {
                auto const& TermType = DomTerm.first;
                if (!TermType->Is<Exprs::ExprArrayType>()) {
                    continue;
                }

                auto TypeAsArr = TermType->SAs<ExprArrayType>();
                auto IndexType = TypeAsArr->GetIndexType();
                auto ValueType = TypeAsArr->GetValueType();
                if (IndexType->Is<Exprs::ExprSymmetricType>()) {
                    continue;
                }

                Modified = true;
                auto const& IndexElems = IndexType->GetElements();
                for (auto const& IndexElem : IndexElems) {
                    auto IndexExp = TheLTS->MakeVal(IndexElem, IndexType);
                    for (auto const& Exp : DomTerm.second) {
                        auto NewExp = TheLTS->MakeOp(LTSOps::OpIndex, Exp, IndexExp);
                        DomainTerms[ValueType].insert(NewExp);
                    }
                }
                
                DomainTerms.erase(DomTerm.first);
                break;
            }

            if (Modified) {
                return;
            }

            // Finally, we extend array types which are 
            // indexed by symmetric values
            for (auto const& DomTerm : DomainTerms) {
                auto const& TermType = DomTerm.first;
                if (!TermType->Is<Exprs::ExprArrayType>()) {
                    continue;
                }

                auto TypeAsArr = TermType->SAs<ExprArrayType>();
                auto IndexType = TypeAsArr->GetIndexType();
                auto ValueType = TypeAsArr->GetValueType();
                if (!IndexType->Is<Exprs::ExprSymmetricType>()) {
                    continue;
                }
                
                // We now try to find terms of the index type
                auto it = DomainTerms.find(IndexType);
                if (it == DomainTerms.end()) {
                    continue;
                }

                auto const& IndexExps = it->second;
                for (auto const& IndexExp : IndexExps) {
                    for (auto const& Exp : DomTerm.second) {
                        // Check if the IndexExp includes Exp
                        auto&& SearchRes = 
                            Mgr->Gather(IndexExp, 
                                        [&] (const ExpBaseT* ExpPtr) -> bool
                                        {
                                            return (Exp == ExpPtr);
                                        });

                        if (SearchRes.size() > 0) {
                            throw ESMCError((string)"Unbounded expansion encountered!");
                        }

                        auto NewExp = TheLTS->MakeOp(LTSOps::OpIndex, Exp, IndexExp);
                        DomainTerms[ValueType].insert(NewExp);
                    }
                }
            }
        }
        
        void IncompleteEFSM::GetDomainTerms(const map<string, ExprTypeRef>& DomainVars, 
                                            map<ExprTypeRef, set<ExpT>>& DomainTerms)
        {
            DomainTerms.clear();
            
            for (auto const& Var : DomainVars) {
                auto VarExp = TheLTS->MakeVar(Var.first, Var.second);
                DomainTerms[Var.second].insert(VarExp);
            }

            // extend the domain terms until fixpoint or until
            // we realize that the number of terms is unbounded
            map<ExprTypeRef, set<ExpT>> OldDomainTerms;
            do {
                OldDomainTerms = DomainTerms;
                ExtendDomainTerms(DomainTerms);
            } while (OldDomainTerms != DomainTerms);

            // Finally, filter out all non-scalar types from the domain terms
            DomainTerms.clear();
            for (auto const& DomTerm : OldDomainTerms) {
                if (DomTerm.first->Is<Exprs::ExprScalarType>()) {
                    DomainTerms.insert(DomTerm);
                }
            }
            return;
        }

        // Find the predicate left uncovered by 
        // input transition on MsgType
        inline ExpT
        IncompleteEFSM::FindUncoveredPred(const vector<LTSSymbTransRef>& Transitions,
                                          const TPRef& TP, const ExprTypeRef& MsgType) const
        {
            vector<ExpT> Guards = { TheLTS->MakeFalse() };
            ExpT CoveredGuard = ExpT::NullPtr;

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

            for (auto const& Trans : RelTransitions) {
                Guards.push_back(Trans->GetGuard());
            }

            if (Guards.size() > 1) {
                CoveredGuard = TheLTS->MakeOp(LTSOps::OpOR, Guards);
            } else {
                CoveredGuard = Guards[0];
            }

            auto NegCovered = TheLTS->MakeOp(LTSOps::OpNOT, CoveredGuard);
            auto Res = TP->CheckSat(NegCovered);
            if (Res == TPResult::SATISFIABLE) {
                return NegCovered;
            } else if (Res == TPResult::UNSATISFIABLE) {
                return TheLTS->MakeFalse();
            } else {
                throw ESMCError((string)"Could not ensure sat or unsat of proposition:\n" + 
                                NegCovered->ToString() + "\n.Theory incomplete perhaps?");
            }
        }

        inline ExpT 
        IncompleteEFSM::FindUncoveredPred(const vector<LTSSymbTransRef>& Transitions,
                                          const TPRef& TP) const
        {
            vector<ExpT> SpontGuards = { TheLTS->MakeFalse() };
            ExpT CoveredGuard = ExpT::NullPtr;

            for (auto const& Trans : Transitions) {
                if (Trans->Is<LTSSymbInternalTransition>() ||
                    Trans->Is<LTSSymbOutputTransition>()) {
                    SpontGuards.push_back(Trans->GetGuard());
                }
            }

            if (SpontGuards.size() > 1) {
                CoveredGuard = TheLTS->MakeOp(LTSOps::OpOR, SpontGuards);
            } else {
                CoveredGuard = SpontGuards[0];
            }

            auto NegCovered = TheLTS->MakeOp(LTSOps::OpNOT, CoveredGuard);
            auto Res = TP->CheckSat(NegCovered);
            if (Res == TPResult::SATISFIABLE) {
                return NegCovered;
            } else if (Res == TPResult::UNSATISFIABLE) {
                return TheLTS->MakeFalse();
            } else {
                throw ESMCError((string)"Could not ensure sat or unsat of proposition:\n" + 
                                NegCovered->ToString() + "\n.Theory incomplete perhaps?");
            }
        }

        inline ExpT IncompleteEFSM::MakeGuard(const map<string, ExprTypeRef>& DomainVars,
                                              const ExpT& UncoveredPred, 
                                              const vector<ExpT>& GuardExps)
        {
            // Get the relevant domain terms first
            map<ExprTypeRef, set<ExpT>> DomainTerms;
            GetDomainTerms(DomainVars, DomainTerms);

            // Filter out any symmetric types from the domain terms
            map<ExprTypeRef, set<ExpT>> SymmetricTerms;
            for (auto const& DomTerm : DomainTerms) {
                if (DomTerm.first->Is<Exprs::ExprSymmetricType>()) {
                    SymmetricTerms.insert(DomTerm);
                }
            }
            for (auto const& SymmTerm : SymmetricTerms) {
                DomainTerms.erase(SymmTerm.first);
            }
            vector<ExpT> AppArgs;
            for (auto const& DomTerm : DomainTerms) {
                AppArgs.insert(AppArgs.end(), DomTerm.second.begin(),
                               DomTerm.second.end());
            }
            for (auto const& SymmTerm : SymmetricTerms) {
                auto const& TermSet = SymmTerm.second;
                if (TermSet.size() == 1) {
                    continue;
                }
                for (auto it1 = TermSet.begin(); it1 != TermSet.end(); ++it1) {
                    auto it2 = it1;
                    ++it2;
                    for (; it2 != TermSet.end(); ++it2) {
                        auto Exp = TheLTS->MakeOp(LTSOps::OpEQ, *it1, *it2);
                        AppArgs.push_back(Exp);
                    }
                }
            }

            // We're now ready to create the guard op
            auto GuardUID = UFUIDGen.GetUID();
            auto GuardFuncName = (string)"SynthGuard_" + to_string(GuardUID);
            vector<ExprTypeRef> DomTypes;
            
            for (auto const& AppArg : AppArgs) {
                DomTypes.push_back(AppArg->GetType());
            }

            auto GuardOp = TheLTS->MakeUF(GuardFuncName, DomTypes, TheLTS->MakeBoolType());
            auto GuardExp = TheLTS->MakeOp(GuardOp, AppArgs);

            // We're done constructing the expression for the guard
            // but we need to constrain it to ensure determinism, etc.

            // Ensure that the guard is always in the uncovered region
            auto GuardInUncovered = TheLTS->MakeOp(LTSOps::OpIMPLIES, GuardExp, UncoveredPred);
            // Ensure that the guard and the rest of the guards are disjoint
            vector<ExpT> DisjointConstraints;
            for (auto const& OtherGuard : GuardExps) {
                auto Conjunction = TheLTS->MakeOp(LTSOps::OpAND, GuardExp, OtherGuard);
                DisjointConstraints.push_back(TheLTS->MakeOp(LTSOps::OpNOT, Conjunction));
            }
            ExpT GuardDisjoint = ExpT::NullPtr;
            if (DisjointConstraints.size() == 0) {
                GuardDisjoint = TheLTS->MakeTrue();
            } else if (DisjointConstraints.size() == 1) {
                GuardDisjoint = DisjointConstraints[0];
            } else {
                GuardDisjoint = TheLTS->MakeOp(LTSOps::OpAND, DisjointConstraints);
            }
            
            auto FinalConstraint = TheLTS->MakeOp(LTSOps::OpAND, 
                                                  GuardInUncovered, 
                                                  GuardDisjoint);
            auto Mgr = TheLTS->GetMgr();
            FinalConstraint = Mgr->Simplify(FinalConstraint);

            cout << "Final Constraint:" << endl << FinalConstraint->ToString() << endl;

            // TODO: Finish up constraint
            return GuardExp;
        }

        inline vector<LTSAssignRef> 
        IncompleteEFSM::MakeUpdates(const map<string, ExprTypeRef>& DomainVars)
        {
            map<ExprTypeRef, set<ExpT>> DomainTerms;
            GetDomainTerms(DomainVars, DomainTerms);

            // Filter out any symmetric types from the domain terms
            map<ExprTypeRef, set<ExpT>> SymmetricTerms;
            for (auto const& DomTerm : DomainTerms) {
                if (DomTerm.first->Is<Exprs::ExprSymmetricType>()) {
                    SymmetricTerms.insert(DomTerm);
                }
            }
            for (auto const& SymmTerm : SymmetricTerms) {
                DomainTerms.erase(SymmTerm.first);
            }
            vector<ExpT> AppArgs;
            for (auto const& DomTerm : DomainTerms) {
                AppArgs.insert(AppArgs.end(), DomTerm.second.begin(),
                               DomTerm.second.end());
            }
            for (auto const& SymmTerm : SymmetricTerms) {
                auto const& TermSet = SymmTerm.second;
                if (TermSet.size() == 1) {
                    continue;
                }
                for (auto it1 = TermSet.begin(); it1 != TermSet.end(); ++it1) {
                    auto it2 = it1;
                    ++it2;
                    for (; it2 != TermSet.end(); ++it2) {
                        auto Exp = TheLTS->MakeOp(LTSOps::OpEQ, *it1, *it2);
                        AppArgs.push_back(Exp);
                    }
                }
            }
            
            vector<LTSAssignRef> Retval;
            // for (auto const& Var : UpdateableVariables) {

            //     vector<ExprTypeRef> DomTypes;
            //     auto LHSExp = TheLTS->MakeVar(Var.first, Var.second);
            //     auto LHSType = LHSExp->GetType();

            //     if (LHSType->Is<Exprs::ExprSymmetricType>()) {
            //         auto const& SymmVarsOfType = SymmetricVarsByType[LHSType];

            //         if (SymmVarsOfType.size() == 0) {
            //             continue;
            //         }

            //         // introduce additional boolean flags for 
            //         // the values of the arguments
            //         auto TypeElems = LHSType->GetElements();
            //         const u32 NumTypeElems = TypeElems.size();

            //         for (u32 i = 0; i < NumTypeElems; ++i) {
            //             DomTypes.push_back(TheLTS->MakeBoolType());
            //             vector<ExpT> Disjuncts;
            //             for (auto it = SymmVarsOfType.begin(); 
            //                  it != SymmVarsOfType.end(); ++it) {
            //                 Disjuncts.push_back(Mgr->MakeExpr(LTSOps::OpEQ,
            //                                                   Mgr->MakeVal(TypeElems[i], 
            //                                                                LHSType),
            //                                                   *it));
            //             }
            //             if (Disjuncts.size() == 1) {
            //                 AppArgs.push_back(Disjuncts[0]);
            //             } else {
            //                 AppArgs.push_back(Mgr->MakeExpr(LTSOps::OpOR, Disjuncts));
            //             }
            //         }
            //     }
            //     // Make a new uninterpreted function
            //     string UpdateOpName = (string)"Update_" + Var.first + "_" + 
            //         to_string(UFUIDGen.GetUID());
            //     auto UpdateOp = TheLTS->MakeUF(UpdateOpName, DomTypes, LHSType);
            //     auto RHSExp = TheLTS->MakeOp(UpdateOp, AppArgs);
            //     Retval.push_back(new LTSAssignSimple(LHSExp, RHSExp));
            // }
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
            auto GuardExp = MakeGuard(DomainVars, UncoveredPred, GuardExps);
            GuardExps.push_back(GuardExp);
            auto&& Updates = MakeUpdates(DomainVars);

            if (MsgDecl->GetNewParams().size() == 0) {
                AddInputTransition(InitStateName, FinalStateName, GuardExp, 
                                   Updates, "InMsg", MsgDecl->GetMessageType(),
                                   MsgDecl->GetMessageParams());
            }
        }

        inline void 
        IncompleteEFSM::CompleteInputTransitions(const string& StateName,
                                                 const vector<LTSSymbTransRef>& TransFromState,
                                                 const ExpT& UncoveredPredicate, const TPRef& TP)
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
                auto UncoveredMsgPred = FindUncoveredPred(TransFromState, TP, 
                                                          MsgDecl->GetMessageType());
                if (UncoveredMsgPred == TheLTS->MakeFalse()) {
                    continue;
                }

                auto ActualUncoveredPred = TheLTS->MakeOp(LTSOps::OpAND, UncoveredPredicate,
                                                          UncoveredMsgPred);
                
                auto SatRes = TP->CheckSat(ActualUncoveredPred);
                if (SatRes == TPResult::UNSATISFIABLE) {
                    continue;
                } else if (SatRes == TPResult::UNKNOWN) {
                    throw ESMCError((string)"Could not ensure sat or unsat of proposition:\n" + 
                                    ActualUncoveredPred->ToString() + "\n." + 
                                    "Perhaps the theory is incomplete?");
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
                        
                    CompleteOneInputTransition(StateName, NameState.first, MsgDecl,
                                               DomainVariables, GuardExps, ActualUncoveredPred);
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
                auto UncoveredPred = FindUncoveredPred(TransFromState, TP);
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
