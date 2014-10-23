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
                throw IncompleteTheoryException(Conjunction);
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

         inline void IncompleteEFSM::AddConstraint(const ExpT& Constraint)
         {
             auto Mgr = TheLTS->GetMgr();
             auto SimpConstraint = Mgr->Simplify(Constraint);
             cout << "Adding Constraint:" << endl << SimpConstraint->ToString() << endl;
             Constraints.insert(SimpConstraint);
         }

         inline void IncompleteEFSM::AddConstraint(const vector<ExpT>& Constraints)
         {
             auto Mgr = TheLTS->GetMgr();
             cout << "Adding Constraints:" << endl;
             for (auto const& Constraint : Constraints) {
                 auto SimpConstraint = Mgr->Simplify(Constraint);
                 cout << SimpConstraint->ToString() << endl << endl;
                 this->Constraints.insert(SimpConstraint);
             }
             cout << "End of constraint set." << endl;
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
         IncompleteEFSM::FindDisjunction(const vector<LTSSymbTransRef>& Transitions, 
                                         const TPRef& TP, 
                                         const ExpT& CoveredRegion)
         {
             vector<ExpT> Guards = { CoveredRegion };
             ExpT CoveredGuard = ExpT::NullPtr;
             auto Mgr = TheLTS->GetMgr();

             for (auto const& Trans : Transitions) {
                 auto const& NewParams = Trans->GetTransParams();
                 const u32 NumNewParams = NewParams.size();
                 if (NumNewParams == 0) {
                     Guards.push_back(Trans->GetGuard());
                 } else {
                     vector<ExprTypeRef> NewParamTypes(NumNewParams);
                     MgrT::SubstMapT SubstMap;
                     for (u32 i = 0; i < NumNewParams; ++i) {
                         auto const& CurParam = NewParams[i];
                         auto const& CurType = CurParam->GetType();
                         NewParamTypes[NumNewParams - i - 1] = CurType;
                         SubstMap[NewParams[i]] = Mgr->MakeBoundVar(CurType, i);
                     }
                     auto ConstrainedGuard = Mgr->MakeExpr(LTSOps::OpAND, 
                                                           Trans->GetGuard(),
                                                           Trans->GetConstraint());

                     auto SubstGuard = Mgr->BoundSubstitute(SubstMap, ConstrainedGuard);
                     Guards.push_back(Mgr->MakeExists(NewParamTypes, SubstGuard));
                 }
             }

             if (Guards.size() > 1) {
                 CoveredGuard = TheLTS->MakeOp(LTSOps::OpOR, Guards);
             } else {
                 CoveredGuard = Guards[0];
             }

             auto NegCovered = TheLTS->MakeOp(LTSOps::OpNOT, CoveredGuard);

             auto Res = TP->CheckSat(NegCovered);
             if (Res == TPResult::SATISFIABLE) {
                 return CoveredGuard;
             } else if (Res == TPResult::UNSATISFIABLE) {
                 return TheLTS->MakeTrue();
             } else {
                 throw IncompleteTheoryException(NegCovered);
             }
         }

         // Find the predicate left uncovered by 
         // input transition on MsgType
         inline ExpT
         IncompleteEFSM::FindInputCoveredRegion(const vector<LTSSymbTransRef>& Transitions,
                                                const TPRef& TP, 
                                                const ExprTypeRef& MsgType,
                                                const ExpT& CoveredRegion)
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

            return FindDisjunction(RelTransitions, TP, CoveredRegion);
        }

        // Find the predicate left uncovered
        // globally, i.e., by the output and internal
        // transitions
        inline ExpT 
        IncompleteEFSM::FindGlobalCoveredRegion(const vector<LTSSymbTransRef>& Transitions,
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

            return FindDisjunction(RelTransitions, TP, TheLTS->MakeFalse());
        }

        inline vector<ExprTypeRef>
        IncompleteEFSM::GetSymmTypesInExpr(const ExpT& Exp)
        {
            auto Mgr = TheLTS->GetMgr();

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
            if (Exp->GetType()->Is<ExprSymmetricType>()) {
                SymmTypeSet.insert(Exp->GetType());
            }

            return vector<ExprTypeRef>(SymmTypeSet.begin(), SymmTypeSet.end());
        }

        inline void IncompleteEFSM::PartitionDomain(const vector<ExpT>& Args, 
                                                    vector<ExpT>& SymmArgs, 
                                                    vector<ExpT>& NonSymmArgs)
        {
            SymmArgs.clear();
            NonSymmArgs.clear();

            for_each(Args.begin(), Args.end(),
                     [&] (const ExpT& Arg) -> void
                     {
                         auto const& ArgType = Arg->GetType();
                         if (ArgType->Is<ExprSymmetricType>()) {
                             SymmArgs.push_back(Arg);
                         } else {
                             NonSymmArgs.push_back(Arg);
                         }
                     });
        }

        inline void
        IncompleteEFSM::MergeEquivalences(const set<ExpT>& NewEquivalences, 
                                          set<set<ExpT>>& EquivalenceSets)
        {
            vector<set<set<ExpT>>::const_iterator> ToMerge;
            set<ExpT> MergedSet(NewEquivalences.begin(), NewEquivalences.end());

            for (auto const& Exp : NewEquivalences) {
                for (auto it = EquivalenceSets.begin(); 
                     it != EquivalenceSets.end(); ++it) {
                    auto const& CurSet = *it;
                    if (CurSet.find(Exp) != CurSet.end()) {
                        if (find(ToMerge.begin(), ToMerge.end(), it) == ToMerge.end()) {
                            ToMerge.push_back(it);
                        }
                    }
                }
            }

            for (auto const& IteratorToMerge : ToMerge) {
                auto const& SetToMerge = *IteratorToMerge;
                MergedSet.insert(SetToMerge.begin(), SetToMerge.end());
            }
            
            for (auto const& IteratorToMerge : ToMerge) {
                EquivalenceSets.erase(IteratorToMerge);
            }
            EquivalenceSets.insert(MergedSet);
        }

        inline set<set<ExpT>> 
        IncompleteEFSM::FindEquivalences(const ExpT& Exp,
                                         const vector<ExprTypeRef>& SymmTypes,
                                         const vector<ExpT>& SymmArgs,
                                         const vector<ExpT>& NonSymmArgs)
        {
            using namespace Symm;

            map<ExprTypeRef, u32> TypeOffsets;
            vector<u32> DomainSizes;
            auto const IsRangeSymmetric = Exp->GetType()->Is<ExprSymmetricType>();
            set<set<ExpT>> Retval;
            const u32 NumSymmArgs = SymmArgs.size();
            auto Mgr = TheLTS->GetMgr();
            
            // Run offset, run!!
            u32 RunningOffset = 0;
            for (auto const& SymmType : SymmTypes) {
                auto const CurSize = SymmType->GetCardinality();
                DomainSizes.push_back(CurSize);
                TypeOffsets[SymmType] = RunningOffset;
                RunningOffset += CurSize;
            }

            PermutationSet PermSet(DomainSizes, false);
            const u32 NumPerms = PermSet.GetSize();
            
            // Get the cross product of values for symmetric arguments
            vector<vector<string>> SymmArgValues;
            for (auto const& SymmArg : SymmArgs) {
                SymmArgValues.push_back(SymmArg->GetType()->GetElements());
                // Add the clear value if arg is an lvalue
                if (SymmArg->Is<Exprs::VarExpression>()) {
                    auto ArgAsVar = SymmArg->SAs<Exprs::VarExpression>();
                    auto const& VarName = ArgAsVar->GetVarName();
                    auto Res = SymTab.Lookup(VarName);
                    if (Res->Is<ParamDecl>()) {
                        continue;
                    } else {
                        SymmArgValues.back().push_back("clear");
                    }
                }
            }
            auto&& ValueTuples = CrossProduct<string>(SymmArgValues.begin(), 
                                                      SymmArgValues.end());
            
            // for each value tuple, we substitute args with
            // the values, and then compute the equivalences
            // based on the permutations
            for (auto const& ValueTuple : ValueTuples) {
                MgrT::SubstMapT SubstMap;

                for (u32 i = 0; i < NumSymmArgs; ++i) {
                    SubstMap[SymmArgs[i]] = Mgr->MakeVal(ValueTuple[i], 
                                                         SymmArgs[i]->GetType());
                }

                set<ExpT> CurEquivalences;

                if (!IsRangeSymmetric) {
                    auto SubstExp = Mgr->TermSubstitute(SubstMap, Exp);
                    for (u32 i = 0; i < NumPerms; ++i) {
                        auto const& CurPerm = PermSet.GetIterator(i).GetPerm();
                        
                        auto PermExp = Mgr->ApplyTransform<ExpressionPermuter>(SubstExp, 
                                                                               CurPerm,
                                                                               TypeOffsets);
                        CurEquivalences.insert(PermExp);
                    }
                } else {
                    // The range itself is symmetric
                    auto const& RangeType = Exp->GetType();
                    auto RangeElems = RangeType->GetElements();
                    RangeElems.push_back("clear");
                    
                    for (auto const& RangeElem : RangeElems) {
                        auto CurVal = Mgr->MakeVal(RangeElem, RangeType);
                        auto EQExp = Mgr->MakeExpr(LTSOps::OpEQ, Exp, CurVal);
                        auto SubstExp = Mgr->TermSubstitute(SubstMap, EQExp);
                        for (u32 i = 0; i < NumPerms; ++i) {
                            auto const& CurPerm = PermSet.GetIterator(i).GetPerm();
                            auto PermExp = Mgr->ApplyTransform<ExpressionPermuter>(SubstExp,
                                                                                   CurPerm,
                                                                                   TypeOffsets);
                            CurEquivalences.insert(PermExp);
                        }
                    }
                }

                MergeEquivalences(CurEquivalences, Retval);
            }
            return Retval;
        }

        static inline ExpT MakeEquivalence(const ExpT& Exp1, const ExpT& Exp2,
                                           MgrT* Mgr)
        {
            if (Exp1->GetType() != Exp2->GetType()) {
                throw InternalError((string)"Types do not match when attempting to " +
                                    "assert equivalence of expressions:\n" + 
                                    Exp1->ToString() + "\nand:\n" + Exp2->ToString() + 
                                    "\nAt: " + __FILE__ + ":" + to_string(__LINE__));
            }
            
            if (Exp1->GetType()->Is<ExprBoolType>()) {
                return Mgr->MakeExpr(LTSOps::OpIFF, Exp1, Exp2);
            } else {
                return Mgr->MakeExpr(LTSOps::OpEQ, Exp1, Exp2);
            }
        }

        inline vector<ExpT> IncompleteEFSM::GetSymmetryConstraints(const ExpT& Exp)
        {
            vector<ExpT> Retval;
            auto Mgr = TheLTS->GetMgr();

            auto ExpAsOp = Exp->As<OpExpression>();
            if (ExpAsOp == nullptr) {
                throw InternalError((string)"Expected an uninterpreted function " + 
                                    "application as expression argument in " + 
                                    "IncompleteEFSM::GetSymmetryConstraints()\n" + 
                                    "At: " + __FILE__ + ":" + to_string(__LINE__));
            }
            auto const& AppArgs = ExpAsOp->GetChildren();
            vector<ExpT> SymmArgs;
            vector<ExpT> NonSymmArgs;
            PartitionDomain(AppArgs, SymmArgs, NonSymmArgs);

            const u32 NumNonSymmArgs = NonSymmArgs.size();
            auto&& RelevantSymmTypes = GetSymmTypesInExpr(Exp);
            auto&& Equivalences = FindEquivalences(Exp, RelevantSymmTypes, 
                                                   SymmArgs, NonSymmArgs);
            
            for (auto const& EqClass : Equivalences) {
                auto const& Representative = *(EqClass.begin());
                for (auto it = next(EqClass.begin()); it != EqClass.end(); ++it) {
                    auto CurConstraint = MakeEquivalence(Representative, *it, Mgr);
                        
                    // Quantify the non symmetric args
                    MgrT::SubstMapT SubstMap;
                    vector<ExprTypeRef> QVarTypes(NumNonSymmArgs);
                        
                    for (u32 i = 0; i < NumNonSymmArgs; ++i) {
                        auto const& ArgType = NonSymmArgs[i]->GetType();
                        SubstMap[NonSymmArgs[i]] = 
                            Mgr->MakeBoundVar(ArgType, i);
                        QVarTypes[NumNonSymmArgs - i - 1] = ArgType;
                    }

                    auto QBody = Mgr->BoundSubstitute(SubstMap, CurConstraint);
                    Retval.push_back(Mgr->MakeForAll(QVarTypes, QBody));
                }
            }

            return Retval;
        }

        inline ExpT IncompleteEFSM::MakeGuard(const set<ExpT>& DomainTerms,
                                              const ExpT& CoveredPred, 
                                              const vector<ExpT>& GuardExps)
        {
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
                (string)"SynGuard_" + this->Name + "_" + to_string(GuardUFUIDGen.GetUID());
            auto GuardOp = Mgr->MakeUninterpretedFunction(GuardUFName, 
                                                          DomainTypes,
                                                          Mgr->MakeType<ExprBoolType>());
            auto GuardExp = Mgr->MakeExpr(GuardOp, DomainTermVec);
            // Make the constraints for symmetry on the guard expression
            auto&& SymmConstraints = GetSymmetryConstraints(GuardExp);
            cout << "Symmetry constraints:" << endl;
            AddConstraint(SymmConstraints);
            
            // Make the determinism constraints
            vector<ExpT> DetConstraints;
            // The intersection of the guard with the covered region
            // must be empty
            auto GuardAndCovered = Mgr->MakeExpr(LTSOps::OpAND, GuardExp, CoveredPred);
            DetConstraints.push_back(Mgr->MakeExpr(LTSOps::OpNOT, GuardAndCovered));

            for (auto const& OtherGuard : GuardExps) {
                auto Conjunction = Mgr->MakeExpr(LTSOps::OpAND, GuardExp,
                                                 OtherGuard);
                DetConstraints.push_back(Mgr->MakeExpr(LTSOps::OpNOT, Conjunction));
            }

            ExpT FinalDetConstraint = ExpT::NullPtr;
            if (DetConstraints.size() == 0) {
                FinalDetConstraint = Mgr->MakeTrue();
            } else if (DetConstraints.size() == 1) {
                FinalDetConstraint = DetConstraints[0];
            } else {
                FinalDetConstraint = Mgr->MakeExpr(LTSOps::OpAND, DetConstraints);
            }

            // Quantify on all the domain terms
            MgrT::SubstMapT SubstMap;
            const u32 NumDomainTerms = DomainTermVec.size();
            vector<ExprTypeRef> QVarTypes(DomainTypes.rbegin(), DomainTypes.rend());
            for (u32 i = 0; i < NumDomainTerms; ++i) {
                auto const& DomTerm = DomainTermVec[i];
                auto const& DomTermType = DomainTypes[i];
                SubstMap[DomTerm] = Mgr->MakeBoundVar(DomTermType, i);
            }

            auto BodyExp = Mgr->BoundSubstitute(SubstMap, FinalDetConstraint);
            auto QExp = Mgr->MakeForAll(QVarTypes, BodyExp);
            cout << "Determinism Constraint:" << endl;
            AddConstraint(QExp);
            return GuardExp;
        }

        inline vector<LTSAssignRef> 
        IncompleteEFSM::MakeUpdates(const set<ExpT>& DomainTerms)
        {
            auto Mgr = TheLTS->GetMgr();
            vector<LTSAssignRef> Retval;
            vector<ExprTypeRef> DomainTypes;
            vector<ExpT> DomainTermVec(DomainTerms.begin(), DomainTerms.end());
            for_each(DomainTerms.begin(), DomainTerms.end(),
                     [&] (const ExpT& DomainTerm) -> void
                     {
                         DomainTypes.push_back(DomainTerm->GetType());
                     });

            map<string, ExprTypeRef> LValues;
            for (auto const& Var : UpdateableVariables) {
                LValues.insert(Var);
            }

            auto&& LValueTerms = GetDomainTerms(LValues);
            // Add the state lvalue
            LValueTerms.insert(TheLTS->MakeVar("state", StateType));

            for (auto const& LValue : LValueTerms) {
                // Register a new uninterpreted function
                // for the update of each domain term
                string UpdateUFName = 
                    (string)"Update_" + this->Name + "_" + to_string(UpdateUFUIDGen.GetUID());
                auto UpdateOp = Mgr->MakeUninterpretedFunction(UpdateUFName,
                                                               DomainTypes,
                                                               LValue->GetType());
                auto UpdateExp = Mgr->MakeExpr(UpdateOp, DomainTermVec);
                auto&& SymmConstraints = GetSymmetryConstraints(UpdateExp);
                cout << "Symmetry constraints for update of term " << LValue->ToString() 
                     << ":" << endl;
                AddConstraint(SymmConstraints);

                Retval.push_back(new LTSAssignSimple(LValue, UpdateExp));
            }
            return Retval;
        }

        inline void 
        IncompleteEFSM::CompleteOneInputTransition(const string& InitStateName, 
                                                   const SymmMsgDeclRef& MsgDecl,
                                                   const map<string, ExprTypeRef>& DomainVars, 
                                                   vector<ExpT>& GuardExps, 
                                                   const ExpT& CoveredPred)
        {
            auto LocalDomVars = DomainVars;
            auto const& NewParams = MsgDecl->GetNewParams();
            for (auto const& Param : NewParams) {
                auto ParamAsVar = Param->As<VarExpression>();
                auto const& ParamName = ParamAsVar->GetVarName();
                LocalDomVars[ParamName] = ParamAsVar->GetVarType();
            }

            // Get the domain terms
            auto&& DomainTerms = GetDomainTerms(LocalDomVars);

            auto GuardExp = MakeGuard(DomainTerms, CoveredPred, GuardExps);
            GuardExps.push_back(GuardExp);

            // Add the input message for the updates
            auto const& MsgType = MsgDecl->GetMessageType();
            ExprTypeRef ActMsgType = ExprTypeRef::NullPtr;
            if (MsgType->Is<ExprParametricType>()) {
                ActMsgType = MsgType->SAs<ExprParametricType>()->GetBaseType();
            } else {
                ActMsgType = MsgType;
            }

            LocalDomVars["InMsg"] = MsgDecl->GetMessageType();

            DomainTerms = GetDomainTerms(LocalDomVars);

            // Make the updates
            auto&& Updates = MakeUpdates(DomainTerms);

            if (NewParams.size() == 0) {
                AddInputTransition(InitStateName, GuardExp, 
                                   Updates, "InMsg", MsgDecl->GetMessageType(),
                                   MsgDecl->GetMessageParams());
            } else {
                AddInputTransitions(NewParams, MsgDecl->GetConstraint(), 
                                    InitStateName,
                                    GuardExp, Updates, "InMsg", 
                                    MsgDecl->GetMessageType(), 
                                    MsgDecl->GetMessageParams());
            }
        }

        inline void 
        IncompleteEFSM::CompleteInputTransitions(const string& StateName,
                                                 const vector<LTSSymbTransRef>& TransFromState,
                                                 const ExpT& CoveredRegion, const TPRef& TP)
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

                auto const& MType = MsgDecl->GetMessageType();
                ExprTypeRef ActMType = ExprTypeRef::NullPtr;
                if (MType->Is<ExprParametricType>()) {
                    ActMType = MType->SAs<ExprParametricType>()->GetBaseType();
                } else {
                    ActMType = MType->SAs<ExprRecordType>();
                }

                // We need to add transitions
                // find the predicate already covered
                // for this state on this input message
                auto CoveredMsgPred = FindInputCoveredRegion(TransFromState, TP, 
                                                             MsgDecl->GetMessageType(),
                                                             CoveredRegion);
                if (CoveredMsgPred == TheLTS->MakeTrue()) {
                    cout << "Nothing to complete on state " << StateName
                         << " and message " << MsgDecl->ToString() << endl;
                    continue;
                }

                cout << "Completing transition on MessageType "
                     << ActMType->SAs<ExprRecordType>()->GetName() 
                     << " on state " << StateName << endl;
                    

                // We're okay to add one or more transitions
                // Gather the list of expresions that could be 
                // arguments to the uninterpreted functions
                map<string, ExprTypeRef> DomainVariables;

                for (auto const& Var : AllVariables) {
                    DomainVariables.insert(Var);
                }

                // Add any parameters as well
                for (auto const& Param : Params) {
                    auto const& ParamName = Param->As<VarExpression>()->GetVarName();
                    DomainVariables[ParamName] = Param->GetType();
                }
                
                // add in the message params to the domain vars
                auto const& NewMsgParams = MsgDecl->GetNewParams();
                for (auto const& NewMsgParam : NewMsgParams) {
                    auto ParamAsVar = NewMsgParam->As<Exprs::VarExpression>();
                    assert(ParamAsVar != nullptr);
                    DomainVariables[ParamAsVar->GetVarName()] = ParamAsVar->GetVarType();
                }

                vector<ExpT> GuardExps;

                CompleteOneInputTransition(StateName, 
                                           MsgDecl, DomainVariables, 
                                           GuardExps, CoveredMsgPred);
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
                auto CoveredPred = FindGlobalCoveredRegion(TransFromState, TP);
                if (CoveredPred == TheLTS->MakeTrue()) {
                    cout << "Nothing to complete on state " << StateName << endl;
                    continue;
                }
                
                // Add transitions
                CompleteInputTransitions(StateName, TransFromState, CoveredPred, TP);
            }
            
            EFSMFrozen = true;
        }
        
    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// LTSEFSM.cpp ends here
