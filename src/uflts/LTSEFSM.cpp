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
        using namespace Symm;

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
                         LTSFairnessType Fairness, bool CheckDeterminism)
            : EFSMBase(TheLTS, Name, Params, Constraint, Fairness), CheckDeterminism(CheckDeterminism)
        {
            // Nothing here
        }


        DetEFSM::~DetEFSM()
        {
            // Nothing here
        }

        static inline bool CheckDisjoint(const ExpT& Exp1, const ExpT& Exp2,
                                         const Z3TPRef& TP)
        {
            auto Mgr = Exp1->GetMgr();
            auto Conjunction = Mgr->MakeExpr(LTSOps::OpAND, Exp1, Exp2);

            ESMC_LOG_FULL(
                          "IncompleteEFSM.Assertions",
                          Out_ << "Checking for unsat:" << endl
                               << Conjunction << endl;
                          );

            auto Res = TP->CheckSat(Conjunction, true);
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

        inline void DetEFSM::CheckTransition(const Z3TPRef& TP, u32 TransIndex,
                                             const vector<LTSSymbTransRef>& CandTrans) const
        {
            auto const Trans = CandTrans[TransIndex];
            auto TransAsInput = Trans->As<LTSSymbInputTransition>();
            TypeRef MType = TypeRef::NullPtr;

            if (TransAsInput != nullptr) {
                MType = TransAsInput->GetMessageType();
            }

            for (u32 j = TransIndex + 1; j < CandTrans.size(); ++j) {
                auto const& OtherTrans = CandTrans[j];
                auto OtherAsInput = OtherTrans->As<LTSSymbInputTransition>();

                if (MType != TypeRef::NullPtr && OtherAsInput != nullptr) {
                    if (OtherAsInput->GetMessageType() != MType) {
                        continue;
                    }
                }

                if (CheckDeterminism) {
                    auto Disjoint =
                        CheckDisjoint(Trans->GetGuard(), OtherTrans->GetGuard(), TP);

                    if (!Disjoint) {
                        throw ESMCError((string)"Determinism Check failed on EFSM \"" +
                                        Name + "\"\nOn Transitions:\n" + Trans->ToString() +
                                        "\n\nand:\n\n" + OtherTrans->ToString());
                    }
                }
            }
        }


        // Overriden to check determinism
        void DetEFSM::Freeze()
        {
            EFSMBase::Freeze();

            // Check for determinism
            Z3TPRef TP = new Z3TheoremProver();

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
                                       LTSFairnessType Fairness,
                                       bool CheckDeterminism)
            : DetEFSM(TheLTS, Name, Params, Constraint, Fairness, CheckDeterminism)
        {
            // Nothing here
        }

        IncompleteEFSM::~IncompleteEFSM()
        {
            // Nothing here
        }

        void IncompleteEFSM::AddVariable(const string& VarName, const TypeRef& VarType)
        {
            DetEFSM::AddVariable(VarName, VarType);
            UpdateableVariables[VarName] = VarType;
            AllVariables[VarName] = VarType;
        }

        WellOrderedExpSetT IncompleteEFSM::GetDomainTerms(const map<string, TypeRef>& DomainVars)
        {
            WellOrderedExpSetT Retval;
            for (auto const& Var : DomainVars) {
                auto VarExp = TheLTS->MakeVar(Var.first, Var.second);
                ExpandExpression(VarExp, Retval);
            }

            return Retval;
        }

        inline void IncompleteEFSM::FilterTerms(WellOrderedExpSetT& DomainTerms,
                                                const TypeRef& RangeType)
        {
            vector<WellOrderedExpSetT::iterator> ToRemove;

            for (auto it1 = DomainTerms.begin(); it1 != DomainTerms.end(); ++it1) {
                auto Type = (*it1)->GetType();
                if (!Type->Is<SymmetricType>()) {
                    continue;
                }
                if (Type == RangeType) {
                    continue;
                }
                bool FoundAnother = false;
                for (auto it2 = DomainTerms.begin(); it2 != DomainTerms.end(); ++it2) {
                    if (it2 == it1) {
                        continue;
                    }
                    if ((*it2)->GetType() == Type) {
                        FoundAnother = true;
                        break;
                    }
                }

                if (!FoundAnother) {
                    ToRemove.push_back(it1);
                }
            }

            for (auto const& it : ToRemove) {
                DomainTerms.erase(it);
            }
        }

        // Find the negation of the disjunction of the guards
        // which is also in the region defined by RegionConstraint
        inline ExpT
        IncompleteEFSM::FindDisjunction(const vector<LTSSymbTransRef>& Transitions,
                                        const Z3TPRef& TP,
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
                    vector<TypeRef> NewParamTypes(NumNewParams);
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

            auto Res = TP->CheckSat(NegCovered, true);
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
                                               const Z3TPRef& TP,
                                               const TypeRef& MsgType,
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
                                                const Z3TPRef& TP)
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

        inline vector<TypeRef>
        IncompleteEFSM::GetSymmTypesInExpr(const ExpT& Exp)
        {
            auto Mgr = TheLTS->GetMgr();

            auto&& SymmTerms =
                Mgr->Gather(Exp,
                            [&] (const ExpT& Exp) -> bool
                            {
                                auto const& ExpType = Exp->GetType();
                                return (ExpType->Is<SymmetricType>());
                            });

            WellOrderedTypeSetT SymmTypeSet;
            for_each(SymmTerms.begin(), SymmTerms.end(),
                     [&] (const ExpT& Term) -> void
                     {
                         SymmTypeSet.insert(Term->GetType());
                     });
            if (Exp->GetType()->Is<SymmetricType>()) {
                SymmTypeSet.insert(Exp->GetType());
            }

            return vector<TypeRef>(SymmTypeSet.begin(), SymmTypeSet.end());
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
                         if (ArgType->Is<SymmetricType>()) {
                             SymmArgs.push_back(Arg);
                         } else {
                             NonSymmArgs.push_back(Arg);
                         }
                     });
        }

        inline void
        IncompleteEFSM::MergeEquivalences(const WellOrderedExpSetT& NewEquivalences,
                                          set<WellOrderedExpSetT>& EquivalenceSets)
        {
            vector<set<WellOrderedExpSetT>::const_iterator> ToMerge;
            WellOrderedExpSetT MergedSet(NewEquivalences.begin(), NewEquivalences.end());

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

        inline set<WellOrderedExpSetT>
        IncompleteEFSM::FindEquivalences(const ExpT& Exp,
                                         const vector<TypeRef>& SymmTypes,
                                         const vector<ExpT>& SymmArgs,
                                         const vector<ExpT>& NonSymmArgs)
        {
            WellOrderedTypeMapT<u32> TypeOffsets;
            vector<u32> DomainSizes;
            auto const IsRangeSymmetric = Exp->GetType()->Is<SymmetricType>();
            set<WellOrderedExpSetT> Retval;
            const u32 NumSymmArgs = SymmArgs.size();
            auto Mgr = TheLTS->GetMgr();

            // Run offset, run!!
            u32 RunningOffset = 0;
            for (auto const& SymmType : SymmTypes) {
                auto const CurSize = SymmType->GetCardinalityNoUndef();
                DomainSizes.push_back(CurSize);
                TypeOffsets[SymmType] = RunningOffset;
                RunningOffset += CurSize;
            }

            PermutationSet PermSet(DomainSizes, false);
            const u32 NumPerms = PermSet.GetSize();

            // Get the cross product of values for symmetric arguments
            vector<vector<string>> SymmArgValues;
            for (auto const& SymmArg : SymmArgs) {
                auto const& ArgValues = SymmArg->GetType()->GetElementsNoUndef();
                SymmArgValues.push_back(ArgValues);
                // Add the clear value if arg is an lvalue
                if (SymmArg->Is<Exprs::VarExpression>()) {
                    auto ArgAsVar = SymmArg->SAs<Exprs::VarExpression>();
                    auto const& VarName = ArgAsVar->GetVarName();
                    auto Res = SymTab.Lookup(VarName);
                    if (Res->Is<ParamDecl>()) {
                        continue;
                    } else {
                        auto TypeAsSym = SymmArg->GetType()->SAs<SymmetricType>();
                        SymmArgValues.back().push_back(TypeAsSym->GetClearValue());
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

                WellOrderedExpSetT CurEquivalences;

                if (!IsRangeSymmetric) {
                    auto SubstExp = Mgr->TermSubstitute(SubstMap, Exp);
                    CurEquivalences.insert(SubstExp);
                    for (u32 i = 0; i < NumPerms; ++i) {
                        auto const& CurPerm = PermSet.GetIterator(i).GetPerm();

                        auto PermExp = Mgr->ApplyTransform<ExpressionPermuter>(SubstExp,
                                                                               CurPerm,
                                                                               TypeOffsets);
                        CurEquivalences.insert(PermExp);
                    }
                    MergeEquivalences(CurEquivalences, Retval);
                } else {
                    // The range itself is symmetric
                    auto const& RangeType = Exp->GetType();
                    auto RangeElems = RangeType->GetElements();

                    for (auto const& RangeElem : RangeElems) {
                        auto CurVal = Mgr->MakeVal(RangeElem, RangeType);
                        auto EQExp = Mgr->MakeExpr(LTSOps::OpEQ, Exp, CurVal);
                        auto SubstExp = Mgr->TermSubstitute(SubstMap, EQExp);

                        CurEquivalences.clear();
                        CurEquivalences.insert(SubstExp);

                        for (u32 i = 0; i < NumPerms; ++i) {
                            auto const& CurPerm = PermSet.GetIterator(i).GetPerm();
                            auto PermExp = Mgr->ApplyTransform<ExpressionPermuter>(SubstExp,
                                                                                   CurPerm,
                                                                                   TypeOffsets);
                            CurEquivalences.insert(PermExp);
                        }
                        MergeEquivalences(CurEquivalences, Retval);
                    }
                }
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

            if (Exp1->GetType()->Is<BooleanType>()) {
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
                    vector<TypeRef> QVarTypes(NumNonSymmArgs);

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

        inline ExpT IncompleteEFSM::MakeGuard(const WellOrderedExpSetT& DomainTerms,
                                              const ExpT& CoveredPred,
                                              const string& NameSuffix)
        {
            auto Mgr = TheLTS->GetMgr();
            auto LocalDomainTerms = DomainTerms;
            FilterTerms(LocalDomainTerms, TheLTS->MakeBoolType());
            vector<ExpT> DomainTermVec(LocalDomainTerms.begin(),
                                       LocalDomainTerms.end());

            vector<TypeRef> DomainTypes;

            for_each(LocalDomainTerms.begin(), LocalDomainTerms.end(),
                     [&] (const ExpT& DomainTerm) -> void
                     {
                         DomainTypes.push_back(DomainTerm->GetType());
                     });

            // Register a new uninterpreted function
            string GuardUFName =
                (string)"SynGuard_" + this->Name + "_" +
                to_string(GuardUFUIDGen.GetUID()) + "_" + NameSuffix;

            auto GuardOp = Mgr->MakeUninterpretedFunction(GuardUFName,
                                                          DomainTypes,
                                                          Mgr->MakeType<BooleanType>());
            auto GuardExp = Mgr->MakeExpr(GuardOp, DomainTermVec);

            auto&& SymmConstraints = GetSymmetryConstraints(GuardExp);

            GuardSymmetryConstraints[GuardOp].insert(SymmConstraints.begin(),
                                                     SymmConstraints.end());

            GuardMutualExclusiveSets[GuardOp].insert(CoveredPred);
            GuardOpToExp[GuardOp] = GuardExp;
            AllOpToExp[GuardOp] = GuardExp;

            return GuardExp;
        }

        inline set<WellOrderedExpSetT>
        IncompleteEFSM::GetArrayLValueGroups(const WellOrderedExpSetT& LValues)
        {
            auto Mgr = TheLTS->GetMgr();
            // get the symmetric types
            WellOrderedTypeSetT SymmTypeSet;
            for (auto const& LValueTerm : LValues) {
                auto&& CurSymmTypes = GetSymmTypesInExpr(LValueTerm);
                SymmTypeSet.insert(CurSymmTypes.begin(), CurSymmTypes.end());
            }

            vector<TypeRef> SymmTypes(SymmTypeSet.begin(), SymmTypeSet.end());

            u32 RunningOffset = 0;
            vector<u32> DomainSizes;
            WellOrderedTypeMapT<u32> TypeOffsets;

            for (auto const& SymmType : SymmTypes) {
                auto const CurSize = SymmType->GetCardinalityNoUndef();
                DomainSizes.push_back(CurSize);
                TypeOffsets[SymmType] = RunningOffset;
                RunningOffset += CurSize;
            }

            PermutationSet PermSet(DomainSizes, false);
            const u32 NumPerms = PermSet.GetSize();

            // for each uncovered term, get all the other lvalues
            // which can be obtained by permuting and put them in
            // the same group
            set<WellOrderedExpSetT> Retval;
            WellOrderedExpSetT CoveredTerms;

            for (auto const& LValue : LValues) {
                if (CoveredTerms.find(LValue) != CoveredTerms.end()) {
                    continue;
                }

                WellOrderedExpSetT CurSet;
                CurSet.insert(LValue);
                for (u32 i = 0; i < NumPerms; ++i) {
                    auto const& CurPerm = PermSet.GetIterator(i).GetPerm();
                    auto PermExp = Mgr->ApplyTransform<ExpressionPermuter>(LValue, CurPerm,
                                                                           TypeOffsets);
                    if ((PermExp != LValue) && (LValues.find(PermExp) != LValues.end())) {
                        CurSet.insert(PermExp);
                        CoveredTerms.insert(PermExp);
                    }
                }

                if (CurSet.size() > 1) {
                    Retval.insert(CurSet);
                }
            }
            return Retval;
        }

        inline vector<ExpT>
        IncompleteEFSM::GetSymmetryConstraints(const WellOrderedExpSetT& UpdateGroup,
                                               const WellOrderedExpMapT<ExpT>& UpdateMap)
        {
            // Get all the symmetric types (again)
            auto Mgr = TheLTS->GetMgr();
            vector<ExpT> Retval;

            WellOrderedTypeSetT SymmTypeSet;
            for (auto const& UpdateTerm : UpdateMap) {
                auto&& CurSymmTypes1 = GetSymmTypesInExpr(UpdateTerm.first);
                SymmTypeSet.insert(CurSymmTypes1.begin(), CurSymmTypes1.end());
                auto&& CurSymmTypes2 = GetSymmTypesInExpr(UpdateTerm.second);
                SymmTypeSet.insert(CurSymmTypes2.begin(), CurSymmTypes2.end());
            }
            vector<TypeRef> SymmTypes(SymmTypeSet.begin(), SymmTypeSet.end());

            vector<ExpT> LHSTerms(UpdateGroup.begin(), UpdateGroup.end());
            vector<ExpT> RHSTerms;

            transform(LHSTerms.begin(), LHSTerms.end(), back_inserter(RHSTerms),
                      [&] (const ExpT& Exp) -> ExpT
                      {
                          auto it = UpdateMap.find(Exp);
                          if (it == UpdateMap.end()) {
                              throw InternalError((string)"Could not find update for term:\n" +
                                                  Exp->ToString() + "\nAt: " + __FILE__ + ":" +
                                                  to_string(__LINE__));
                          }
                          return it->second;
                      });

            // Get the args of the application
            auto CandidateApp = *(RHSTerms.begin());
            auto CandidateAppAsOp = CandidateApp->As<OpExpression>();
            if (CandidateAppAsOp == nullptr) {
                throw InternalError((string)"Expected an uninterpreted function " +
                                    "application as expression argument in " +
                                    "IncompleteEFSM::GetSymmetryConstraints()\n" +
                                    "At: " + __FILE__ + ":" + to_string(__LINE__));
            }
            auto const& AllArgs = CandidateAppAsOp->GetChildren();
            vector<ExpT> SymmArgs;
            vector<ExpT> NonSymmArgs;
            PartitionDomain(AllArgs, SymmArgs, NonSymmArgs);

            const u32 NumNonSymmArgs = NonSymmArgs.size();


            vector<TypeRef> LHSTypes;
            transform(LHSTerms.begin(), LHSTerms.end(), back_inserter(LHSTypes),
                      [&] (const ExpT& Exp) -> TypeRef
                      {
                          return Exp->GetType();
                      });

            const u32 NumLHSTerms = LHSTerms.size();

            vector<vector<string>> LHSCPValues;
            transform(LHSTypes.begin(), LHSTypes.end(), back_inserter(LHSCPValues),
                      [&] (const TypeRef& Type) -> const vector<string>
                      {
                          return Type->GetElements();
                      });

            auto&& LHSCP = CrossProduct<string>(LHSCPValues.begin(),
                                                LHSCPValues.end());

            const string TempVarName = (string)"__temp_var__";

            set<WellOrderedExpSetT> AllEquivalences;

            for (auto const& CPTuple : LHSCP) {
                MgrT::SubstMapT PostSubstMap;
                vector<ExpT> Conjuncts;

                for (u32 i = 0; i < NumLHSTerms; ++i) {
                    auto CurVal = Mgr->MakeVal(CPTuple[i], LHSTypes[i]);

                    // create a new temp var that's isomorphic to the
                    // base LHS var in terms of symmetry
                    auto BaseLHSVar = GetBaseLValue(LHSTerms[i]);
                    MgrT::SubstMapT TempSubstMap;
                    TempSubstMap[BaseLHSVar] = Mgr->MakeVar(TempVarName,
                                                            BaseLHSVar->GetType());
                    auto NewLHSTerm = Mgr->Substitute(TempSubstMap, LHSTerms[i]);

                    auto CurConstraint = MakeEquivalence(RHSTerms[i], NewLHSTerm, Mgr);
                    PostSubstMap[NewLHSTerm] = CurVal;
                    Conjuncts.push_back(CurConstraint);
                }

                auto Antecedent = MakeConjunction(Conjuncts, Mgr);
                auto&& CurEquivalences = FindEquivalences(Antecedent, SymmTypes,
                                                          SymmArgs, NonSymmArgs);
                // Subst out the LHS terms
                set<WellOrderedExpSetT> SubstEquivalences;
                for (auto const& Equivalence : CurEquivalences) {
                    WellOrderedExpSetT SubstEquivalence;
                    for (auto const& Exp : Equivalence) {
                        auto SubstExp = Mgr->TermSubstitute(PostSubstMap, Exp);
                        SubstEquivalence.insert(SubstExp);
                    }
                    SubstEquivalences.insert(SubstEquivalence);
                }

                for (auto const& NewEquivalence : SubstEquivalences) {
                    MergeEquivalences(NewEquivalence, AllEquivalences);
                }
            }

            // Make the constraints and return
            for (auto const& EqClass : AllEquivalences) {
                auto const& Representative = *(EqClass.begin());
                for (auto it = next(EqClass.begin()); it != EqClass.end(); ++it) {
                    auto CurConstraint = MakeEquivalence(Representative, *it, Mgr);

                    MgrT::SubstMapT SubstMap;
                    vector<TypeRef> QVarTypes(NumNonSymmArgs);

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

        inline WellOrderedExpSetT
        IncompleteEFSM::GetDomainTermsForUpdate(const ExpT& LValueTerm,
                                                const WellOrderedExpSetT& DomainTerms,
                                                const SymmMsgDeclRef& MsgDecl)
        {
            auto BaseLVal = GetBaseLValue(LValueTerm);
            auto BaseLValAsVar = BaseLVal->As<VarExpression>();
            auto const& LValVarName = BaseLValAsVar->GetVarName();
            WellOrderedExpSetT Retval;

            if (LValVarName == "OutMsg") {
                auto LValAsOp = LValueTerm->As<OpExpression>();
                auto FieldVar = LValAsOp->GetChildren()[1];
                auto const& FieldName = FieldVar->As<VarExpression>()->GetVarName();

                auto it = OutMsgFieldDeps.find(make_pair(FieldName, MsgDecl));
                if (it == OutMsgFieldDeps.end()) {
                    return DomainTerms;
                } else {
                    for (auto const& DomainTerm : DomainTerms) {
                        auto BaseVar = GetBaseLValue(DomainTerm);
                        auto BaseAsVar = BaseVar->As<VarExpression>();
                        auto LocalParams = Params;
                        auto const& NewParams = MsgDecl->GetNewParams();
                        LocalParams.insert(LocalParams.end(), NewParams.begin(), NewParams.end());
                        if (find(LocalParams.begin(), LocalParams.end(), BaseVar) !=
                            LocalParams.end()) {
                            Retval.insert(DomainTerm);
                        } else {
                            auto const& BaseVarName = BaseAsVar->GetVarName();
                            if (it->second.find(BaseVarName) != it->second.end()) {
                                Retval.insert(DomainTerm);
                            }
                        }
                    }
                    return Retval;
                }
            } else {
                set<string> MyVarDeps;
                set<string> MyMsgFieldDeps;
                auto it = VarDeps.find(make_pair(LValVarName, MsgDecl));
                auto it2 = VarMsgFieldDeps.find(make_pair(LValVarName, MsgDecl));
                if (it2 == VarMsgFieldDeps.end() && it == VarDeps.end()) {
                        return DomainTerms;
                }
                if (it2 != VarMsgFieldDeps.end()) {
                    MyMsgFieldDeps = it2->second;
                }
                if (it != VarDeps.end()) {
                    MyVarDeps = it->second;
                }

                for (auto const& DomainTerm : DomainTerms) {
                    auto BaseVar = GetBaseLValue(DomainTerm);
                    auto BaseAsVar = BaseVar->As<VarExpression>();
                    auto const& BaseVarName = BaseAsVar->GetVarName();
                    string MsgFieldName;
                    if (BaseVarName == "InMsg") {
                        auto TermAsOp = DomainTerm->As<OpExpression>();
                        auto const& FieldVar = TermAsOp->GetChildren()[1]->As<VarExpression>();
                        MsgFieldName = FieldVar->GetVarName();
                    }

                    auto LocalParams = Params;
                    auto const& NewParams = MsgDecl->GetNewParams();
                    LocalParams.insert(LocalParams.end(), NewParams.begin(), NewParams.end());
                    if (find(LocalParams.begin(), LocalParams.end(), BaseVar) !=
                        LocalParams.end()) {
                        Retval.insert(DomainTerm);
                    } else {
                        if (MyVarDeps.find(BaseVarName) != MyVarDeps.end()) {
                            Retval.insert(DomainTerm);
                        }
                        if (MyMsgFieldDeps.find(MsgFieldName) != MyMsgFieldDeps.end()) {
                            Retval.insert(DomainTerm);
                        }
                    }
                }

                return Retval;
            }
        }

        inline vector<LTSAssignRef>
        IncompleteEFSM::MakeUpdates(i64 GuardOp,
                                    const string& InitStateName,
                                    const WellOrderedExpSetT& DomainTerms,
                                    const string& NameSuffix,
                                    const SymmMsgDeclRef& MsgDecl)
        {
            auto Mgr = TheLTS->GetMgr();
            vector<LTSAssignRef> Retval;

            map<string, TypeRef> LValues;
            for (auto const& Var : UpdateableVariables) {
                LValues.insert(Var);
            }

            auto&& LValueTerms = GetDomainTerms(LValues);
            auto&& ArrayLValueGroups = GetArrayLValueGroups(LValueTerms);
            WellOrderedExpMapT<WellOrderedExpSetT> GroupedLValues;
            WellOrderedExpMapT<ExpT> GroupedLValueToUpdateExp;

            if (ArrayLValueGroups.size() > 0) {

                for (auto const& Group : ArrayLValueGroups) {
                    for (auto const& LVal : Group) {
                        GroupedLValues[LVal] = Group;
                    }
                }
            }

            // Add the state lvalue
            LValueTerms.insert(TheLTS->MakeVar("state", StateType));

            for (auto const& LValue : LValueTerms) {

                auto&& LocalDomTerms = GetDomainTermsForUpdate(LValue, DomainTerms, MsgDecl);
                FilterTerms(LocalDomTerms, LValue->GetType());

                if (LValue->GetType()->Is<SymmetricType>()) {
                    bool FoundOne = false;
                    for (auto const& DomTerm : LocalDomTerms) {
                        if (DomTerm->GetType() == LValue->GetType()) {
                            FoundOne = true;
                            break;
                        }
                    }

                    // Symmetric type that doesn't have
                    // an argument of its type, continue
                    if (!FoundOne) {
                        continue;
                    }
                }

                vector<TypeRef> DomainTypes;
                vector<ExpT> DomainTermVec(LocalDomTerms.begin(),
                                           LocalDomTerms.end());
                for_each(LocalDomTerms.begin(), LocalDomTerms.end(),
                         [&] (const ExpT& DomainTerm) -> void
                         {
                             DomainTypes.push_back(DomainTerm->GetType());
                         });

                // Register a new uninterpreted function
                // for the update of each domain term
                string UpdateUFName =
                    (string)"Update_" + this->Name + "_" +
                    to_string(UpdateUFUIDGen.GetUID()) + "_" +
                    NameSuffix + "_" + LValue->ToString();

                auto UpdateOp = Mgr->MakeUninterpretedFunction(UpdateUFName,
                                                               DomainTypes,
                                                               LValue->GetType());
                auto UpdateExp = Mgr->MakeExpr(UpdateOp, DomainTermVec);
                GuardOpToUpdates[GuardOp].insert(UpdateExp);
                AllOpToExp[UpdateOp] = UpdateExp;

                vector<ExpT> SymmConstraints;
                if (GroupedLValues.find(LValue) == GroupedLValues.end()) {
                    auto&& SymmConstraints = GetSymmetryConstraints(UpdateExp);
                    GuardOpToUpdateSymmetryConstraints[GuardOp].insert(SymmConstraints.begin(),
                                                                       SymmConstraints.end());
                } else {
                    // Group symmetry constraints are applied later
                    GroupedLValueToUpdateExp[LValue] = UpdateExp;
                }

                Retval.push_back(new LTSAssignSimple(LValue, UpdateExp));

                auto BaseLValue = GetBaseLValue(LValue);
                auto BaseLValueVarName = BaseLValue->SAs<VarExpression>()->GetVarName();
                if (AllVariables.find(BaseLValueVarName) == AllVariables.end() &&
                    LValue != TheLTS->MakeVar("state", StateType)) {
                    continue;
                } else if (LValue != TheLTS->MakeVar("state", StateType)) {
                    UpdateOpToUpdateLValue[UpdateOp] = make_pair(UpdateExp, LValue);
                } else {
                    StateUpdateOpToExp[UpdateOp] = UpdateExp;
                    // also constrain the possible range if applicable
                    auto nsit = NextStatesOnTransition.find(make_pair(InitStateName, MsgDecl));
                    if (nsit != NextStatesOnTransition.end()) {
                        auto CandidateNS = nsit->second;
                        auto const& UpdateArgs = GetOpArgs(UpdateExp);
                        vector<TypeRef> UpdateArgTypes;
                        transform(UpdateArgs.begin(), UpdateArgs.end(),
                                  back_inserter(UpdateArgTypes),
                                  [&] (const ExpT& Exp) -> TypeRef
                                  {
                                      return Exp->GetType();
                                  });
                        const u32 NumUpdateArgs = UpdateArgs.size();
                        MgrT::SubstMapT SubstMap;
                        for (u32 i = 0; i < NumUpdateArgs; ++i) {
                            SubstMap[UpdateArgs[i]] = Mgr->MakeBoundVar(UpdateArgTypes[i],
                                                                        NumUpdateArgs - i - 1);
                        }
                        auto Body = Mgr->BoundSubstitute(SubstMap, UpdateExp);

                        vector<ExpT> Disjuncts;
                        for (auto const& NextState : CandidateNS) {
                            auto BodyEQTarget = Mgr->MakeExpr(LTSOps::OpEQ, Body,
                                                              Mgr->MakeVal(NextState, StateType));
                            Disjuncts.push_back(BodyEQTarget);
                        }

                        auto FinalBody = MakeDisjunction(Disjuncts, Mgr);
                        auto FinalQExpr = Mgr->MakeForAll(UpdateArgTypes, FinalBody);
                        GuardOpToUpdateSymmetryConstraints[GuardOp].insert(FinalQExpr);
                    }
                }
            }

            for (auto const& ArrayLValueGroup : ArrayLValueGroups) {
                auto&& SymmConstraints = GetSymmetryConstraints(ArrayLValueGroup,
                                                                GroupedLValueToUpdateExp);
                GuardOpToUpdateSymmetryConstraints[GuardOp].insert(SymmConstraints.begin(),
                                                                   SymmConstraints.end());
            }

            return Retval;
        }

        inline void
        IncompleteEFSM::CompleteOneInputTransition(const string& InitStateName,
                                                   const SymmMsgDeclRef& MsgDecl,
                                                   const map<string, TypeRef>& DomainVars,
                                                   const ExpT& CoveredPred)
        {
            auto LocalDomVars = DomainVars;

            // Get the domain terms
            auto&& DomainTerms = GetDomainTerms(LocalDomVars);
            FilterTerms(DomainTerms, TheLTS->MakeBoolType());

            // Add the input message for the updates
            auto const& MsgType = MsgDecl->GetMessageType();
            TypeRef ActMsgType = TypeRef::NullPtr;
            if (MsgType->Is<ParametricType>()) {
                ActMsgType = MsgType->SAs<ParametricType>()->GetBaseType();
            } else {
                ActMsgType = MsgType;
            }

            auto MsgTypeAsRecord = ActMsgType->SAs<RecordType>();
            auto NameSuffix = InitStateName + "_" + MsgTypeAsRecord->GetName();
            auto GuardExp = MakeGuard(DomainTerms, CoveredPred, NameSuffix);
            auto GuardOp = GuardExp->SAs<OpExpression>()->GetOpCode();

            LocalDomVars["InMsg"] = MsgDecl->GetMessageType();

            DomainTerms = GetDomainTerms(LocalDomVars);

            // Make the updates
            auto&& Updates = MakeUpdates(GuardOp, InitStateName, DomainTerms, NameSuffix, MsgDecl);

            auto const& NewParams = MsgDecl->GetNewParams();

            if (NewParams.size() == 0) {
                AddInputTransition(InitStateName, GuardExp,
                                   Updates, "InMsg", MsgDecl->GetMessageType(),
                                   MsgDecl->GetMessageParams(), true);
            } else {
                AddInputTransitions(NewParams, MsgDecl->GetConstraint(),
                                    InitStateName,
                                    GuardExp, Updates, "InMsg",
                                    MsgDecl->GetMessageType(),
                                    MsgDecl->GetMessageParams(), true);
            }

            AddedTransitionsByState[InitStateName].insert(GuardExp);
        }

        inline void
        IncompleteEFSM::CompleteInputTransitions(const string& StateName,
                                                 const vector<LTSSymbTransRef>& TransFromState,
                                                 const ExpT& CoveredRegion, const Z3TPRef& TP)
        {
            if (CompleteStates.find(StateName) != CompleteStates.end()) {
                return;
            }

            for (auto const& MsgDecl : SymmetricMessages) {
                if (!MsgDecl->IsInput()) {
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
                auto CoveredMsgPred = FindInputCoveredRegion(TransFromState, TP,
                                                             MsgDecl->GetMessageType(),
                                                             CoveredRegion);
                if (CoveredMsgPred == TheLTS->MakeTrue()) {
                    continue;
                }

                // We're okay to add one or more transitions
                // Gather the list of expresions that could be
                // arguments to the uninterpreted functions
                map<string, TypeRef> DomainVariables;

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

                CompleteOneInputTransition(StateName, MsgDecl, DomainVariables, CoveredMsgPred);
            }
        }

        void IncompleteEFSM::CompleteOneOutputTransition(const string& InitStateName,
                                                         const SymmMsgDeclRef& MsgDecl,
                                                         const map<string, TypeRef>& DomainVars,
                                                         vector<ExpT>& GuardExps,
                                                         const ExpT& CoveredPred)
        {
            auto&& DomainTerms = GetDomainTerms(DomainVars);

            auto const& MsgType = MsgDecl->GetMessageType();
            TypeRef ActMsgType = TypeRef::NullPtr;
            if (MsgType->Is<ParametricType>()) {
                ActMsgType = MsgType->SAs<ParametricType>()->GetBaseType();
            } else {
                ActMsgType = MsgType;
            }

            auto MsgTypeAsRecord = ActMsgType->SAs<RecordType>();
            auto NameSuffix = InitStateName + "_" + MsgTypeAsRecord->GetName();
            auto GuardExp = MakeGuard(DomainTerms, CoveredPred, NameSuffix);
            auto GuardOp = GuardExp->SAs<OpExpression>()->GetOpCode();

            UpdateableVariables["OutMsg"] = MsgType;

            auto&& Updates = MakeUpdates(GuardOp, InitStateName, DomainTerms, NameSuffix, MsgDecl);

            UpdateableVariables.erase("OutMsg");

            auto const& NewParams = MsgDecl->GetNewParams();

            if (NewParams.size() == 0) {
                AddOutputTransition(InitStateName, GuardExp,
                                    Updates, "OutMsg", MsgDecl->GetMessageType(),
                                    MsgDecl->GetMessageParams(), set<string>(),
                                    true);
            } else {
                AddOutputTransitions(NewParams, MsgDecl->GetConstraint(),
                                     InitStateName, GuardExp, Updates, "OutMsg",
                                     MsgDecl->GetMessageType(),
                                     MsgDecl->GetMessageParams(),
                                     set<string>(), set<string>(), true);
            }

            AddedTransitionsByState[InitStateName].insert(GuardExp);
            for (auto const& OtherGuard : GuardExps) {
                auto OtherOp = OtherGuard->SAs<OpExpression>()->GetOpCode();
                GuardMutualExclusiveSets[GuardOp].insert(OtherGuard);
                GuardMutualExclusiveSets[OtherOp].insert(GuardExp);
                GuardMutualExclusiveSets[GuardOp].insert(CoveredPred);
            }
            GuardExps.push_back(GuardExp);
        }

        void IncompleteEFSM::CompleteOutputTransitions(const string& StateName,
                                                       const ExpT& CoveredPredicate,
                                                       const Z3TPRef& TP)
        {
            if (CompleteStates.find(StateName) != CompleteStates.end()) {
                return;
            }

            auto it = AddedTransitionsByState.find(StateName);
            vector<ExpT> GuardExps;
            if (it != AddedTransitionsByState.end()) {
                GuardExps.insert(GuardExps.end(), it->second.begin(),
                                 it->second.end());
            }

            for (auto const& MsgDecl : SymmetricMessages) {
                if (!MsgDecl->IsOutput()) {
                    continue;
                }

                auto it = BlockedCompletions.find(StateName);
                if (it != BlockedCompletions.end() &&
                    it->second.find(MsgDecl) != it->second.end()) {
                    continue;
                }

                map<string, TypeRef> DomainVariables;
                for (auto const& Var : AllVariables) {
                    DomainVariables.insert(Var);
                }

                for (auto const& Param : Params) {
                    auto const& ParamName = Param->As<VarExpression>()->GetVarName();
                    DomainVariables[ParamName] = Param->GetType();
                }

                auto const& NewMsgParams = MsgDecl->GetNewParams();
                for (auto const& NewMsgParam : NewMsgParams) {
                    auto ParamAsVar = NewMsgParam->As<VarExpression>();
                    assert(ParamAsVar != nullptr);
                    DomainVariables[ParamAsVar->GetVarName()] = ParamAsVar->GetVarType();
                }

                CompleteOneOutputTransition(StateName, MsgDecl, DomainVariables,
                                            GuardExps, CoveredPredicate);
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

            Z3TPRef TP = new Z3TheoremProver();
            vector<ExpT> AddedGuards;
            auto Mgr = TheLTS->GetMgr();

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
                    continue;
                }

                // Add transitions
                CompleteInputTransitions(StateName, TransFromState, CoveredPred, TP);
            }

            for (auto const& NameState : States) {
                auto const& StateName = NameState.first;
                auto Pred =
                    [&] (const LTSSymbTransRef& Trans) -> bool
                    {
                        auto Part1 = (Trans->GetInitState().GetName() == StateName);
                        auto OpCode = Trans->GetGuard()->SAs<OpExpression>()->GetOpCode();
                        auto Part2 = (LTSReservedOps.find(OpCode) != LTSReservedOps.end());
                        return (Part1 && Part2);
                    };
                auto&& FixedTransFromState = GetSymbolicTransitions(Pred);
                auto CoveredPred = FindDisjunction(FixedTransFromState, TP, Mgr->MakeFalse());
                if (CoveredPred == Mgr->MakeTrue()) {
                    continue;
                }

                CompleteOutputTransitions(StateName, CoveredPred, TP);
            }

            EFSMFrozen = true;
        }

        void IncompleteEFSM::IgnoreAllMsgsOnState(const string& StateName)
        {
            for (auto const& MsgDecl : SymmetricMessages) {
                IgnoreMsgOnState(MsgDecl, StateName);
            }
        }

        void IncompleteEFSM::HandleMsgOnState(const SymmMsgDeclRef& MsgDecl,
                                              const string& StateName)
        {
            auto it = BlockedCompletions.find(StateName);
            if (it == BlockedCompletions.end()) {
                return;
            }
            BlockedCompletions[StateName].erase(MsgDecl);
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

        void IncompleteEFSM::MarkAllStatesComplete()
        {
            for (auto const& State : States) {
                MarkStateComplete(State.first);
            }
        }

        void IncompleteEFSM::MarkStateIncomplete(const string& StateName)
        {
            CompleteStates.erase(StateName);
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

        void IncompleteEFSM::MarkAllVariablesReadOnly()
        {
            for (auto const& Var : AllVariables) {
                ReadOnlyVars.insert(Var.first);
                UpdateableVariables.erase(Var.first);
            }
        }

        void IncompleteEFSM::MarkVariableWriteable(const string& VarName)
        {
            auto const& STEntry = SymTab.Lookup(VarName);
            if (STEntry == DeclRef::NullPtr || !STEntry->Is<VarDecl>()) {
                throw ESMCError((string)"Object named \"" + VarName + "\" is not " +
                                "a variable of the EFSM named \"" + Name + "\"" +
                                " in call to IncompleteEFSM::MarkVariableReadOnly()");
            }
            ReadOnlyVars.erase(VarName);
            UpdateableVariables[VarName] = STEntry->GetType();
        }

        void IncompleteEFSM::SetVariableDepsOnMsg(const string& VarName,
                                                  const SymmMsgDeclRef& MsgDecl,
                                                  const set<string>& DepVars,
                                                  const set<string>& MessageFieldNames)
        {
            if (AllVariables.find(VarName) == AllVariables.end()) {
                throw ESMCError((string)"EFSM \"" + Name + "\" has no variable named \"" +
                                VarName + "\", but SetVariableDepsOnMsg called on it!");
            }

            if (find(SymmetricMessages.begin(), SymmetricMessages.end(), MsgDecl) ==
                SymmetricMessages.end()) {
                throw ESMCError((string)"Invalid message decl on SetVariableDepsOnMsg()");
            }

            for (auto const& DepVar : DepVars) {
                if (AllVariables.find(DepVar) == AllVariables.end()) {
                    throw ESMCError((string)"EFSM \"" + Name + "\" has no variable named \"" +
                                    DepVar + "\", but SetVariableDepsOnMsg called on it!");
                }
            }

            VarDeps[make_pair(VarName, MsgDecl)] = DepVars;

            if (MessageFieldNames.size() > 0 && MsgDecl->IsOutput()) {
                throw ESMCError((string)"Message field dep specified for EFSM, but the " +
                                "message associated with dependency is an output message");
            }

            auto BaseType = MsgDecl->GetBaseMessageType();
            auto BaseAsRec = BaseType->SAs<RecordType>();

            auto const& FieldMap = BaseAsRec->GetMemberMap();
            for (auto const& MessageFieldName : MessageFieldNames) {
                if (FieldMap.find(MessageFieldName) == FieldMap.end()) {
                    throw ESMCError((string)"Message type named \"" + BaseAsRec->GetName() + "\"" +
                                    " does not have a field called \"" + MessageFieldName + "\"");
                }
            }
            VarMsgFieldDeps[make_pair(VarName, MsgDecl)] = MessageFieldNames;
        }

        void IncompleteEFSM::SetOutMsgFieldDeps(const SymmMsgDeclRef& OutMsgDecl,
                                                const string& FieldName,
                                                const set<string>& DepVars)
        {
            auto BaseType = OutMsgDecl->GetBaseMessageType();
            auto BaseAsRec = BaseType->SAs<RecordType>();

            if (find(SymmetricMessages.begin(), SymmetricMessages.end(), OutMsgDecl) ==
                SymmetricMessages.end() || !OutMsgDecl->IsOutput()) {
                throw ESMCError((string)"Invalid message decl on SetVariableDepsOnMsg()");
            }

            for (auto const& DepVar : DepVars) {
                if (AllVariables.find(DepVar) == AllVariables.end()) {
                    throw ESMCError((string)"EFSM \"" + Name + "\" has no variable named \"" +
                                    DepVar + "\", but SetVariableDepsOnMsg called on it!");
                }
            }

            auto const& FieldMap = BaseAsRec->GetMemberMap();
            if (FieldMap.find(FieldName) == FieldMap.end()) {
                throw ESMCError((string)"Message type named \"" + BaseAsRec->GetName() + "\"" +
                                " does not have a field called \"" + FieldName + "\"");
            }

            OutMsgFieldDeps[make_pair(FieldName, OutMsgDecl)] = DepVars;
        }

        void IncompleteEFSM::SetNextStatesOnTransition(const string& StateName,
                                                       const SymmMsgDeclRef& MsgDecl,
                                                       const set<string>& NextStateNames)
        {
            auto it = States.find(StateName);
            if (it == States.end()) {
                throw ESMCError((string)"No state called \"" + StateName + "\" exists " +
                                "in EFSM named \"" + Name + "\" in call to " +
                                "IncompleteEFSM::SetNextStatesOnTransition()");
            }
            if (find(SymmetricMessages.begin(), SymmetricMessages.end(), MsgDecl) ==
                SymmetricMessages.end()) {
                throw ESMCError((string)"Invalid message decl in call to " +
                                "IncompleteEFSM::SetNextStatesOnTransition()");
            }
            for (auto const& NextState : NextStateNames) {
                auto it2 = States.find(NextState);
                if (it2 == States.end()) {
                    throw ESMCError((string)"No state called \"" + NextState + "\" exists " +
                                    "in EFSM named \"" + Name + "\" in call to " +
                                    "IncompleteEFSM::SetNextStatesOnTransition()");
                }
            }

            NextStatesOnTransition[make_pair(StateName, MsgDecl)] = NextStateNames;
        }


    } /* end namespace LTS */
} /* end namespace ESMC */

//
// LTSEFSM.cpp ends here
