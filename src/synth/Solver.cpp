// Solver.cpp ---
//
// Filename: Solver.cpp
// Author: Abhishek Udupa
// Created: Thu Oct 23 11:13:37 2014 (-0400)
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
#include "../mc/Trace.hpp"
#include "../mc/Compiler.hpp"
#include "../uflts/LabelledTS.hpp"
#include "../uflts/LTSTypes.hpp"
#include "../uflts/LTSEFSM.hpp"
#include "../uflts/LTSUtils.hpp"
#include "../mc/LTSChecker.hpp"
#include "../symexec/LTSAnalyses.hpp"
#include "../mc/OmegaAutomaton.hpp"
#include "../mc/StateVecPrinter.hpp"

#include "Solver.hpp"

namespace ESMC {
    namespace Synth {

        using namespace ESMC::LTS;
        using namespace ESMC::MC;
        using namespace ESMC::TP;
        using namespace ESMC::Analyses;

        using LTS::ExpT;

        const u64 TentativeEdgeCost = ((u64)1 << 30);
        const u32 LimitOnBound = 1024;

        const string Solver::BoundsVarPrefix = (string)"__SynthBound__";

        Solver::Solver(LTSChecker* Checker)
            : TP(new Z3TheoremProver()),
              TheLTS(Checker->TheLTS),
              Compiler(Checker->Compiler),
              Checker(Checker),
              Bound(0),
              GuardedCommands(TheLTS->GetGuardedCmds()),
              UpdateBoundsMultiplier(0),
              UpdateBound(0)
        {
            auto Mgr = TheLTS->GetMgr();
            // push a scope onto the theorem prover and assert
            // true

            TP->Push();
            TP->Assert(TheLTS->MakeTrue(), true);

            TPAsZ3 = const_cast<Z3TheoremProver*>(TP->As<Z3TheoremProver>());
            Ctx = TPAsZ3->GetCtx();

            // Find the maximum number of lvalues in any incomplete EFSM
            u32 MaxLValues = 0;
            auto const& AllEFSMs = TheLTS->AllEFSMs;

            for (auto const& NameEFSM : AllEFSMs) {
                if (!NameEFSM.second->Is<IncompleteEFSM>()) {
                    continue;
                }
                auto EFSMAsInc = NameEFSM.second->SAs<IncompleteEFSM>();

                vector<ExpT> LValues;
                for (auto const& Var : EFSMAsInc->UpdateableVariables) {
                    auto&& CurLValues = GetScalarTerms(Mgr->MakeVar(Var.first,
                                                                    Var.second));
                    LValues.insert(LValues.end(), CurLValues.begin(),
                                   CurLValues.end());
                }

                if (LValues.size() > MaxLValues) {
                    MaxLValues = LValues.size();
                }
            }

            UpdateBoundsMultiplier = MaxLValues;

            // Populate the set of fixed commands and create the cost function
            unordered_set<u32> FixedCommands;
            for (auto const& Cmd : GuardedCommands) {
                auto const& SynthOps = GetSynthOps(Cmd->GetGuard());
                if (SynthOps.size() == 0) {
                    FixedCommands.insert(Cmd->GetCmdID());
                }
            }

            CostFunction = Detail::SynthCostFunction(FixedCommands);
        }

        inline void Solver::CheckedAssert(const ExpT& Assertion)
        {
            if (AssertedConstraints.find(Assertion) != AssertedConstraints.end()) {
                return;
            }

            AssertedConstraints.insert(Assertion);
            cout << "Asserting: " << Assertion->ToString() << endl;
            TP->Assert(Assertion, true);
        }

        Solver::~Solver()
        {
            // Nothing here
        }

        inline void Solver::HandleOneSafetyViolation(const StateVec* ErrorState,
                                                     const ExpT& BlownInvariant)
        {
            auto Mgr = TheLTS->GetMgr();
            auto AQS = Checker->AQS;

            cout << "Handling one safety violation, computing shortest path... ";
            flush(cout);

            // auto PPath = AQS->FindShortestPath(ErrorState, CostFunction);
            auto PPath = AQS->FindPath(ErrorState);

            cout << "Done!" << endl << "Unwinding trace... ";
            flush(cout);

            auto Trace = TraceBase::MakeSafetyViolation(PPath, Checker, BlownInvariant);

            cout << "Done!" << endl
                 << "Got trace with " << Trace->GetTraceElems().size() << " steps" << endl
                 << "Finding invariant that was blown... ";
            flush(cout);

            auto LastState = Trace->GetTraceElems().back().second;

            auto ActualBlownInvariant = BlownInvariant;
            // Find out the invariant blown on the last state of trace now
            if (BlownInvariant != Checker->TheLTS->InvariantExp) {
                auto const& BoundsInvariants = Checker->BoundsInvariants;

                bool FoundBlown = false;
                for (auto const& Invar : BoundsInvariants) {
                    // cout << "Evaluating invariant: " << Invar->ToString() << endl;
                    auto Interp = Invar->ExtensionData.Interp;
                    auto Res = Interp->Evaluate(LastState);
                    if (Res == UndefValue) {
                        continue;
                    } else if (Res == 0) {
                        FoundBlown = true;
                        ActualBlownInvariant = Invar;
                    }
                }

                if (!FoundBlown) {
                    ostringstream sstr;
                    Checker->Printer->PrintState(LastState, sstr);
                    throw InternalError((string)"Could not find the bounds invariant that was " +
                                        "blown in call to Solver::HandleOneSafetyViolation()\n" +
                                        "The State:\n" + sstr.str() + "\nCould not find bounds " +
                                        "invariant that was blown for the state listed above.\n" +
                                        "At: " + __FILE__ + ":" + to_string(__LINE__));
                }
            }

            cout << "Done!" << endl << "Blown Invariant: " << endl
                 << ActualBlownInvariant->ToString() << endl << "Computing weakest pre... ";
            flush(cout);

            auto&& WPConditions =
                TraceAnalyses::WeakestPrecondition(this, Trace, ActualBlownInvariant);
            for (auto const& Pred : WPConditions) {
                cout << "Asserting Safety Pre:" << endl
                     << Pred->ToString() << endl << endl;
                MakeAssertion(Mgr->Simplify(Pred));
            }

            delete Trace;
        }

        inline void Solver::CreateMutualExclusionConstraint(const ExpT& GuardExp1,
                                                            const ExpT& GuardExp2)
        {
            auto Mgr = TheLTS->GetMgr();

            set<ExpT> Args;
            vector<ExpT> ArgsMine;
            if (GuardExp1->Is<OpExpression>() &&
                LTSReservedOps.find(GuardExp1->SAs<OpExpression>()->GetOpCode()) ==
                LTSReservedOps.end()) {
                ArgsMine = GetOpArgs(GuardExp1);
            }
            vector<ExpT> ArgsOther;
            if (GuardExp2->Is<OpExpression>() &&
                LTSReservedOps.find(GuardExp2->SAs<OpExpression>()->GetOpCode()) ==
                LTSReservedOps.end()) {
                ArgsOther = GetOpArgs(GuardExp2);
            }

            Args.insert(ArgsMine.begin(), ArgsMine.end());
            Args.insert(ArgsOther.begin(), ArgsOther.end());

            vector<ExpT> QVars(Args.begin(), Args.end());
            vector<ExprTypeRef> QVarTypes;

            auto MutexExp = Mgr->MakeExpr(LTSOps::OpAND, GuardExp1, GuardExp2);
            MutexExp = Mgr->MakeExpr(LTSOps::OpNOT, MutexExp);

            transform(QVars.begin(), QVars.end(), back_inserter(QVarTypes),
                      [&] (const ExpT& Exp) -> ExprTypeRef
                      {
                          return Exp->GetType();
                      });
            MgrT::SubstMapT SubstMap;
            const u32 NumQVars = QVarTypes.size();
            for (u32 i = 0; i < NumQVars; ++i) {
                SubstMap[QVars[i]] = Mgr->MakeBoundVar(QVarTypes[i],
                                                       NumQVars - i - 1);
            }
            auto QBody = Mgr->BoundSubstitute(SubstMap, MutexExp);
            auto Constraint = Mgr->MakeForAll(QVarTypes, QBody);
            CheckedAssert(Constraint);
        }

        inline void Solver::CreateGuardIndicator(i64 GuardOp)
        {
            auto Mgr = TheLTS->GetMgr();

            auto IndicatorUID = IndicatorUIDGenerator.GetUID();
            string IndicatorVarName = (string)"__indicator_" + to_string(IndicatorUID);
            auto FuncType = Mgr->LookupUninterpretedFunction(GuardOp)->As<ExprFuncType>();
            IndicatorVarName += (string)"_" + FuncType->GetName();
            auto const& DomainTypes = FuncType->GetArgTypes();
            vector<ExpT> BoundArgs;
            const u32 NumDomainTypes = DomainTypes.size();
            for (u32 i = 0; i < NumDomainTypes; ++i) {
                auto BoundVar = Mgr->MakeBoundVar(DomainTypes[i], NumDomainTypes - i - 1);
                BoundArgs.push_back(BoundVar);
            }
            auto AppExp = Mgr->MakeExpr(GuardOp, BoundArgs);
            auto ExistsExp = Mgr->MakeExists(DomainTypes, AppExp);
            auto IndicatorType = Mgr->MakeType<ExprRangeType>(0, 1);

            auto IndicatorVar = Mgr->MakeVar(IndicatorVarName, IndicatorType);
            IndicatorExps[GuardOp] = IndicatorVar;

            auto Implies = Mgr->MakeExpr(LTSOps::OpIMPLIES, ExistsExp,
                                         Mgr->MakeExpr(LTSOps::OpEQ, IndicatorVar,
                                                       Mgr->MakeVal("1", IndicatorType)));
            cout << "Asserting Indicator Implication:" << endl
                 << Implies->ToString() << endl << endl;
            CheckedAssert(Implies);
        }

        inline void Solver::CreateUpdateIndicator(i64 UpdateOp)
        {
            auto Mgr = TheLTS->GetMgr();
            auto const& UpdateOpToLValue = TheLTS->UpdateOpToUpdateLValue;

            auto it = UpdateOpToLValue.find(UpdateOp);
            if (it == UpdateOpToLValue.end()) {
                // This must be an exempt lvalue
                return;
            }

            auto IndicatorUID = UpdateIndicatorUIDGenerator.GetUID();
            string IndicatorVarName =
                (string)"__update_indicator_" + to_string(IndicatorUID);

            auto const& UpdateExp = it->second.first;
            auto const& LValue = it->second.second;

            auto const& OpArgs = GetOpArgs(UpdateExp);
            // remove the lvalue itself from the op args
            vector<ExprTypeRef> ArgTypes;
            transform(OpArgs.begin(), OpArgs.end(), back_inserter(ArgTypes),
                      [&] (const ExpT& Exp) -> ExprTypeRef
                      {
                          return Exp->GetType();
                      });

            // replace each of the arg types by a bound var
            MgrT::SubstMapT SubstMap;
            const u32 NumArgs = ArgTypes.size();
            for (u32 i = 0; i < NumArgs; ++i) {
                auto BoundVar = Mgr->MakeBoundVar(ArgTypes[i], NumArgs - i - 1);
                SubstMap[OpArgs[i]] = BoundVar;
            }

            // Substitute the bound vars
            auto QBodyExp = Mgr->MakeExpr(LTSOps::OpEQ, LValue, UpdateExp);
            QBodyExp = Mgr->MakeExpr(LTSOps::OpNOT, QBodyExp);
            QBodyExp = Mgr->BoundSubstitute(SubstMap, QBodyExp);
            auto ExistsExp = Mgr->MakeExists(ArgTypes, QBodyExp);
            auto IndicatorType = Mgr->MakeType<ExprRangeType>(0, 1);
            auto IndicatorVar = Mgr->MakeVar(IndicatorVarName, IndicatorType);
            UpdateIndicatorExps[UpdateOp] = IndicatorVar;

            auto ImpliesExp = Mgr->MakeExpr(LTSOps::OpIMPLIES, ExistsExp,
                                            Mgr->MakeExpr(LTSOps::OpEQ, IndicatorVar,
                                                          Mgr->MakeVal("1", IndicatorType)));
            cout << "Asserting Update Indicator Implication:" << endl
                 << ImpliesExp->ToString() << endl << endl;
            CheckedAssert(ImpliesExp);
        }

        inline void Solver::UpdateCommands()
        {
            for (auto const& Cmd : GuardedCommands) {
                auto&& SynthOps = GetSynthOps(Cmd->GetGuard());
                bool FullyInterpreted = true;
                for (auto const& Op : SynthOps) {
                    if (InterpretedOps.find(Op) == InterpretedOps.end()) {
                        FullyInterpreted = false;
                        break;
                    }
                }

                Cmd->SetFullyInterpreted(FullyInterpreted);
            }
        }

        void Solver::UnveilGuardOp(i64 Op)
        {
            auto const& GuardOpToExp = TheLTS->GuardOpToExp;
            auto const& GuardSymmetryConstraints = TheLTS->GuardSymmetryConstraints;
            auto const& GuardMutualExclusiveSets = TheLTS->GuardMutualExclusiveSets;
            auto const& GuardOpToUpdates = TheLTS->GuardOpToUpdates;
            auto const& GuardOpToUpdateSymmetryConstraints =
                TheLTS->GuardOpToUpdateSymmetryConstraints;

            unordered_set<i64> NewlyUnveiledUpdates;
            auto ExpIt = GuardOpToExp.find(Op);
            if (ExpIt == GuardOpToExp.end()) {
                // Not unveiled, and not a guard, must be an unveiled update
                if (UnveiledUpdateOps.find(Op) == UnveiledUpdateOps.end()) {
                    throw InternalError((string)"Expected Op " + to_string(Op) +
                                        "to have been unveiled already.\nWhen asserting: " +
                                        "At: " + __FILE__ + ":" +
                                        to_string(__LINE__));
                }
                return;
            }

            auto GuardExp = ExpIt->second;

            cout << "Unveiling Guard Exp: " << GuardExp->ToString() << endl;
            cout << "Asserting Symmetry constraints:" << endl;

            // This is a new guard
            // Assert the symmetry constraints
            auto it = GuardSymmetryConstraints.find(Op);
            if (it != GuardSymmetryConstraints.end()) {
                for (auto const& Constraint : it->second) {
                    CheckedAssert(Constraint);
                }
            }

            cout << "End of Symmetry constraints:" << endl;
            cout << "Asserting Determinism constraints:" << endl;

            // Assert the determinism constraints wrt guards
            // that have already been unveiled
            auto it2 = GuardMutualExclusiveSets.find(Op);
            if (it2 != GuardMutualExclusiveSets.end()) {
                for (auto const& OtherGuard : it2->second) {
                    auto OtherOp = OtherGuard->SAs<OpExpression>()->GetOpCode();
                    if (UnveiledGuardOps.find(OtherOp) == UnveiledGuardOps.end() &&
                        LTSReservedOps.find(OtherOp) == LTSReservedOps.end()) {
                        continue;
                    }
                    // Assert the mutual exclusion constraint
                    CreateMutualExclusionConstraint(GuardExp, OtherGuard);
                }
            }
            cout << "End of Determinism constraints:" << endl;
            cout << "Asserting Symmetry constraints on updates:" << endl;

            // add the symmetry constraints for updates associated with
            // this guard
            auto it3 = GuardOpToUpdateSymmetryConstraints.find(Op);
            if (it3 != GuardOpToUpdateSymmetryConstraints.end()) {
                for (auto const& Constraint : it3->second) {
                    CheckedAssert(Constraint);
                }
            }

            cout << "End of Symmetry constraints on updates:" << endl;

            // Mark the guard and its updates as unveiled
            UnveiledGuardOps.insert(Op);
            InterpretedOps.insert(Op);
            auto it4 = GuardOpToUpdates.find(Op);
            if (it4 != GuardOpToUpdates.end()) {
                for (auto const& UpdateExp : it4->second) {
                    auto UpdateOp = UpdateExp->SAs<OpExpression>()->GetOpCode();
                    UnveiledUpdateOps.insert(UpdateOp);
                    NewlyUnveiledUpdates.insert(UpdateOp);
                    InterpretedOps.insert(UpdateOp);
                }
            }

            // Create the indicator variable for the guard
            // and assert the implication on the indicator variable
            CreateGuardIndicator(Op);

            // Create indicator variables for each newly unveiled update as well
            for (auto const& NewUpdateOp : NewlyUnveiledUpdates) {
                CreateUpdateIndicator(NewUpdateOp);
            }
            UpdateCommands();
        }

        void Solver::MakeAssertion(const ExpT& Pred)
        {
            CheckedAssert(Pred);
            auto&& SynthOps = GetSynthOps(Pred);
            for (auto const& Op : SynthOps) {
                if (UnveiledGuardOps.find(Op) != UnveiledGuardOps.end()) {
                    continue;
                }
                UnveilGuardOp(Op);
            }
        }

        inline void Solver::HandleOneDeadlockViolation(const StateVec* ErrorState)
        {
            auto Mgr = TheLTS->GetMgr();
            auto AQS = Checker->AQS;

            cout << "Handling one deadlock violation, computing shortest path... ";
            flush(cout);

            auto PPath = AQS->FindShortestPath(ErrorState, CostFunction);

            cout << "Done!" << endl << "Unwinding trace... ";
            flush(cout);

            auto Trace = TraceBase::MakeDeadlockViolation(PPath, Checker);

            cout << "Done!" << endl
                 << "Got trace with " << Trace->GetTraceElems().size() << " steps" << endl
                 << "Computing Disjuncts... ";
            flush(cout);

            // cout << "The Deadlock Trace:" << endl << Trace->ToString() << endl << endl
            //      << "The Error State:" << endl;
            // Checker->Printer->PrintState(ErrorState, cout);
            // cout << endl << endl;

            const StateVec* LastState;
            auto TraceElems = Trace->GetTraceElems();
            if (TraceElems.size() > 0) {
                LastState = TraceElems.back().second;
            } else {
                LastState = Trace->GetInitialState();
            }

            // Gather the guards of guarded commands that could
            // possibly solve this deadlock
            vector<ExpT> Disjuncts;
            for (auto const& Cmd : GuardedCommands) {
                auto const& FixedInterp = Cmd->GetFixedInterpretation();
                auto Interp = FixedInterp->ExtensionData.Interp;
                auto Res = Interp->Evaluate(LastState);
                if (Res == 0) {
                    continue;
                }
                Disjuncts.push_back(Cmd->GetGuard());
            }

            ExpT GoodExp = ExpT::NullPtr;
            auto UnreachableExp = TraceAnalyses::AutomataStatesCondition(TheLTS, LastState);
            UnreachableExp = Mgr->MakeExpr(LTSOps::OpNOT, UnreachableExp);

            // cout << "Unreachable Exp: " << endl << UnreachableExp->ToString() << endl << endl;

            Disjuncts.push_back(UnreachableExp);

            if (Disjuncts.size() == 1) {
                GoodExp = Disjuncts[0];
            } else {
                GoodExp = Mgr->MakeExpr(LTSOps::OpOR, Disjuncts);
            }

            cout << "Done!" << endl << "Computing weakest pre... ";
            flush(cout);

            auto&& WPConditions =
                TraceAnalyses::WeakestPrecondition(this,
                                                   Trace->As<SafetyViolation>(),
                                                   GoodExp);

            for (auto const& Pred : WPConditions) {
                cout << "Asserting Pre:" << endl << Pred->ToString() << endl << endl;
                MakeAssertion(Pred);
            }

            cout << "Done!" << endl;
            flush(cout);

            delete Trace;
        }

        inline void Solver::HandleSafetyViolations()
        {
            auto const& ErrorStates = Checker->GetAllErrorStates();
            auto const& DeadlockFreeInvar = Checker->DeadlockFreeInvariant;

            u32 NumDeadlocks = 0;
            u32 i = 0;
            set<ExpT> BlownInvariantsCovered;
            cout << "Found " << ErrorStates.size() << " error states in all!" << endl;
            for (auto const& ErrorState : ErrorStates) {
                auto SVPtr = ErrorState.first;
                auto const& BlownInvariant = ErrorState.second;
                bool IsDeadlock = (BlownInvariant == DeadlockFreeInvar);
                if (IsDeadlock &&
                    NumDeadlocks++ < 8) {
                    HandleOneDeadlockViolation(SVPtr);
                    ++i;
                } else if (!IsDeadlock) {
                    if (BlownInvariantsCovered.find(BlownInvariant) !=
                        BlownInvariantsCovered.end()) {
                        continue;
                    }
                    ++i;
                    BlownInvariantsCovered.insert(BlownInvariant);
                    HandleOneSafetyViolation(SVPtr, BlownInvariant);
                }
                if (i > 16) {
                    break;
                }
            }
        }

        inline void Solver::HandleLivenessViolation(const LivenessViolation* Trace,
                                                    StateBuchiAutomaton* Monitor)
        {
            auto Predicate =
                TraceAnalyses::WeakestPreconditionForLiveness(this, Monitor, Trace);
            cout << "Asserting predicate for liveness violation:" << endl
                 << Predicate->ToString() << endl << endl;
            MakeAssertion(Predicate);
            delete Trace;
        }

        inline void Solver::AssertBoundsConstraint()
        {
            auto Mgr = TheLTS->GetMgr();
            vector<ExpT> Summands;
            for (auto const& IndexExp : IndicatorExps) {
                Summands.push_back(IndexExp.second);
            }

            ExpT SumExp = ExpT::NullPtr;
            if (Summands.size() == 0) {
                return;
            } else if (Summands.size() == 1) {
                SumExp = Summands[0];
            } else {
                SumExp = Mgr->MakeExpr(LTSOps::OpADD, Summands);
            }

            auto BoundExp = Mgr->MakeVal(to_string(Bound),
                                         Mgr->MakeType<ExprRangeType>(0, Bound));
            auto EQExp = Mgr->MakeExpr(LTSOps::OpEQ, SumExp, BoundExp);
            cout << "Asserting Bounds Constraint:" << endl
                 << EQExp->ToString() << endl << endl;
            TP->Assert(EQExp, true);

            // Now make the bounds constraint for the update
            Summands.clear();
            for (auto const& IndexExp : UpdateIndicatorExps) {
                Summands.push_back(IndexExp.second);
            }
            SumExp = ExpT::NullPtr;
            if (Summands.size() == 0) {
                return;
            } else if (Summands.size() == 1) {
                SumExp = Summands[0];
            } else {
                SumExp = Mgr->MakeExpr(LTSOps::OpADD, Summands);
            }
            BoundExp = Mgr->MakeVal(to_string(UpdateBound),
                                    Mgr->MakeType<ExprRangeType>(0, UpdateBound));
            auto LEExp = Mgr->MakeExpr(LTSOps::OpLE, SumExp, BoundExp);
            cout << "Asserting Update Bounds Constraint:" << endl
                 << LEExp->ToString() << endl << endl;
            TP->Assert(LEExp, true);
        }

        // Algorithm:
        // unlocked := {}
        // bound := 0
        // while (true)
        //   success := model check
        //   if success then
        //     return completed protocol
        //   else
        //     Analyze counterexample
        //       - Safety: Add constraints
        //       - Deadlock: "unlock" additional transitions
        //                   and add constraints
        //       - Liveness: "unlock" additional transitions
        //                   and add constraints
        //     model := get an interpretation based on the constraints
        //     if model is undefined (unsat) then
        //       bound := bound + 1 and retry model generation
        //     else
        //       continue
        void Solver::Solve()
        {
            while (Bound <= LimitOnBound) {

                TP->Push();
                AssertBoundsConstraint();
                auto TPRes = TP->CheckSat();
                TP->Pop();

                if (TPRes == TPResult::UNSATISFIABLE) {
                    if (UpdateBound < Bound * UpdateBoundsMultiplier) {
                        ++UpdateBound;
                    } else {
                        UpdateBound = 0;
                        ++Bound;
                    }
                    continue;
                } else if (TPRes == TPResult::UNKNOWN) {
                    throw ESMCError((string)"Could not solve constraints!");
                }


                // all good. extract a model
                auto const& Model = TPAsZ3->GetModel();

                PrintSolution();

                Compiler->UpdateModel(Model, InterpretedOps, IndicatorExps);

                // Okay, we're good to model check now
                Checker->ClearAQS();
                auto Safe = Checker->BuildAQS(AQSConstructionMethod::BreadthFirst, 1);
                if (!Safe) {
                    HandleSafetyViolations();
                    continue;
                }

                // Safe
                bool CompletionGood = true;
                auto const& LivenessNames = Checker->GetBuchiMonitorNames();
                for (auto const& Liveness : LivenessNames) {
                    auto MonBase = Checker->AllBuchiAutomata[Liveness];
                    auto Monitor = MonBase->As<StateBuchiAutomaton>();
                    auto LiveTrace = Checker->CheckLiveness(Liveness);
                    if (LiveTrace != nullptr) {
                        HandleLivenessViolation(LiveTrace->As<LivenessViolation>(), Monitor);
                        CompletionGood = false;
                        break;
                    }
                }

                if (!CompletionGood) {
                    continue;
                } else {
                    cout << "Found Correct Completion!" << endl;
                    cout << "With Bound = " << Bound << ", Model: " << endl;
                    cout << Model.ToString() << endl;
                    return;
                }
            }

            cout << "[Solver]: Exceeded limit on number of transitions "
                 << "that were allowed to be added. Bailing..." << endl << endl;
        }

        void Solver::PrintUFModel(i64 UFCode) {
            auto Mgr = TheLTS->GetMgr();
            vector<vector<string>> CPElems;
            auto UFType = Mgr->LookupUninterpretedFunction(UFCode);
            auto UFTypeAsFunc = UFType->As<ExprFuncType>();
            auto ArgTypes = UFTypeAsFunc->GetArgTypes();
            for (auto ArgType : ArgTypes) {
                CPElems.push_back(ArgType->GetElementsNoUndef());
            }
            auto&& CPRes = CrossProduct<string>(CPElems.begin(), CPElems.end());
            for (auto ArgVector : CPRes) {
                vector<ExpT> Args;
                for (u32 i = 0; i < ArgVector.size(); ++i) {
                    auto StringArg = ArgVector[i];
                    auto ArgType = ArgTypes[i];
                    auto Arg = Mgr->MakeVal(StringArg, ArgType);
                    cout << Arg << " ";
                    Args.push_back(Arg);
                }
                auto AppExp = Mgr->MakeExpr(UFCode, Args);
                auto ModelValue = TPAsZ3->Evaluate(AppExp);
                cout << "-> " << ModelValue << endl;
            }
        }

        void Solver::PrintSolution()
        {
            auto HasUF = [&] (const ExpBaseT* Exp) -> bool
                {
                    auto ExpAsOpExp = Exp->As<OpExpression>();
                    if (ExpAsOpExp != nullptr) {
                        auto Code = ExpAsOpExp->GetOpCode();
                        return (Code >= LTSOps::UFOffset);
                    }
                    return false;
                };
            auto IsIncomplete = [&](const EFSMBase* EFSM)
                {
                    return EFSM->Is<IncompleteEFSM>();
                };
            auto Model = TPAsZ3->GetModel();
            vector<LTSTransRef> AllTransitions;
            auto IncompleteEFSMs = TheLTS->GetEFSMs(IsIncomplete);
            for (auto IncompleteEFSM : IncompleteEFSMs) {
                auto OutputMsgs = IncompleteEFSM->GetOutputs();
                for (auto OutputMsg : OutputMsgs) {
                    auto OutputTransitions =
                        IncompleteEFSM->GetOutputTransitionsOnMsg(OutputMsg);
                    for (auto OutputTransition : OutputTransitions) {
                        AllTransitions.push_back(OutputTransition);
                    }
                }
                auto InputMsgs = IncompleteEFSM->GetInputs();
                for (auto InputMsg : InputMsgs) {
                    auto InputTransitionsPerParam =
                        IncompleteEFSM->GetInputTransitionsOnMsg(InputMsg);
                    auto InputTransitions = InputTransitionsPerParam[0];
                    for (auto InputTransition : InputTransitions) {
                        AllTransitions.push_back(InputTransition);
                    }
                }
                auto InternalTransitions = IncompleteEFSM->GetInternalTransitions();
                for (auto InternalTransition : InternalTransitions) {
                    AllTransitions.push_back(InternalTransition);
                }
            }
            cout << "The solution is the following" << endl;
            for (auto NewOpIndicatorVar : IndicatorExps) {
                auto NewOp = NewOpIndicatorVar.first;
                auto IndicatorVar = NewOpIndicatorVar.second;
                auto IndicatorValue = TPAsZ3->Evaluate(IndicatorVar);
                if (IndicatorValue->ToString() == "1") {
                    for (auto Transition : AllTransitions) {
                        auto Guard = Transition->GetGuard();
                        auto UFFunctionsInGuard = Guard->GetMgr()->Gather(Guard, HasUF);
                        bool PrintTransition = false;
                        for (auto UFFunction : UFFunctionsInGuard) {
                            auto OpCode = UFFunction->As<OpExpression>()->GetOpCode();
                            if (OpCode == NewOp) {
                                PrintTransition = true;
                            }
                        }
                        if (PrintTransition) {
                            cout << Transition->ToString() << endl;
                            cout << Guard << endl;
                            PrintUFModel(NewOp);
                            for (auto Update : Transition->GetUpdates()) {
                                auto RHS = Update->GetRHS();
                                if (RHS->Is<OpExpression>()) {
                                    auto OpCode = RHS->As<OpExpression>()->GetOpCode();
                                    cout << Update->ToString() << endl;
                                    PrintUFModel(OpCode);
                                }
                            }
                        }
                    }
                }
            }
        }

    } /* end namespace Synth */
} /* end namespace ESMC */

//
// Solver.cpp ends here
