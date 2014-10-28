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

#include "Solver.hpp"

namespace ESMC {
    namespace Synth {
        
        using namespace ESMC::LTS;
        using namespace ESMC::MC;
        using namespace ESMC::TP;
        using namespace ESMC::Analyses;

        using LTS::ExpT;

        const u64 TentativeEdgeCost = ((u64)1 << 30);

        const string Solver::BoundsVarPrefix = (string)"__SynthBound__";

        Solver::Solver(LTSChecker* Checker)
            : TP(new Z3TheoremProver()), TheLTS(Checker->TheLTS), 
              Compiler(Checker->Compiler), Bound(0), 
              GuardedCommands(TheLTS->GetGuardedCmds())
        {
            // push a scope onto the theorem prover and assert
            // true
            TP->Push();
            TP->Assert(TheLTS->MakeTrue());
            
            TPAsZ3 = const_cast<Z3TheoremProver*>(TP->As<Z3TheoremProver>());
            Ctx = TPAsZ3->GetCtx();
        }

        Solver::~Solver()
        {
            // Nothing here
        }

        inline void Solver::HandleOneSafetyViolation(const StateVec *ErrorState, 
                                                     const ExpT& BlownInvariant)
        {
            Detail::SynthCostFunction CostFunc(EnabledCommands);
            auto AQS = Checker->AQS;
            auto PPath = AQS->FindShortestPath(ErrorState, CostFunc);
            auto Trace = TraceBase::MakeSafetyViolation(PPath, Checker, BlownInvariant);

            // Palm off to the analyses engine
            delete Trace;
        }

        inline void Solver::MakeAssertion(const ExpT& Pred)
        {
            auto Mgr = TheLTS->GetMgr();
            TP->Assert(Pred);
            auto&& SynthOps = GetSynthOps(Pred);

            // Find the structural constraints that need to be added
            // for the newly exposed operations
            set<i64> NewGuardOps;
            auto const& ConstraintsByOp = TheLTS->GetConstraintsByOp();
            for (auto const& SynthOp : SynthOps) {
                if (InterpretedOps.find(SynthOp) != InterpretedOps.end()) {
                    // We've already interpreted these,
                    // no need to add structural constraints
                    continue;
                }
                InterpretedOps.insert(SynthOp);

                auto it = ConstraintsByOp.find(SynthOp);
                if (it == ConstraintsByOp.end()) {
                    continue;
                }

                // This is a new guard op
                NewGuardOps.insert(SynthOp);

                auto const& Constraints = it->second;

                for (auto const& Pred : Constraints) {
                    // Any synth ops in the predicate are also interpreted
                    auto&& PredOps = GetSynthOps(Pred);
                    for (auto const& PredOp : PredOps) {
                        InterpretedOps.insert(PredOp);
                    }
                    TP->Assert(Pred);
                }
            }
            
            // Mark all guards that have a full interpretation as such
            for (auto const& Cmd : GuardedCommands) {
                auto const& Guard = Cmd->GetGuard();
                auto&& SynthOps = GetSynthOps(Guard);
                bool FullyInterpreted = true;
                for (auto const& Op : SynthOps) {
                    if (InterpretedOps.find(Op) == InterpretedOps.end()) {
                        FullyInterpreted = false;
                        break;
                    }
                }
                EnabledCommands.insert(Cmd->GetCmdID());
                Cmd->SetFullyInterpreted(FullyInterpreted);
            }

            // Create indicator variables for the new guards
            // and assert constraints on those as well
            for (auto const& NewOp : NewGuardOps) {
                auto IndicatorUID = IndicatorUIDGenerator.GetUID();
                string IndicatorVarName = (string)"__indicator_" + to_string(IndicatorUID);
                auto FuncType = Mgr->LookupUninterpretedFunction(NewOp)->As<ExprFuncType>();
                auto const& DomainTypes = FuncType->GetArgTypes();
                vector<ExpT> BoundArgs;
                const u32 NumDomainTypes = DomainTypes.size();
                for (u32 i = 0; i < NumDomainTypes; ++i) {
                    auto BoundVar = Mgr->MakeBoundVar(DomainTypes[i], NumDomainTypes - i - 1);
                    BoundArgs.push_back(BoundVar);
                    i++;
                }
                auto AppExp = Mgr->MakeExpr(NewOp, BoundArgs);
                auto ExistsExp = Mgr->MakeExists(DomainTypes, AppExp);
                auto IndicatorType = Mgr->MakeType<ExprRangeType>(0, 1);
                
                auto IndicatorVar = Mgr->MakeVar(IndicatorVarName, IndicatorType);
                IndicatorExps[NewOp] = IndicatorVar;

                auto Implies = Mgr->MakeExpr(LTSOps::OpIMPLIES, ExistsExp,
                                             Mgr->MakeExpr(LTSOps::OpEQ, IndicatorVar,
                                                           Mgr->MakeVal("1", IndicatorType)));
                TP->Assert(Implies);
            }
        }

        inline void Solver::HandleOneDeadlockViolation(const StateVec* ErrorState)
        {
            auto Mgr = TheLTS->GetMgr();
            Detail::SynthCostFunction CostFunc(EnabledCommands);
            auto AQS = Checker->AQS;
            auto PPath = AQS->FindShortestPath(ErrorState, CostFunc);
            auto Trace = TraceBase::MakeDeadlockViolation(PPath, Checker);

            // Gather the guards of guarded commands that could
            // possibly solve this deadlock
            vector<ExpT> Disjuncts;
            for (auto const& Cmd : GuardedCommands) {
                if (!Cmd->IsTentative()) {
                    continue;
                }

                auto const& FixedInterp = Cmd->GetFixedInterpretation();
                auto Interp = FixedInterp->ExtensionData.Interp;
                auto Res = Interp->Evaluate(ErrorState);
                if (Res == 0) {
                    continue;
                }
                Disjuncts.push_back(Cmd->GetGuard());
            }

            ExpT GoodExp = ExpT::NullPtr;
            if (Disjuncts.size() == 0) {
                throw InternalError((string)"Could not find any commands to enable to " + 
                                    "fix deadlock!\nAt: " + __FILE__ + ":" + to_string(__LINE__));
            } else if (Disjuncts.size() == 1) {
                GoodExp = Disjuncts[0];
            } else {
                GoodExp = Mgr->MakeExpr(LTSOps::OpOR, Disjuncts);
            }

            auto&& WPConditions = 
                TraceAnalyses::WeakestPrecondition(TheLTS, 
                                                   Trace->As<SafetyViolation>(), 
                                                   GoodExp);

            for (auto const& Pred : WPConditions) {
                MakeAssertion(Pred);
            }

            delete Trace;
        }

        inline void Solver::HandleSafetyViolations()
        {
            auto const& ErrorStates = Checker->GetAllErrorStates();
            auto const& DeadlockFreeInvar = Checker->DeadlockFreeInvariant;
            
            for (auto const& ErrorState : ErrorStates) {
                auto SVPtr = ErrorState.first;
                auto const& BlownInvariant = ErrorState.second;

                if (BlownInvariant == DeadlockFreeInvar) {
                    HandleOneDeadlockViolation(SVPtr);
                } else {
                    HandleOneSafetyViolation(SVPtr, BlownInvariant);
                }
            }
        }

        inline void Solver::HandleLivenessViolation(const LivenessViolation* Trace)
        {
            return;
        }

        inline void Solver::AssertBoundsConstraint()
        {
            auto Mgr = TheLTS->GetMgr();
            vector<ExpT> Summands;
            for (auto const& IndexExp : IndicatorExps) {
                Summands.push_back(IndexExp.second);
            }

            auto SumExp = Mgr->MakeExpr(LTSOps::OpAND, Summands);
            auto BoundExp = Mgr->MakeVal(to_string(Bound), 
                                         Mgr->MakeType<ExprRangeType>(0, Bound));
            auto EQExp = Mgr->MakeExpr(LTSOps::OpEQ, SumExp, BoundExp);
            TP->Assert(EQExp);
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
            while (true) {

                TP->Push();
                AssertBoundsConstraint();
                auto TPRes = TP->CheckSat();
                TP->Pop();

                if (TPRes == TPResult::UNSATISFIABLE) {
                    // TODO: Bound the bound
                    ++Bound;
                    continue;
                } else if (TPRes == TPResult::UNKNOWN) {
                    throw ESMCError((string)"Could not solve constraints!");
                }

                // all good. extract a model
                auto const& Model = TPAsZ3->GetModel();

                Compiler->UpdateModel(Model, InterpretedOps, IndicatorExps);

                // Okay, we're good to model check now
                Checker->ClearAQS();
                auto Safe = Checker->BuildAQS();
                if (!Safe) {
                    HandleSafetyViolations();
                    continue;
                }
                
                // Safe
                bool CompletionGood = true;
                auto const& LivenessNames = Checker->GetBuchiMonitorNames();
                for (auto const& Liveness : LivenessNames) {
                    auto LiveTrace = Checker->CheckLiveness(Liveness);
                    if (LiveTrace != nullptr) {
                        HandleLivenessViolation(LiveTrace->As<LivenessViolation>());
                        CompletionGood = false;
                        break;
                    }
                }

                if (!CompletionGood) {
                    continue;
                } else {
                    cout << "Found Correct Completion!" << endl;
                    cout << Model.ToString() << endl;
                }
            }
        }

    } /* end namespace Synth */
} /* end namespace ESMC */

// 
// Solver.cpp ends here
