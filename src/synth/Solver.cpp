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
#include "../mc/LTSChecker.hpp"

#include "Solver.hpp"

namespace ESMC {
    namespace Synth {
        
        using namespace ESMC::LTS;
        using namespace ESMC::MC;
        using namespace ESMC::TP;

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
            
            BoundExpr = Z3Expr(Ctx, Z3_mk_int(*Ctx, 0, Z3_mk_int_sort(*Ctx)));
            auto BoundsVarName = 
                BoundsVarPrefix + to_string(BoundsVarUIDGenerator.GetUID());
            auto BoundsVarSym = Z3_mk_string_symbol(*Ctx, BoundsVarName.c_str());
            BoundsVar = Z3Expr(Ctx, Z3_mk_const(*Ctx, BoundsVarSym, Z3_mk_int_sort(*Ctx)));
        }

        inline void Solver::AssertBoundsConstraint()
        {
            TP->Push();
            TPAsZ3->Assert(Z3Expr(Ctx, Z3_mk_eq(*Ctx, BoundsVar, BoundExpr)));
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
            
            // TODO:
            // 1. Palm the trace off to the analysis engine
            // 2. Add constraints and unlock commands as required
            
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
                AssertBoundsConstraint();
                auto TPRes = TP->CheckSat();
                if (TPRes == TPResult::UNSATISFIABLE) {
                    // TODO: Handle unsat by increasing bounds
                } else if (TPRes == TPResult::UNKNOWN) {
                    throw ESMCError((string)"Could not solve constraints!");
                }

                // all good. extract a model
                auto const& Model = TPAsZ3->GetModel();
                TP->Pop();

                Compiler->UpdateModel(Model, InterpretedOps);

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
