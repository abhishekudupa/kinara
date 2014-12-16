// Solver.hpp ---
//
// Filename: Solver.hpp
// Author: Abhishek Udupa
// Created: Thu Oct 23 11:07:12 2014 (-0400)
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

#if !defined ESMC_SOLVER_HPP_
#define ESMC_SOLVER_HPP_

#include <tuple>

#include "../common/ESMCFwdDecls.hpp"
#include "../containers/RefCountable.hpp"
#include "../uflts/LTSTransitions.hpp"
#include "../utils/UIDGenerator.hpp"
#include "../mc/AQStructure.hpp"
#include "../utils/TimeValue.hpp"

namespace ESMC {
    namespace Synth {

        using namespace ESMC::LTS;
        using namespace ESMC::MC;
        using namespace ESMC::TP;
        using namespace ESMC::Synth;

        extern const u64 TentativeEdgeCost;

        enum class GuardBoundingMethodT {
            PointBound, NonFalseBound, VarDepBound, NoBounding
        };

        enum class UpdateBoundingMethodT {
            PointBound, NonIdentityBound, VarDepBound, NoBounding
        };

        enum class StateUpdateBoundingMethodT {
            AllSame, VarDepBound, NoBounding
        };

        class SolverOptionsT {
        public:
            GuardBoundingMethodT GBoundMethod;
            UpdateBoundingMethodT UBoundMethod;
            StateUpdateBoundingMethodT SBoundMethod;
            bool UnrollQuantifiers;
            u64 CPULimitInSeconds;
            u64 MemLimitInMB;
            u32 NumCExToProcess;

            inline SolverOptionsT()
                : GBoundMethod(GuardBoundingMethodT::NoBounding),
                  UBoundMethod(UpdateBoundingMethodT::NoBounding),
                  SBoundMethod(StateUpdateBoundingMethodT::NoBounding),
                  UnrollQuantifiers(false), CPULimitInSeconds(UINT64_MAX),
                  MemLimitInMB(UINT64_MAX), NumCExToProcess(8)
            {
                // Nothing here
            }

            inline SolverOptionsT(const SolverOptionsT& Other)
                : GBoundMethod(Other.GBoundMethod),
                  UBoundMethod(Other.UBoundMethod),
                  SBoundMethod(Other.SBoundMethod),
                  UnrollQuantifiers(Other.UnrollQuantifiers),
                  CPULimitInSeconds(Other.CPULimitInSeconds),
                  MemLimitInMB(Other.MemLimitInMB),
                  NumCExToProcess(Other.NumCExToProcess == 0 ?
                                  UINT32_MAX : Other.NumCExToProcess)
            {
                // Nothing here
            }

            inline SolverOptionsT& operator = (const SolverOptionsT& Other)
            {
                if (&Other == this) {
                    return *this;
                }

                GBoundMethod = Other.GBoundMethod;
                SBoundMethod = Other.SBoundMethod;
                UBoundMethod = Other.UBoundMethod;
                UnrollQuantifiers = Other.UnrollQuantifiers;
                CPULimitInSeconds = Other.CPULimitInSeconds;
                MemLimitInMB = Other.MemLimitInMB;
                NumCExToProcess =
                    Other.NumCExToProcess == 0 ? UINT32_MAX : Other.NumCExToProcess;
            }
        };

        namespace Detail {

            // A cost functor for shortest paths
            class SynthCostFunction
            {
            private:
                unordered_set<u32> FixedCommands;

            public:
                inline SynthCostFunction()
                {
                    // Default (empty) constructor
                }

                inline SynthCostFunction(const unordered_set<u32>& FixedCommands)
                    : FixedCommands(FixedCommands)
                {
                    // Nothing here
                }

                ~SynthCostFunction()
                {
                    // Nothing here
                }

                inline u64 operator () (const StateVec* SVPtr, const AQSEdge* Edge) const
                {
                    auto it = FixedCommands.find(Edge->GetGCmdIndex());
                    if (it == FixedCommands.end()) {
                        return TentativeEdgeCost;
                    } else {
                        return 1;
                    }
                }
            };

            class UnorderedExpSetHasher
            {
            public:
                inline u64 operator () (const ExpT& Exp) const
                {
                    u64 Retval = 0;
                    boost::hash_combine(Retval, Exp.GetPtr_());
                    return Retval;
                }
            };

        } /* end namespace Detail */


        struct SolverStatsT
        {
            TimeValue SolveStartTime;
            TimeValue SolveEndTime;
            u64 InitialNumAssertions;
            u64 FinalNumAssertions;
            u64 NumIterations;
            u64 TotalSMTTime;
            u64 MinSMTTime;
            u64 MaxSMTTime;

            inline SolverStatsT()
                : InitialNumAssertions(0), FinalNumAssertions(0),
                  NumIterations(0), TotalSMTTime(0), MinSMTTime(0),
                  MaxSMTTime(0)
            {
                // Nothing here
            }
        };

        class Solver
        {
            friend class ESMC::Analyses::TraceAnalyses;

        private:
            static const string BoundsVarPrefix;
            static const u32 MaxBound;

            SolverOptionsT Options;

            // Assertions in the current iteration
            // Barring the cost bounds assertions
            FastExpSetT CurrentAssertions;
            deque<Z3Expr> CurrentAssumptions;
            ExpT BoundsVariable;
            Z3TPRef TP;
            LabelledTS* TheLTS;
            LTSCompiler* Compiler;
            LTSChecker* Checker;
            u32 Bound;
            vector<GCmdRef> GuardedCommands;
            Detail::SynthCostFunction CostFunction;

            unordered_set<i64> UnveiledGuardOps;
            unordered_set<i64> UnveiledUpdateOps;
            bool UnveiledNewOps;
            // Union of the two sets above, maintained
            // for efficiency
            unordered_set<i64> InterpretedOps;

            // The set of assertions already asserted
            unordered_set<ExpT, Detail::UnorderedExpSetHasher> AssertedConstraints;

            unordered_map<i64, ExpT> GuardIndicatorExps;
            unordered_map<i64, ExpT> UpdateIndicatorExps;
            Z3Ctx Ctx;
            UIDGenerator GuardIndicatorUIDGenerator;
            UIDGenerator UpdateIndicatorUIDGenerator;
            UIDGenerator IdentityUpdateUIDGenerator;
            UIDGenerator VarDepIndicatorUIDGenerator;
            UIDGenerator FunctionCostUIDGenerator;
            UIDGenerator GuardPointUIDGenerator;
            UIDGenerator UpdatePointUIDGenerator;
            UIDGenerator AllFalseUIDGenerator;
            FastExpSetT AllIndicators;

            SolverStatsT Stats;

            inline void CheckedAssert(const ExpT& Assertion);
            inline void AssertCurrentConstraints();

            inline tuple<ExpT, ExpT, ExpT, ExpT>
            CreateIndicatorSubsts(vector<TypeRef>& ExistsQVars,
                                  const ExpT& UpdateExp,
                                  const ExpT& CurrentArg);
            inline vector<ExpT>
            CreateArgDepConstraints(const ExpT& OpExpression,
                                    const ExpT& IdentityIndicatorExp);
            inline void CreateIndicators(const ExpT& OpExpression, const ExpT& LValueExp,
                                         bool IsGuard);
            inline void CreateIndicators(i64 OpCode);

            inline void MakeStateIdenticalConstraints(const ExpT& Exp);

            inline void CreateGuardIndicator(i64 GuardOp);
            inline void CreateUpdatePointBounds(i64 UpdateOp);
            inline void CreateUpdateIndicator(i64 UpdateOp);
            inline void CreateBoundsConstraints(i64 UpdateOp);
            inline void CreateMutualExclusionConstraint(const ExpT& GuardExp1,
                                                        const ExpT& GuardExp2);
            inline void AssertBoundsConstraint();
            inline void HandleSafetyViolations();
            inline void HandleOneSafetyViolation(const StateVec* ErrorState,
                                                 const ExpT& BlownInvariant);
            inline void HandleOneDeadlockViolation(const StateVec* ErrorState);
            inline void HandleLivenessViolation(const LivenessViolation* Trace,
                                                StateBuchiAutomaton* Monitor);
            inline void UpdateCommands();
            inline void ResetStats();
            inline void PrintStats(ostream& Out);

        public:
            Solver(LTSChecker* Checker, const SolverOptionsT& Options = SolverOptionsT());
            virtual ~Solver();

            // makes an assertion. Also fixes up interpretations
            // and marks the appropriate set of commands as having a
            // fixed interpretation.
            void MakeAssertion(const ExpT& Pred);
            void UnveilGuardOp(i64 Op);
            void UnveilNonCompletionGuardOp(i64 Op);
            void UnveilNonCompletionOp(i64 Op);
            void Solve();
            void PrintUFModel(i64 UFCode);
            void PrintSolution();
            ExpT Evaluate(const ExpT& Input);
            void PrintOneUFFinalSolution(const vector<const UFInterpreter*>& Interps,
                                         ostream& Out);
            void PrintFinalSolution(ostream& Out);
            const SolverStatsT& GetStats() const;
        };

    } /* end namespace Synth */
} /* end namespace ESMC */

#endif /* ESMC_SOLVER_HPP_ */

//
// Solver.hpp ends here
