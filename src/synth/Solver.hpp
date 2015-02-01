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

enum class GuardBoundingMethodT
{
    PointBound, NonFalseBound, VarDepBound, NoBounding
};

enum class UpdateBoundingMethodT
{
    PointBound, NonIdentityBound, VarDepBound, NoBounding
};

enum class StateUpdateBoundingMethodT
{
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
    float DesiredCoverage;
    u32 BoundLimit;
    bool GeneralFixForDL;
    bool ResetTPOnBoundsBump;
    BFSPrioMethodT BFSPrioMethod;
    u32 IncSolverTimeout;

    inline SolverOptionsT()
        : GBoundMethod(GuardBoundingMethodT::NoBounding),
          UBoundMethod(UpdateBoundingMethodT::NoBounding),
          SBoundMethod(StateUpdateBoundingMethodT::NoBounding),
          UnrollQuantifiers(false), CPULimitInSeconds(UINT64_MAX),
          MemLimitInMB(UINT64_MAX), DesiredCoverage(1),
          BoundLimit(256), GeneralFixForDL(false),
          ResetTPOnBoundsBump(false),
          BFSPrioMethod(BFSPrioMethodT::None), IncSolverTimeout(UINT32_MAX)
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
          DesiredCoverage(Other.DesiredCoverage == 0.0 ?
                          FLT_MAX : Other.DesiredCoverage),
          BoundLimit(Other.BoundLimit == 0 ? 256 : Other.BoundLimit),
          GeneralFixForDL(Other.GeneralFixForDL),
          ResetTPOnBoundsBump(Other.ResetTPOnBoundsBump),
          BFSPrioMethod(Other.BFSPrioMethod),
          IncSolverTimeout(Other.IncSolverTimeout)
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
        DesiredCoverage =
            Other.DesiredCoverage == 0.0 ? FLT_MAX : Other.DesiredCoverage;
        BoundLimit = Other.BoundLimit == 0 ? 256 : Other.BoundLimit;
        GeneralFixForDL = Other.GeneralFixForDL;
        ResetTPOnBoundsBump = Other.ResetTPOnBoundsBump;
        BFSPrioMethod = Other.BFSPrioMethod;
        IncSolverTimeout = Other.IncSolverTimeout;
        return *this;
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

    SolverOptionsT Options;
    // The set of assertions already asserted
    FastExpSetT AssertedConstraintSet;
    vector<ExpT> AssertedConstraints;
    // Barring the cost bounds assertions, which are in the
    // assumptions queue here:
    deque<Z3Expr> CurrentAssumptions;
    vector<Z3Expr> AllBoundsAssumptions;
    Z3TPRef TP;
    LabelledTS* TheLTS;
    LTSCompiler* Compiler;
    LTSChecker* Checker;
    u32 Bound;
    ExpT BoundsVariable;
    vector<GCmdRef> GuardedCommands;
    Detail::SynthCostFunction CostFunction;

    unordered_set<i64> UnveiledGuardOps;
    unordered_set<i64> UnveiledUpdateOps;
    bool UnveiledNewOps;
    // Union of the two sets above, maintained
    // for efficiency
    unordered_set<i64> InterpretedOps;


    vector<ExpT> GuardFuncCosts;
    vector<ExpT> UpdateFuncCosts;
    // AllFalse preds for guards
    // used for updating the models in the compiler/interpreter
    unordered_map<i64, ExpT> AllFalsePreds;

    Z3Ctx Ctx;
    SolverStatsT Stats;

    inline void CheckedAssert(const ExpT& Assertion);
    inline void CreateMutualExclusionConstraint(const ExpT& GuardExp1,
                                                const ExpT& GuardExp2);
    inline void AssertBoundsConstraint(u32 CurrentBound);
    inline void HandleSafetyViolations();
    inline void HandleOneSafetyViolation(const StateVec* ErrorState,
                                         const ExpT& BlownInvariant);
    inline void HandleOneDeadlockViolation(const StateVec* ErrorState);
    inline void HandleLivenessViolation(const LivenessViolation* Trace,
                                        StateBuchiAutomaton* Monitor);
    inline void UpdateCommands();
    inline void ResetStats();
    inline void PrintStats();
    inline void HandleResourceLimit();

    // Returns the set of update ops associated with the guard
    inline unordered_set<i64> AssertConstraintsForNewGuard(i64 GuardOp);

    // returns predicate for allfalse and alltrue
    inline pair<ExpT, ExpT> MakeAllFalseConstraint(i64 GuardOp,
                                                   const ExpT& GuardExp,
                                                   bool MakeAllTrue);
    inline ExpT MakeIdentityConstraint(i64 Op, const ExpT& UpdateExp,
                                       u32 LValueIndex);
    inline ExpT MakeConstantConstraint(i64 Op, const ExpT& Exp);
    inline tuple<ExpT, ExpT, ExpT, ExpT>
    CreateArgDepSubsts(vector<TypeRef>& QVars,
                       const ExpT& UpdateExp,
                       const ExpT& CurrentArg);
    inline ExpT MakeArgDepConstraints(i64 OpCode, const ExpT& Exp);
    inline vector<ExpT> MakePointApplications(i64 OpCode);

    inline void MakeArgDepCCForGuard(i64 Op, const ExpT& Exp);
    inline void MakeNonFalseCCForGuard(i64 Op, const ExpT& Exp);
    inline void MakePointCCForGuard(i64 Op, const ExpT& Exp);

    inline void MakeArgDepCCForLValUpdate(i64 Op, const ExpT& Exp,
                                          const ExpT& LValueExp);
    inline void MakeIdentityCCForLValUpdate(i64 Op, const ExpT& Exp,
                                            const ExpT& LValueExp);
    inline void MakePointCCForLValUpdate(i64 Op, const ExpT& Exp,
                                         const ExpT& LValueExp);

    inline void MakeCostConstraintsForLValueUpdate(i64 Op, const ExpT& UpdateExp,
                                                   const ExpT& LValueExp);
    inline void MakeCostConstraintsForStateUpdate(i64 Op, const ExpT& UpdateExp);
    inline void MakeCostConstraintsForNonLValueUpdate(i64 Op, const ExpT& UpdateExp);
    inline void MakeCostConstraintsForGuard(i64 Op, const ExpT& Exp);
    inline void MakeCostConstraintsForOp(i64 Op);
    inline void MakeRangeConstraintsForOp(i64 UpdateOp);
    inline void HandleTPReset();

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
