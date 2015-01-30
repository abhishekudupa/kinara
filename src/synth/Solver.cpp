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

#include <boost/algorithm/string/predicate.hpp>

#include "../tpinterface/TheoremProver.hpp"
#include "../mc/Trace.hpp"
#include "../mc/Compiler.hpp"
#include "../uflts/LabelledTS.hpp"
#include "../uflts/LTSDecls.hpp"
#include "../uflts/LTSEFSM.hpp"
#include "../uflts/LTSUtils.hpp"
#include "../mc/LTSChecker.hpp"
#include "../symexec/LTSAnalyses.hpp"
#include "../mc/OmegaAutomaton.hpp"
#include "../mc/StateVecPrinter.hpp"
#include "../utils/ResourceLimitManager.hpp"

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

        Solver::Solver(LTSChecker* Checker, const SolverOptionsT& Options)
            : Options(Options),
              TP(new Z3TheoremProver(Options.IncSolverTimeout)),
              TheLTS(Checker->TheLTS),
              Compiler(Checker->Compiler),
              Checker(Checker),
              Bound(0),
              GuardedCommands(TheLTS->GetGuardedCmds()),
              UnveiledNewOps(false)
        {
            Ctx = TP->GetCtx();

            // Populate the set of fixed commands and create the cost function
            unordered_set<u32> FixedCommands;
            for (auto const& Cmd : GuardedCommands) {
                auto const& SynthOps = GetSynthOps(Cmd->GetGuard());
                if (SynthOps.size() == 0) {
                    FixedCommands.insert(Cmd->GetCmdID());
                }
            }

            CostFunction = Detail::SynthCostFunction(FixedCommands);

            auto Mgr = TheLTS->GetMgr();

            auto BoundsType = Mgr->MakeType<RangeType>(0, Options.BoundLimit);
            BoundsVariable = Mgr->MakeVar(BoundsVarPrefix, BoundsType);

            if (Options.UBoundMethod != UpdateBoundingMethodT::NoBounding ||
                Options.GBoundMethod != GuardBoundingMethodT::NoBounding ||
                (Options.SBoundMethod != StateUpdateBoundingMethodT::NoBounding &&
                 Options.SBoundMethod != StateUpdateBoundingMethodT::AllSame)) {
                // initialize the bounds assertions
                for (u32 i = 0; i < Options.BoundLimit; ++i) {
                    auto CurBoundsVal = Mgr->MakeVal(to_string(i), BoundsType);
                    auto CurProp = Mgr->MakeVar("__assumption_prop_var_" + to_string(i),
                                                Mgr->MakeType<BooleanType>());
                    LTS::LTSLCRef LTSCtx = new LTS::LTSLoweredContext(Ctx);
                    auto LoweredProp = Mgr->LowerExpr(CurProp, LTSCtx);
                    CheckedAssert(Mgr->MakeExpr(LTSOps::OpIMPLIES, CurProp,
                                                Mgr->MakeExpr(LTSOps::OpLE,
                                                              BoundsVariable,
                                                              CurBoundsVal)));
                    CurrentAssumptions.push_back(LoweredProp);
                }
            }
        }

        inline void Solver::CheckedAssert(const ExpT& Assertion)
        {
            // Check for non-existence of stray variables
            auto Mgr = TheLTS->GetMgr();
            ExpT FinalAssertion = ExpT::NullPtr;
            auto&& Vars = Mgr->Gather(Assertion,
                                      [&] (const ExpPtrT Exp) -> bool
                                      {
                                          auto ExpAsVar = Exp->As<VarExpression>();
                                          if (ExpAsVar == nullptr) {
                                              return false;
                                          }
                                          auto const& VarName = ExpAsVar->GetVarName();
                                          if (boost::algorithm::starts_with(VarName, "__")) {
                                              return false;
                                          }
                                          return true;
                                      });
            if (Vars.size() > 0) {
                FinalAssertion =
                    Mgr->ApplyTransform<LTS::Detail::ConstraintPurifier>(Assertion);

                ESMC_LOG_FULL(
                              "Solver.Purification",
                              Out_ << "Purified Expression:" << endl << Assertion << endl;
                              Out_ << "To:" << endl << FinalAssertion << endl;
                              );
            } else {
                FinalAssertion = Assertion;
            }

            if (AssertedConstraintSet.find(FinalAssertion) !=
                AssertedConstraintSet.end()) {
                if (FinalAssertion != TheLTS->MakeTrue()) {
                    ESMC_LOG_FULL(
                                  "Solver.Duplicates",
                                  Out_ << "Not asserting duplicate constraint:"
                                       << endl << FinalAssertion->ToString() << endl;
                                  );
                }
                return;
            }

            AssertedConstraintSet.insert(FinalAssertion);
            AssertedConstraints.push_back(FinalAssertion);
            ESMC_LOG_FULL(
                          "Solver.Duplicates",
                          Out_ << "Asserting (non-duplicate): "
                               << Assertion->ToString() << endl;
                          );

            TP->Assert(FinalAssertion, true, Options.UnrollQuantifiers);
        }

        Solver::~Solver()
        {
            // Nothing here
        }

        inline void Solver::CreateMutualExclusionConstraint(const ExpT& GuardExp1,
                                                            const ExpT& GuardExp2)
        {
            auto Mgr = TheLTS->GetMgr();

            vector<ExpT> Args;
            FastExpSetT ArgSet;
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

            // All this hoopla to make things deterministic and
            // not dependent on the particular pointer values used
            for (auto const& Arg : ArgsMine) {
                if (ArgSet.find(Arg) == ArgSet.end()) {
                    ArgSet.insert(Arg);
                    Args.push_back(Arg);
                }
            }

            for (auto const& Arg : ArgsOther) {
                if (ArgSet.find(Arg) == ArgSet.end()) {
                    ArgSet.insert(Arg);
                    Args.push_back(Arg);
                }
            }
            vector<ExpT> QVars(Args.begin(), Args.end());
            vector<TypeRef> QVarTypes;

            auto MutexExp = Mgr->MakeExpr(LTSOps::OpAND, GuardExp1, GuardExp2);
            MutexExp = Mgr->MakeExpr(LTSOps::OpNOT, MutexExp);

            transform(QVars.begin(), QVars.end(), back_inserter(QVarTypes),
                      [&] (const ExpT& Exp) -> TypeRef
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

            ESMC_LOG_SHORT(
                           "Solver.OtherAssertions",
                           Out_ << "Determinism constraint:" << endl
                                << Constraint << endl;
                           );

            CheckedAssert(Constraint);
        }

        inline void Solver::MakeRangeConstraintsForOp(i64 UpdateOp)
        {
            auto Mgr = TheLTS->GetMgr();
            auto FunType = Mgr->LookupUninterpretedFunction(UpdateOp)->As<FuncType>();
            auto RType = FunType->GetEvalType();
            auto RTypeAsRange = RType->As<RangeType>();
            if (RTypeAsRange == nullptr) {
                return;
            }

            auto High = RTypeAsRange->GetHigh();
            auto Low = RTypeAsRange->GetLow();

            auto HighVal = Mgr->MakeVal(to_string(High), RType);
            auto LowVal = Mgr->MakeVal(to_string(Low), RType);

            // We need to assert constraints
            auto const& DomTypes = FunType->GetArgTypes();
            const u32 NumArgs = DomTypes.size();
            vector<ExpT> FunArgs(NumArgs);
            for (u32 i = 0; i < NumArgs; ++i) {
                FunArgs[i] = Mgr->MakeBoundVar(DomTypes[i], NumArgs - i - 1);
            }

            auto AppExp = Mgr->MakeExpr(UpdateOp, FunArgs);
            auto LEExp = Mgr->MakeExpr(LTSOps::OpLE, AppExp, HighVal);
            auto GEExp = Mgr->MakeExpr(LTSOps::OpGE, AppExp, LowVal);

            auto QBody = Mgr->MakeExpr(LTSOps::OpAND, LEExp, GEExp);
            auto QExpr = Mgr->MakeForAll(DomTypes, QBody);
            CheckedAssert(QExpr);
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

        // Returns the set of update ops associated with the guard
        inline unordered_set<i64>
        Solver::AssertConstraintsForNewGuard(i64 GuardOp)
        {
            auto const& GuardOpToExp = TheLTS->GuardOpToExp;
            auto const& GuardSymmetryConstraints = TheLTS->GuardSymmetryConstraints;
            auto const& GuardMutualExclusiveSets = TheLTS->GuardMutualExclusiveSets;
            auto const& GuardOpToUpdates = TheLTS->GuardOpToUpdates;
            auto const& GuardOpToUpdateSymmetryConstraints =
                TheLTS->GuardOpToUpdateSymmetryConstraints;

            unordered_set<i64> NewlyUnveiledUpdates;

            auto ExpIt = GuardOpToExp.find(GuardOp);
            if (ExpIt == GuardOpToExp.end()) {
                throw InternalError((string)"Could not find expression for op: " +
                                    to_string(GuardOp) + "\nAt: " + __FILE__ + ":" +
                                    to_string(__LINE__));
            }

            auto GuardExp = ExpIt->second;

            ESMC_LOG_FULL(
                          "Solver.OtherAssertions",
                          Out_ << "Unveiling Guard Exp: " << GuardExp << endl;
                          Out_ << "Asserting Symmetry constraints:" << endl;
                          );

            // This is a new guard
            // Assert the symmetry constraints
            auto it = GuardSymmetryConstraints.find(GuardOp);
            if (it != GuardSymmetryConstraints.end()) {
                for (auto const& Constraint : it->second) {
                    ESMC_LOG_SHORT(
                                   "Solver.OtherAssertions",
                                   Out_ << "Symmetry constraint:" << endl
                                        << Constraint << endl;
                                   );
                    CheckedAssert(Constraint);
                }
            }

            ESMC_LOG_SHORT(
                           "Solver.OtherAssertions",
                           Out_ << "End of Symmetry constraints:" << endl;
                           Out_ << "Asserting Determinism constraints:" << endl;
                           );

            // Assert the determinism constraints wrt guards
            // that have already been unveiled
            auto it2 = GuardMutualExclusiveSets.find(GuardOp);
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

            ESMC_LOG_SHORT(
                           "Solver.OtherAssertions",
                           Out_ << "End of Determinism constraints:" << endl;
                           Out_ << "Asserting Symmetry constraints on updates:" << endl;
                           );

            // add the symmetry constraints for updates associated with
            // this guard
            auto it3 = GuardOpToUpdateSymmetryConstraints.find(GuardOp);
            if (it3 != GuardOpToUpdateSymmetryConstraints.end()) {
                for (auto const& Constraint : it3->second) {

                    ESMC_LOG_SHORT(
                                   "Solver.OtherAssertions",
                                   Out_ << Constraint->ToString() << endl;
                                   );

                    CheckedAssert(Constraint);
                }
            }

            ESMC_LOG_SHORT(
                           "Solver.OtherAssertions",
                           Out_ << "End of Symmetry constraints on updates:" << endl;
                           );

            // Mark the guard and its updates as unveiled
            UnveiledGuardOps.insert(GuardOp);
            UnveiledNewOps = true;
            InterpretedOps.insert(GuardOp);

            auto it4 = GuardOpToUpdates.find(GuardOp);
            if (it4 != GuardOpToUpdates.end()) {
                for (auto const& UpdateExp : it4->second) {
                    auto UpdateOp = UpdateExp->SAs<OpExpression>()->GetOpCode();
                    UnveiledUpdateOps.insert(UpdateOp);
                    NewlyUnveiledUpdates.insert(UpdateOp);
                    InterpretedOps.insert(UpdateOp);
                }
            }

            return NewlyUnveiledUpdates;
        }

        inline pair<ExpT, ExpT>
        Solver::MakeAllFalseConstraint(i64 GuardOp,
                                       const ExpT& GuardExp,
                                       bool MakeAllTrue)
        {
            auto Mgr = TheLTS->GetMgr();
            auto const& Args = GetOpArgs(GuardExp);
            const u32 NumArgs = Args.size();
            auto const& FunType = Mgr->LookupUninterpretedFunction(GuardOp)->As<FuncType>();
            auto const& DomTypes = FunType->GetArgTypes();
            MgrT::SubstMapT SubstMap;
            for (u32 i = 0; i < NumArgs; ++i) {
                SubstMap[Args[i]] = Mgr->MakeBoundVar(DomTypes[i], NumArgs - i - 1);
            }
            auto Body = Mgr->BoundSubstitute(SubstMap, GuardExp);
            auto NotBody = Mgr->MakeExpr(LTSOps::OpNOT, Body);

            auto AllFalseConstraint = Mgr->MakeForAll(DomTypes, NotBody);
            auto AllTrueConstraint = Mgr->MakeForAll(DomTypes, Body);

            auto AllFalsePred = Mgr->MakeVar((string)"__allfalse_" + to_string(GuardOp),
                                             Mgr->MakeType<BooleanType>());
            auto AllTruePred = Mgr->MakeVar((string)"__alltrue_" + to_string(GuardOp),
                                            Mgr->MakeType<BooleanType>());

            auto IffFalse = Mgr->MakeExpr(LTSOps::OpIFF, AllFalseConstraint, AllFalsePred);
            auto IffTrue = Mgr->MakeExpr(LTSOps::OpIFF, AllTrueConstraint, AllTruePred);
            CheckedAssert(IffFalse);
            if (MakeAllTrue) {
                CheckedAssert(IffTrue);
                return make_pair(AllFalsePred, AllTruePred);
            } else {
                return make_pair(AllFalsePred, ExpT::NullPtr);
            }
        }

        inline vector<ExpT> Solver::MakePointApplications(i64 OpCode)
        {
            auto Mgr = TheLTS->GetMgr();
            auto FunType = Mgr->LookupUninterpretedFunction(OpCode)->As<FuncType>();
            auto const& ArgTypes = FunType->GetArgTypes();
            const u32 NumArgs = ArgTypes.size();

            vector<vector<string>> ArgValues;
            for (auto const& ArgType : ArgTypes) {
                ArgValues.push_back(ArgType->GetElements());
            }
            auto&& CPTuples = CrossProduct<string>(ArgValues.begin(), ArgValues.end());
            const u32 NumPoints = CPTuples.size();
            vector<ExpT> Retval(NumPoints);
            for (u32 j = 0; j < NumPoints; ++j) {
                auto const& Tuple = CPTuples[j];
                vector<ExpT> AppArgs(NumArgs);
                for (u32 i = 0; i < NumArgs; ++i) {
                    AppArgs[i] = Mgr->MakeVal(Tuple[i], ArgTypes[i]);
                }
                auto AppExp = Mgr->MakeExpr(OpCode, AppArgs);
                Retval[j] = AppExp;
            }
            return Retval;
        }

        inline tuple<ExpT, ExpT, ExpT, ExpT>
        Solver::CreateArgDepSubsts(vector<TypeRef>& QVars,
                                   const ExpT& UpdateExp,
                                   const ExpT& CurrentArg)
        {
            auto Mgr = TheLTS->GetMgr();

            auto const& UpdateArgs = GetOpArgs(UpdateExp);
            vector<ExpT> OtherArgs;
            for (auto const& UpdateArg : UpdateArgs) {
                if (UpdateArg != CurrentArg) {
                    OtherArgs.push_back(UpdateArg);
                }
            }
            const u32 NumOtherArgs = OtherArgs.size();
            vector<TypeRef> OtherArgTypes;
            transform(OtherArgs.begin(), OtherArgs.end(), back_inserter(OtherArgTypes),
                      [&] (const ExpT& Exp) -> TypeRef
                      {
                          return Exp->GetType();
                      });
            MgrT::SubstMapT OtherArgSubstMap1;
            for (u32 i = 0; i < NumOtherArgs; ++i) {
                auto BoundVar = Mgr->MakeBoundVar(OtherArgTypes[i], NumOtherArgs - i - 1);
                OtherArgSubstMap1[OtherArgs[i]] = BoundVar;
            }
            auto ArgBoundVar = Mgr->MakeBoundVar(CurrentArg->GetType(), NumOtherArgs);
            auto ArgPrimeBoundVar = Mgr->MakeBoundVar(CurrentArg->GetType(), NumOtherArgs + 1);
            OtherArgSubstMap1[CurrentArg] = ArgBoundVar;
            auto OtherArgSubstMap2 = OtherArgSubstMap1;
            OtherArgSubstMap2[CurrentArg] = ArgPrimeBoundVar;

            QVars.push_back(CurrentArg->GetType());
            QVars.push_back(CurrentArg->GetType());
            QVars.insert(QVars.end(), OtherArgTypes.begin(), OtherArgTypes.end());

            auto SubstExp1 = Mgr->BoundSubstitute(OtherArgSubstMap1, UpdateExp);
            auto SubstExp2 = Mgr->BoundSubstitute(OtherArgSubstMap2, UpdateExp);

            return make_tuple(SubstExp1, SubstExp2, ArgBoundVar, ArgPrimeBoundVar);
        }

        inline ExpT Solver::MakeArgDepConstraints(i64 OpCode, const ExpT& Exp)
        {
            auto Mgr = TheLTS->GetMgr();
            auto const IsGuard = (TheLTS->GuardOpToExp.find(OpCode) !=
                                  TheLTS->GuardOpToExp.end());
            string PerArgCostVarNamePrefix;
            if (IsGuard) {
                PerArgCostVarNamePrefix = (string)"__arg_dep_cost_guard_" + to_string(OpCode);
            } else {
                PerArgCostVarNamePrefix = (string)"__arg_dep_cost_update_" + to_string(OpCode);
            }

            auto ZeroOneType = Mgr->MakeType<RangeType>(0, 1);
            auto TVType = Mgr->MakeType<RangeType>(0, 2);
            auto ZeroExp = Mgr->MakeVal("0", ZeroOneType);
            auto OneExp = Mgr->MakeVal("1", ZeroOneType);
            auto TwoExp = Mgr->MakeVal("2", TVType);

            vector<ExpT> PerArgCosts;

            auto const& OpArgs = GetOpArgs(Exp);
            const u32 NumArgs = OpArgs.size();
            u32 Count = 1;
            for (auto const& Arg : OpArgs) {
                vector<TypeRef> QVars;
                auto Substs = CreateArgDepSubsts(QVars, Exp, Arg);

                auto const& SubstExp1 = get<0>(Substs);
                auto const& SubstExp2 = get<1>(Substs);

                auto QBody = Mgr->MakeExpr(LTSOps::OpEQ, SubstExp1, SubstExp2);
                auto Antecedent = Mgr->MakeForAll(QVars, QBody);
                auto NegAntecedent = Mgr->MakeExpr(LTSOps::OpNOT, Antecedent);

                if (IsGuard || !CheckTypeCompat(Arg->GetType(), Exp->GetType())) {
                    auto CurArgCostVar = Mgr->MakeVar(PerArgCostVarNamePrefix + "_on_arg_" +
                                                      to_string(Count), ZeroOneType);
                    auto CostEQ0 = Mgr->MakeExpr(LTSOps::OpEQ, CurArgCostVar, ZeroExp);
                    auto CostEQ1 = Mgr->MakeExpr(LTSOps::OpEQ, CurArgCostVar, OneExp);

                    auto Constraint0 = Mgr->MakeExpr(LTSOps::OpIFF, Antecedent, CostEQ0);
                    auto Constraint1 = Mgr->MakeExpr(LTSOps::OpIFF, NegAntecedent, CostEQ1);

                    CheckedAssert(Constraint0);
                    CheckedAssert(Constraint1);

                    PerArgCosts.push_back(CurArgCostVar);
                } else {
                    auto CurArgCostVar = Mgr->MakeVar(PerArgCostVarNamePrefix + "_on_arg_" +
                                                      to_string(Count), TVType);
                    auto CostEQ0 = Mgr->MakeExpr(LTSOps::OpEQ, CurArgCostVar, ZeroExp);
                    auto CostEQ1 = Mgr->MakeExpr(LTSOps::OpEQ, CurArgCostVar, OneExp);
                    auto CostEQ2 = Mgr->MakeExpr(LTSOps::OpEQ, CurArgCostVar, TwoExp);

                    auto ExpEQArg = Mgr->MakeExpr(LTSOps::OpEQ, Exp, Arg);

                    MgrT::SubstMapT EQSubstMap;
                    auto FunType = Mgr->LookupUninterpretedFunction(OpCode)->As<FuncType>();
                    auto const& DomTypes = FunType->GetArgTypes();
                    for (u32 i = 0; i < NumArgs; ++i) {
                        EQSubstMap[OpArgs[i]] = Mgr->MakeBoundVar(DomTypes[i], NumArgs - i - 1);
                    }

                    auto ExpEQArgBody = Mgr->BoundSubstitute(EQSubstMap, ExpEQArg);
                    auto ForAllArgExpEQArg = Mgr->MakeForAll(DomTypes, ExpEQArgBody);
                    auto NegForAllArgExpEQArg = Mgr->MakeExpr(LTSOps::OpNOT,
                                                              ForAllArgExpEQArg);

                    auto Constraint0 = Mgr->MakeExpr(LTSOps::OpIFF, Antecedent, CostEQ0);
                    auto Constraint2 = Mgr->MakeExpr(LTSOps::OpIFF,
                                                     Mgr->MakeExpr(LTSOps::OpAND,
                                                                   NegAntecedent,
                                                                   NegForAllArgExpEQArg),
                                                     CostEQ2);
                    auto Constraint1 = Mgr->MakeExpr(LTSOps::OpIFF,
                                                     Mgr->MakeExpr(LTSOps::OpAND,
                                                                   NegAntecedent,
                                                                   ForAllArgExpEQArg),
                                                     CostEQ1);

                    CheckedAssert(Constraint0);
                    CheckedAssert(Constraint1);
                    CheckedAssert(Constraint2);
                    PerArgCosts.push_back(CurArgCostVar);
                }
                ++Count;
            }

            auto TotalArgDepCostType = Mgr->MakeType<RangeType>(0, 2 * PerArgCosts.size());
            auto TotalArgDepCostVar = Mgr->MakeVar(PerArgCostVarNamePrefix, TotalArgDepCostType);
            auto SumExp = MakeSum(PerArgCosts, Mgr, TotalArgDepCostType);
            auto Constraint = Mgr->MakeExpr(LTSOps::OpEQ, TotalArgDepCostVar, SumExp);
            CheckedAssert(Constraint);
            return TotalArgDepCostVar;
        }

        inline void Solver::MakeArgDepCCForGuard(i64 Op, const ExpT& Exp)
        {
            string CostVarName("__guard_func_cost_");
            CostVarName += to_string(Op);
            auto const& OpArgs = GetOpArgs(Exp);
            const u32 NumArgs = OpArgs.size();
            auto Mgr = TheLTS->GetMgr();

            auto CostVarType = Mgr->MakeType<RangeType>(0, 2 * NumArgs + 1);
            auto CostVar = Mgr->MakeVar(CostVarName, CostVarType);
            auto AFATPair = MakeAllFalseConstraint(Op, Exp, false);
            auto const& AFPred = AFATPair.first;
            auto NegAFPred = Mgr->MakeExpr(LTSOps::OpNOT, AFPred);
            auto CostEQ0 = Mgr->MakeExpr(LTSOps::OpEQ, CostVar,
                                         Mgr->MakeVal("0", CostVarType));
            auto OneExp = Mgr->MakeVal("1", CostVarType);
            auto Constraint0 = Mgr->MakeExpr(LTSOps::OpIFF, AFPred, CostEQ0);
            auto ArgDepCost = MakeArgDepConstraints(Op, Exp);
            auto OnePlusADCost = Mgr->MakeExpr(LTSOps::OpADD, OneExp, ArgDepCost);
            auto CostEQArgDep = Mgr->MakeExpr(LTSOps::OpEQ, CostVar, OnePlusADCost);
            auto ConstraintGen = Mgr->MakeExpr(LTSOps::OpIFF, NegAFPred, CostEQArgDep);
            CheckedAssert(Constraint0);
            CheckedAssert(ConstraintGen);
            GuardFuncCosts.push_back(CostVar);
            AllFalsePreds[Op] = AFPred;
            return;
        }

        inline void Solver::MakeNonFalseCCForGuard(i64 Op, const ExpT& Exp)
        {
            string CostVarName("__guard_func_cost_");
            CostVarName += to_string(Op);
            auto Mgr = TheLTS->GetMgr();

            auto CostVarType = Mgr->MakeType<RangeType>(0, 2);
            auto CostVar = Mgr->MakeVar(CostVarName, CostVarType);
            auto AFATPair = MakeAllFalseConstraint(Op, Exp, true);
            auto const& AFPred = AFATPair.first;
            auto const& ATPred = AFATPair.second;
            auto CostEQ0 = Mgr->MakeExpr(LTSOps::OpEQ, CostVar,
                                         Mgr->MakeVal("0", CostVarType));
            auto CostEQ1 = Mgr->MakeExpr(LTSOps::OpEQ, CostVar,
                                         Mgr->MakeVal("1", CostVarType));
            auto CostEQ2 = Mgr->MakeExpr(LTSOps::OpEQ, CostVar,
                                         Mgr->MakeVal("2", CostVarType));

            auto Constraint0 = Mgr->MakeExpr(LTSOps::OpIFF, AFPred, CostEQ0);
            auto Constraint1 = Mgr->MakeExpr(LTSOps::OpIFF, ATPred, CostEQ1);
            auto NotAFNotAT = Mgr->MakeExpr(LTSOps::OpAND,
                                            Mgr->MakeExpr(LTSOps::OpNOT, AFPred),
                                            Mgr->MakeExpr(LTSOps::OpNOT, ATPred));
            auto Constraint2 = Mgr->MakeExpr(LTSOps::OpIFF, NotAFNotAT, CostEQ2);
            CheckedAssert(Constraint0);
            CheckedAssert(Constraint1);
            CheckedAssert(Constraint2);
            GuardFuncCosts.push_back(CostVar);
            AllFalsePreds[Op] = AFPred;
            return;
        }

        inline void Solver::MakePointCCForGuard(i64 Op, const ExpT& Exp)
        {
            string CostVarName("__guard_func_cost_");
            CostVarName += to_string(Op);
            auto Mgr = TheLTS->GetMgr();

            auto&& PointApps = MakePointApplications(Op);
            const u32 NumPoints = PointApps.size();
            auto CostVarType = Mgr->MakeType<RangeType>(0, NumPoints + 1);
            auto CostVar = Mgr->MakeVar(CostVarName, CostVarType);
            auto AFATPair = MakeAllFalseConstraint(Op, Exp, true);
            auto const& AFPred = AFATPair.first;
            auto const& ATPred = AFATPair.second;
            auto NegAFPred = Mgr->MakeExpr(LTSOps::OpNOT, AFPred);
            auto NegATPred = Mgr->MakeExpr(LTSOps::OpNOT, ATPred);

            auto CostEQ0 = Mgr->MakeExpr(LTSOps::OpEQ, CostVar,
                                         Mgr->MakeVal("0", CostVarType));
            auto CostEQ1 = Mgr->MakeExpr(LTSOps::OpEQ, CostVar,
                                         Mgr->MakeVal("1", CostVarType));
            string PointCostVarNamePrefix("__point_cost_guard_");
            PointCostVarNamePrefix += to_string(Op);
            auto PerPointCostVarType = Mgr->MakeType<RangeType>(0, 1);
            vector<ExpT> PerPointCostVars(NumPoints);

            for (u32 i = 0; i < NumPoints; ++i) {
                auto const& CurApp = PointApps[i];
                auto NegCurApp = Mgr->MakeExpr(LTSOps::OpNOT, CurApp);
                auto CurPointCostVar = Mgr->MakeVar(PointCostVarNamePrefix +
                                                    "_" + to_string(i),
                                                    PerPointCostVarType);
                auto CurPointCostEQ0 = Mgr->MakeExpr(LTSOps::OpEQ,
                                                     CurPointCostVar,
                                                     Mgr->MakeVal("0", PerPointCostVarType));
                auto CurPointCostEQ1 = Mgr->MakeExpr(LTSOps::OpEQ,
                                                     CurPointCostVar,
                                                     Mgr->MakeVal("1", PerPointCostVarType));
                auto Constraint1 = Mgr->MakeExpr(LTSOps::OpIFF, CurApp, CurPointCostEQ1);
                auto Constraint0 = Mgr->MakeExpr(LTSOps::OpIFF, NegCurApp, CurPointCostEQ0);
                CheckedAssert(Constraint0);
                CheckedAssert(Constraint1);
                PerPointCostVars[i] = CurPointCostVar;
            }

            auto Constraint0 = Mgr->MakeExpr(LTSOps::OpIFF, AFPred, CostEQ0);
            auto Constraint1 = Mgr->MakeExpr(LTSOps::OpIFF, ATPred, CostEQ1);
            auto NotAFNotAT = Mgr->MakeExpr(LTSOps::OpAND, NegAFPred, NegATPred);
            auto PointSum = MakeSum(PerPointCostVars, Mgr, CostVarType);
            auto OnePlusPointSum = Mgr->MakeExpr(LTSOps::OpADD, PointSum,
                                                 Mgr->MakeVal("1", CostVarType));
            auto ConstraintN = Mgr->MakeExpr(LTSOps::OpIFF, NotAFNotAT,
                                             Mgr->MakeExpr(LTSOps::OpEQ, CostVar,
                                                           OnePlusPointSum));

            CheckedAssert(Constraint0);
            CheckedAssert(Constraint1);
            CheckedAssert(ConstraintN);
            GuardFuncCosts.push_back(CostVar);
            AllFalsePreds[Op] = AFPred;
            return;
        }

        inline void Solver::MakeCostConstraintsForGuard(i64 Op, const ExpT& Exp)
        {
            if (Options.GBoundMethod == GuardBoundingMethodT::NoBounding) {
                auto AFATPair = MakeAllFalseConstraint(Op, Exp, false);
                auto AFPred = AFATPair.first;
                AllFalsePreds[Op] = AFPred;
                return;
            }
            if (Options.GBoundMethod == GuardBoundingMethodT::NonFalseBound) {
                MakeNonFalseCCForGuard(Op, Exp);
                return;
            }
            if (Options.GBoundMethod == GuardBoundingMethodT::VarDepBound) {
                MakeArgDepCCForGuard(Op, Exp);
                return;
            }
            if (Options.GBoundMethod == GuardBoundingMethodT::PointBound) {
                MakePointCCForGuard(Op, Exp);
                return;
            }
        }

        inline ExpT
        Solver::MakeIdentityConstraint(i64 Op, const ExpT& UpdateExp, u32 LValueIndex)
        {
            auto Mgr = TheLTS->GetMgr();
            auto FunType = Mgr->LookupUninterpretedFunction(Op)->As<FuncType>();
            auto const& DomTypes = FunType->GetArgTypes();
            const u32 NumArgs = DomTypes.size();
            auto const& OpArgs = GetOpArgs(UpdateExp);
            auto const& LValueExp = OpArgs[LValueIndex];
            auto AppEQLValue = Mgr->MakeExpr(LTSOps::OpEQ, UpdateExp, LValueExp);

            MgrT::SubstMapT SubstMap;
            for (u32 i = 0; i < NumArgs; ++i) {
                SubstMap[OpArgs[i]] = Mgr->MakeBoundVar(DomTypes[i], NumArgs - i - 1);
            }
            auto Body = Mgr->BoundSubstitute(SubstMap, AppEQLValue);
            auto ForallExp = Mgr->MakeForAll(DomTypes, Body);
            auto IdentityPred = Mgr->MakeVar("__identity_" + to_string(Op),
                                             Mgr->MakeType<BooleanType>());
            auto Constraint = Mgr->MakeExpr(LTSOps::OpIFF, ForallExp, IdentityPred);
            CheckedAssert(Constraint);
            return IdentityPred;
        }

        inline ExpT
        Solver::MakeConstantConstraint(i64 Op, const ExpT& Exp)
        {
            auto Mgr = TheLTS->GetMgr();
            auto&& PointApps = MakePointApplications(Op);
            const u32 NumPoints = PointApps.size();

            if (NumPoints == 1) {
                return Mgr->MakeTrue();
            } else {
                vector<ExpT> Conjuncts(NumPoints - 1);
                auto Reference = PointApps[0];
                for (u32 i = 1; i < NumPoints; ++i) {
                    Conjuncts[i - 1] = Mgr->MakeExpr(LTSOps::OpEQ, Reference, PointApps[i]);
                }
                auto AllEQExp = MakeConjunction(Conjuncts, Mgr);
                auto ConstantPred = Mgr->MakeVar("__constant_update_" + to_string(Op),
                                                 Mgr->MakeType<BooleanType>());
                auto IffExp = Mgr->MakeExpr(LTSOps::OpIFF, AllEQExp, ConstantPred);
                CheckedAssert(IffExp);
                return ConstantPred;
            }
        }

        inline void
        Solver::MakeArgDepCCForLValUpdate(i64 Op, const ExpT& Exp,
                                          const ExpT& LValueExp)
        {
            auto Mgr = TheLTS->GetMgr();
            auto const& OpArgs = GetOpArgs(Exp);
            const u32 NumArgs = OpArgs.size();
            i32 LValueIndex = -1;
            for (u32 i = 0; i < NumArgs; ++i) {
                if (OpArgs[i] == LValueExp) {
                    LValueIndex = i;
                    break;
                }
            }

            if (LValueIndex < 0) {
                throw InternalError((string)"Could not find index of lvalue in update exp:\n" +
                                    Exp->ToString() + "\nAt: " + __FILE__ + ":" +
                                    to_string(__LINE__));
            }

            string CostVarName("__update_func_cost_");
            CostVarName += to_string(Op);

            auto CostVarType = Mgr->MakeType<RangeType>(0, 2 * NumArgs + 1);
            auto ZeroExp = Mgr->MakeVal("0", CostVarType);
            auto OneExp = Mgr->MakeVal("1", CostVarType);
            auto CostVar = Mgr->MakeVar(CostVarName, CostVarType);

            auto CostEQ0 = Mgr->MakeExpr(LTSOps::OpEQ, CostVar, ZeroExp);
            auto CostEQ1 = Mgr->MakeExpr(LTSOps::OpEQ, CostVar, OneExp);
            auto ArgDepCost = MakeArgDepConstraints(Op, Exp);
            auto OnePlusArgDepCost = Mgr->MakeExpr(LTSOps::OpADD, ArgDepCost, OneExp);
            auto CostEQN = Mgr->MakeExpr(LTSOps::OpEQ, CostVar, OnePlusArgDepCost);

            auto IdentityPred = MakeIdentityConstraint(Op, Exp, LValueIndex);
            auto ConstantPred = MakeConstantConstraint(Op, Exp);
            auto NegIdentityPred = Mgr->MakeExpr(LTSOps::OpNOT, IdentityPred);
            auto NegConstantPred = Mgr->MakeExpr(LTSOps::OpNOT, ConstantPred);
            auto NonIdNonConst = Mgr->MakeExpr(LTSOps::OpAND, NegIdentityPred,
                                               NegConstantPred);

            auto Constraint0 = Mgr->MakeExpr(LTSOps::OpIFF, IdentityPred, CostEQ0);
            auto Constraint1 = Mgr->MakeExpr(LTSOps::OpIFF, ConstantPred, CostEQ1);
            auto ConstraintN = Mgr->MakeExpr(LTSOps::OpIFF, NonIdNonConst, CostEQN);

            CheckedAssert(Constraint0);
            CheckedAssert(Constraint1);
            CheckedAssert(ConstraintN);
            UpdateFuncCosts.push_back(CostVar);
            return;
        }

        inline void
        Solver::MakeIdentityCCForLValUpdate(i64 Op, const ExpT& Exp,
                                            const ExpT& LValueExp)
        {
            auto Mgr = TheLTS->GetMgr();
            auto const& OpArgs = GetOpArgs(Exp);
            const u32 NumArgs = OpArgs.size();
            i32 LValueIndex = -1;
            for (u32 i = 0; i < NumArgs; ++i) {
                if (OpArgs[i] == LValueExp) {
                    LValueIndex = i;
                    break;
                }
            }

            if (LValueIndex < 0) {
                throw InternalError((string)"Could not find index of lvalue in update exp:\n" +
                                    Exp->ToString() + "\nAt: " + __FILE__ + ":" +
                                    to_string(__LINE__));
            }

            string CostVarName("__update_func_cost_");
            CostVarName += to_string(Op);

            auto CostVarType = Mgr->MakeType<RangeType>(0, 2);
            auto CostVar = Mgr->MakeVar(CostVarName, CostVarType);
            auto ZeroExp = Mgr->MakeVal("0", CostVarType);
            auto OneExp = Mgr->MakeVal("1", CostVarType);
            auto TwoExp = Mgr->MakeVal("2", CostVarType);
            auto CostEQ0 = Mgr->MakeExpr(LTSOps::OpEQ, CostVar, ZeroExp);
            auto CostEQ1 = Mgr->MakeExpr(LTSOps::OpEQ, CostVar, OneExp);
            auto CostEQ2 = Mgr->MakeExpr(LTSOps::OpEQ, CostVar, TwoExp);
            auto IdentityPred = MakeIdentityConstraint(Op, Exp, LValueIndex);
            auto ConstantPred = MakeConstantConstraint(Op, Exp);
            auto NegIdentityPred = Mgr->MakeExpr(LTSOps::OpNOT, IdentityPred);
            auto NegConstantPred = Mgr->MakeExpr(LTSOps::OpNOT, ConstantPred);
            auto NonIdNonConst = Mgr->MakeExpr(LTSOps::OpAND, NegIdentityPred,
                                               NegConstantPred);

            auto Constraint0 = Mgr->MakeExpr(LTSOps::OpIFF, IdentityPred, CostEQ0);
            auto Constraint1 = Mgr->MakeExpr(LTSOps::OpIFF, ConstantPred, CostEQ1);
            auto Constraint2 = Mgr->MakeExpr(LTSOps::OpIFF, NonIdNonConst, CostEQ2);

            CheckedAssert(Constraint0);
            CheckedAssert(Constraint1);
            CheckedAssert(Constraint2);
            UpdateFuncCosts.push_back(CostVar);
            return;
        }

        inline void
        Solver::MakePointCCForLValUpdate(i64 Op, const ExpT& Exp,
                                         const ExpT& LValueExp)
        {
            auto Mgr = TheLTS->GetMgr();
            auto const& OpArgs = GetOpArgs(Exp);
            const u32 NumArgs = OpArgs.size();
            i32 LValueIndex = -1;
            for (u32 i = 0; i < NumArgs; ++i) {
                if (OpArgs[i] == LValueExp) {
                    LValueIndex = i;
                    break;
                }
            }

            if (LValueIndex < 0) {
                throw InternalError((string)"Could not find index of lvalue in update exp:\n" +
                                    Exp->ToString() + "\nAt: " + __FILE__ + ":" +
                                    to_string(__LINE__));
            }

            string CostVarName("__update_func_cost_");
            CostVarName += to_string(Op);

            auto&& PointApps = MakePointApplications(Op);
            const u32 NumPoints = PointApps.size();
            auto CostVarType = Mgr->MakeType<RangeType>(0, NumPoints + 1);
            auto CostVar = Mgr->MakeVar(CostVarName, CostVarType);

            auto IdentityPred = MakeIdentityConstraint(Op, Exp, LValueIndex);
            auto ConstantPred = MakeConstantConstraint(Op, Exp);
            auto NegIdentityPred = Mgr->MakeExpr(LTSOps::OpNOT, IdentityPred);
            auto NegConstantPred = Mgr->MakeExpr(LTSOps::OpNOT, ConstantPred);
            auto NonIdNonConst = Mgr->MakeExpr(LTSOps::OpAND, NegIdentityPred,
                                               NegConstantPred);
            auto CostEQ0 = Mgr->MakeExpr(LTSOps::OpEQ, CostVar,
                                         Mgr->MakeVal("0", CostVarType));
            auto CostEQ1 = Mgr->MakeExpr(LTSOps::OpEQ, CostVar,
                                         Mgr->MakeVal("1", CostVarType));
            string PointCostVarNamePrefix("__point_cost_update_");
            PointCostVarNamePrefix += to_string(Op);
            auto PerPointCostVarType = Mgr->MakeType<RangeType>(0, 1);
            vector<ExpT> PerPointCostVars(NumPoints);

            for (u32 i = 0; i < NumPoints; ++i) {
                auto const& CurApp = PointApps[i];
                auto const& CurAppArgs = GetOpArgs(CurApp);
                auto AppEQLVal = Mgr->MakeExpr(LTSOps::OpEQ, CurApp,
                                               CurAppArgs[LValueIndex]);
                auto NegAppEQLVal = Mgr->MakeExpr(LTSOps::OpNOT, AppEQLVal);
                auto CurPointCostVar = Mgr->MakeVar(PointCostVarNamePrefix +
                                                    "_" + to_string(i),
                                                    PerPointCostVarType);
                auto CurPointCostEQ0 = Mgr->MakeExpr(LTSOps::OpEQ, CurPointCostVar,
                                                     Mgr->MakeVal("0", PerPointCostVarType));
                auto CurPointCostEQ1 = Mgr->MakeExpr(LTSOps::OpEQ, CurPointCostVar,
                                                     Mgr->MakeVal("1", PerPointCostVarType));
                auto Constraint0 = Mgr->MakeExpr(LTSOps::OpIFF, AppEQLVal, CurPointCostEQ0);
                auto Constraint1 = Mgr->MakeExpr(LTSOps::OpIFF, NegAppEQLVal, CurPointCostEQ1);
                CheckedAssert(Constraint0);
                CheckedAssert(Constraint1);
                PerPointCostVars[i] = CurPointCostVar;
            }

            auto Constraint0 = Mgr->MakeExpr(LTSOps::OpIFF, IdentityPred, CostEQ0);
            auto Constraint1 = Mgr->MakeExpr(LTSOps::OpIFF, ConstantPred, CostEQ1);

            auto PointSum = MakeSum(PerPointCostVars, Mgr, CostVarType);
            auto OnePlusPointSum = Mgr->MakeExpr(LTSOps::OpADD, PointSum,
                                                 Mgr->MakeVal("1", CostVarType));
            auto ConstraintN = Mgr->MakeExpr(LTSOps::OpIFF, NonIdNonConst,
                                             Mgr->MakeExpr(LTSOps::OpEQ, CostVar,
                                                           OnePlusPointSum));
            CheckedAssert(Constraint0);
            CheckedAssert(Constraint1);
            CheckedAssert(ConstraintN);
            UpdateFuncCosts.push_back(CostVar);
            return;
        }

        inline void
        Solver::MakeCostConstraintsForLValueUpdate(i64 Op, const ExpT& UpdateExp,
                                                   const ExpT& LValueExp)
        {
            if (Options.UBoundMethod == UpdateBoundingMethodT::NoBounding) {
                return;
            }
            if (Options.UBoundMethod == UpdateBoundingMethodT::NonIdentityBound) {
                MakeIdentityCCForLValUpdate(Op, UpdateExp, LValueExp);
                return;
            }
            if (Options.UBoundMethod == UpdateBoundingMethodT::VarDepBound) {
                MakeArgDepCCForLValUpdate(Op, UpdateExp, LValueExp);
                return;
            }
            if (Options.UBoundMethod == UpdateBoundingMethodT::PointBound) {
                MakePointCCForLValUpdate(Op, UpdateExp, LValueExp);
                return;
            }
        }

        inline void
        Solver::MakeCostConstraintsForStateUpdate(i64 Op, const ExpT& UpdateExp)
        {
            if (Options.SBoundMethod == StateUpdateBoundingMethodT::NoBounding) {
                return;
            }
            if (Options.SBoundMethod == StateUpdateBoundingMethodT::AllSame) {
                auto ConstantPred = MakeConstantConstraint(Op, UpdateExp);
                CheckedAssert(ConstantPred);
            }
            if (Options.SBoundMethod == StateUpdateBoundingMethodT::VarDepBound) {
                auto ArgDepCost = MakeArgDepConstraints(Op, UpdateExp);
                auto const& AppArgs = GetOpArgs(UpdateExp);
                const u32 NumArgs = AppArgs.size();
                string CostVarName("__state_update_func_cost_");
                CostVarName += to_string(Op);
                auto Mgr = TheLTS->GetMgr();
                auto CostVarType = Mgr->MakeType<RangeType>(0, NumArgs);
                auto CostVar = Mgr->MakeVar(CostVarName, CostVarType);
                auto Constraint = Mgr->MakeExpr(LTSOps::OpEQ, CostVar, ArgDepCost);
                CheckedAssert(Constraint);
                UpdateFuncCosts.push_back(CostVar);
            }
        }

        inline void
        Solver::MakeCostConstraintsForNonLValueUpdate(i64 Op, const ExpT& UpdateExp)
        {
            if (Options.UBoundMethod == UpdateBoundingMethodT::NoBounding) {
                return;
            } else {
                // Always apply vardep, since it's the only thing that makes
                // sense
                auto const& AppArgs = GetOpArgs(UpdateExp);
                const u32 NumArgs = AppArgs.size();
                auto Mgr = TheLTS->GetMgr();
                auto ConstantPred = MakeConstantConstraint(Op, UpdateExp);
                auto NegConstantPred = Mgr->MakeExpr(LTSOps::OpNOT, ConstantPred);
                string CostVarName("__msg_update_func_cost_");
                CostVarName += to_string(Op);
                auto CostVarType = Mgr->MakeType<RangeType>(0, 2 * NumArgs + 1);
                auto CostVar = Mgr->MakeVar(CostVarName, CostVarType);
                auto CostEQ0 = Mgr->MakeExpr(LTSOps::OpEQ, CostVar,
                                             Mgr->MakeVal("0", CostVarType));
                auto ArgDepCost = MakeArgDepConstraints(Op, UpdateExp);
                auto OnePlusADCost = Mgr->MakeExpr(LTSOps::OpADD, ArgDepCost,
                                                   Mgr->MakeVal("1", CostVarType));
                auto CostEQN = Mgr->MakeExpr(LTSOps::OpEQ, CostVar,
                                             OnePlusADCost);
                auto Constraint0 = Mgr->MakeExpr(LTSOps::OpIFF, ConstantPred, CostEQ0);
                auto ConstraintN = Mgr->MakeExpr(LTSOps::OpIFF, NegConstantPred, CostEQN);
                CheckedAssert(Constraint0);
                CheckedAssert(ConstraintN);
                UpdateFuncCosts.push_back(CostVar);
                return;
            }
        }

        inline void
        Solver::MakeCostConstraintsForOp(i64 Op)
        {
            auto const& GuardOpToExp = TheLTS->GuardOpToExp;
            auto it1 = GuardOpToExp.find(Op);
            if (it1 != GuardOpToExp.end()) {
                // This is a guard
                MakeCostConstraintsForGuard(Op, it1->second);
                return;
            }
            // This is an update
            auto const& UpdateOpToUpdateLValue = TheLTS->UpdateOpToUpdateLValue;
            auto it2 = UpdateOpToUpdateLValue.find(Op);
            if (it2 != UpdateOpToUpdateLValue.end()) {
                // This is an update with an lvalue associated with it
                MakeCostConstraintsForLValueUpdate(Op, it2->second.first, it2->second.second);
                return;
            }

            auto const& StateUpdateOpToExp = TheLTS->StateUpdateOpToExp;
            auto it3 = StateUpdateOpToExp.find(Op);
            if (it3 != StateUpdateOpToExp.end()) {
                // This is an update on the state (location) variable
                MakeCostConstraintsForStateUpdate(Op, it3->second);
                return;
            }

            // This is for sure an update on a message field
            auto const& AllOpToExp = TheLTS->AllOpToExp;
            auto it4 = AllOpToExp.find(Op);
            if (it4 == AllOpToExp.end()) {
                throw InternalError((string)"Could not find expression associated with " +
                                    "op: " + to_string(Op) + "\nAt: " + __FILE__ + ":" +
                                    to_string(__LINE__));
            }
            MakeCostConstraintsForNonLValueUpdate(Op, it4->second);
        }

        void Solver::UnveilGuardOp(i64 Op)
        {
            auto&& UpdateOps = AssertConstraintsForNewGuard(Op);

            MakeCostConstraintsForOp(Op);

            // Create indicator variables for each newly unveiled update as well
            for (auto const& NewUpdateOp : UpdateOps) {
                MakeCostConstraintsForOp(NewUpdateOp);
                MakeRangeConstraintsForOp(NewUpdateOp);
            }
            UpdateCommands();
        }

        void Solver::UnveilNonCompletionGuardOp(i64 Op)
        {
            UnveiledGuardOps.insert(Op);
            InterpretedOps.insert(Op);
            // CreateGuardIndicator(Op);
            MakeCostConstraintsForOp(Op);
            UpdateCommands();
        }

        void Solver::UnveilNonCompletionOp(i64 Op)
        {
            // TODO putting it in GuardOps,
            // perhaps a common unveiled ops set should be added.
            UnveiledGuardOps.insert(Op);
            InterpretedOps.insert(Op);
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
                // Unveil if this is indeed a guard op
                if (TheLTS->GuardOpToExp.find(Op) != TheLTS->GuardOpToExp.end()) {
                    UnveilGuardOp(Op);
                }
            }
        }

        inline void Solver::HandleOneSafetyViolation(const StateVec* ErrorState,
                                                     const ExpT& BlownInvariant)
        {
            auto AQS = Checker->AQS;

            auto PPath = AQS->FindShortestPath(ErrorState, CostFunction);
            SafetyViolation* Trace;
            if (BlownInvariant == Checker->LoweredInvariant) {
                Trace = TraceBase::MakeSafetyViolation(PPath, Checker, BlownInvariant);
            } else {
                Trace = TraceBase::MakeBoundsViolation(PPath, Checker);
            }

            auto const& TraceBlownInvar = Trace->GetInvariantBlown();

            ESMC_LOG_FULL(
                          "Solver.Traces",
                          Out_ << "Safety violation trace:" << endl;
                          Out_ << Trace->ToString() << endl;
                          );

            ESMC_LOG_FULL(
                          "Solver.CEXAssertions",
                          Out_ << "Computing weakest pre (safety) of: " << endl
                               << TraceBlownInvar << endl;
                          );

            auto&& WPConditions =
                TraceAnalyses::WeakestPrecondition(this, Trace, TraceBlownInvar);
            for (auto const& Pred : WPConditions) {

                ESMC_LOG_SHORT(
                               "Solver.CEXAssertions",
                               Out_ << "Obtained Safety Pre:" << endl
                                    << Pred << endl;
                               );

                MakeAssertion(Pred);
            }

            delete Trace;
        }

        inline void Solver::HandleOneDeadlockViolation(const StateVec* ErrorState)
        {
            auto Mgr = TheLTS->GetMgr();
            auto AQS = Checker->AQS;

            auto PPath = AQS->FindShortestPath(ErrorState, CostFunction);

            auto Trace = TraceBase::MakeDeadlockViolation(PPath, Checker);

            ESMC_LOG_FULL(
                          "Solver.Traces",
                          Out_ << "Deadlock Trace:" << endl;
                          Out_ << Trace->ToString() << endl;
                          );

            ExpT GoodExp = ExpT::NullPtr;

            if (!Options.GeneralFixForDL) {
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
                    Disjuncts.push_back(Cmd->GetLoweredGuard());
                }

                auto UnreachableExp = TraceAnalyses::AutomataStatesCondition(TheLTS, LastState);
                UnreachableExp = Mgr->MakeExpr(LTSOps::OpNOT, UnreachableExp);
                UnreachableExp = Mgr->ApplyTransform<ArrayRValueTransformer>(UnreachableExp);

                Disjuncts.push_back(UnreachableExp);
                GoodExp = MakeDisjunction(Disjuncts, Mgr);
                GoodExp = Mgr->SimplifyFP(GoodExp);
            } else {
                GoodExp = Checker->LoweredDLFInvariant;
            }

            ESMC_LOG_FULL(
                          "Solver.CEXAssertions",
                          Out_ << "Computing Weakest Pre of: " << endl << GoodExp << endl;
                          );

            auto&& WPConditions =
                TraceAnalyses::WeakestPrecondition(this, Trace->As<SafetyViolation>(), GoodExp);

            for (auto const& Pred : WPConditions) {
                ESMC_LOG_FULL(
                              "Solver.CEXAssertions",
                              Out_ << "Obtained Deadlock Pre:" << endl << Pred << endl;
                              );

                MakeAssertion(Pred);
            }

            delete Trace;
        }

        inline void Solver::HandleSafetyViolations()
        {
            auto const& ErrorStates = Checker->GetAllErrorStates();

            ESMC_LOG_MIN_SHORT(
                               Out_ << "Building Constraints for "
                                    << ErrorStates.size() << " error(s)...";
                               );

            for (auto const& ErrorState : ErrorStates) {
                auto SVPtr = ErrorState.first;
                auto const& BlownInvariant = ErrorState.second;
                if (BlownInvariant == Checker->LoweredDLFInvariant) {
                    HandleOneDeadlockViolation(SVPtr);
                } else {
                    HandleOneSafetyViolation(SVPtr, BlownInvariant);
                }
            }

            ESMC_LOG_MIN_SHORT(
                               Out_ << " Done!" << endl;
                               );
        }

        inline void Solver::HandleLivenessViolation(const LivenessViolation* Trace,
                                                    StateBuchiAutomaton* Monitor)
        {
            auto&& WPConditions =
                TraceAnalyses::WeakestPreconditionForLiveness(this, Monitor, Trace);
            cout << "computed weakest precondition" << endl;
            for (auto const& Pred : WPConditions) {

                ESMC_LOG_SHORT(
                               "Solver.CEXAssertions",
                               Out_ << "Obtained Liveness Pre:" << endl
                                    << Pred << endl;
                               );

                MakeAssertion(Pred);
            }
            delete Trace;
        }

        inline void Solver::HandleTPReset()
        {
            // The theorem prover object has been reset,
            // and has a new context, we need to
            // translate the assumptions into the new context
            auto OldCtx = Ctx;
            auto NewCtx = TP->GetCtx();
            deque<Z3Expr> NewAssumptions;
            for (auto const& Assumption : CurrentAssumptions) {
                Z3Expr NewAssumption(NewCtx, Z3_translate(*OldCtx, Assumption, *NewCtx));
                NewAssumptions.push_back(NewAssumption);
            }
            Ctx = NewCtx;
            CurrentAssumptions = NewAssumptions;
        }

        inline void Solver::AssertBoundsConstraint(u32 CurrentBound)
        {
            if (!UnveiledNewOps) {
                return;
            }

            UnveiledNewOps = false;

            auto Mgr = TheLTS->GetMgr();
            vector<ExpT> Summands;
            ExpT SumExp = nullptr;

            // Sum over ALL the indicators
            for (auto const& FuncCost : GuardFuncCosts) {
                Summands.push_back(FuncCost);
            }
            for (auto const& FuncCost : UpdateFuncCosts) {
                Summands.push_back(FuncCost);
            }

            auto OldCtx = Ctx;

            if (Summands.size() == 0) {
                // No bounding, nothing changes, leave everything
                // alone and return.
                return;
            } else {
                if (Summands.size() == 1) {
                    SumExp = Summands[0];
                } else {
                    SumExp = Mgr->MakeExpr(LTSOps::OpADD, Summands);
                }
            }

            auto EQExp = Mgr->MakeExpr(LTSOps::OpEQ, SumExp, BoundsVariable);

            TP->ResetToImmutable();
            TP->Assert(EQExp, false, Options.UnrollQuantifiers);

            HandleTPReset();

            return;
        }

        inline void Solver::ResetStats()
        {
            Stats.SolveStartTime = TimeValue();
            Stats.SolveEndTime = TimeValue();
            Stats.MinSMTTime = UINT64_MAX;
            Stats.MaxSMTTime = 0;
            Stats.NumIterations = 0;
            Stats.TotalSMTTime = 0;
            Stats.InitialNumAssertions = 0;
            Stats.FinalNumAssertions = 0;
        }

        inline void Solver::PrintStats()
        {
            string Z3Stats;
            auto StatsObj = Z3_solver_get_statistics(*Ctx, TP->GetSolver());
            Z3_stats_inc_ref(*Ctx, StatsObj);
            Z3Stats = Z3_stats_to_string(*Ctx, StatsObj);
            Z3_stats_dec_ref(*Ctx, StatsObj);

            ESMC_LOG_MIN_SHORT(
                               auto SolveTimeInMicroSecs = (Stats.SolveEndTime -
                                                            Stats.SolveStartTime).InMicroSeconds();

                               Out_ << "Solver Stats:" << endl << endl;
                               Out_ << "----------------------------------------------------------------" << endl;
                               Out_ << "Solve Time: " << ((float)SolveTimeInMicroSecs / 1000000.0)
                               << " (s)" << endl;
                               Out_ << "Initial Asserts: " << Stats.InitialNumAssertions << endl;
                               Out_ << "Final Asserts: " << Stats.FinalNumAssertions << endl;
                               Out_ << "Num Iterations: " << Stats.NumIterations << endl;
                               Out_ << "Total SMT Time: " << Stats.TotalSMTTime << " (uS)" << endl;
                               Out_ << "Min SMT Time: " << Stats.MinSMTTime << " (uS)" << endl;
                               Out_ << "Max SMT Time: " << Stats.MaxSMTTime << " (uS)" << endl;
                               Out_ << "Avg SMT Time: "
                               << ((float)Stats.TotalSMTTime / Stats.NumIterations)
                               << " (uS)" << endl;
                               Out_ << "Final Bound: " << Bound << endl;
                               Out_ << "----------------------------------------------------------------" << endl;
                               Out_ << "Z3 Solver Stats:" << endl << endl;
                               Out_ << "----------------------------------------------------------------" << endl;
                               Out_ << Z3Stats << endl;
                               Out_ << "----------------------------------------------------------------" << endl;

                               );
        }

        inline void Solver::HandleResourceLimit()
        {
            Stats.SolveEndTime = TimeValue::GetTimeValue();
            if (ResourceLimitManager::CheckMemOut()) {
                ESMC_LOG_MIN_FULL(
                                  Out_ << "Memory limit reached. Aborting with Memout!" << endl;
                                  );
            } else {
                ESMC_LOG_MIN_FULL(
                                  Out_ << "CPU Time limit reached. Aborting with Timeout" << endl;
                                  );
            }
            PrintStats();
            ResourceLimitManager::QueryEnd();
            exit(1);
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
            ResetStats();
            Stats.SolveStartTime = TimeValue::GetTimeValue();
            ResourceLimitManager::SetCPULimit(Options.CPULimitInSeconds);
            ResourceLimitManager::SetMemLimit((Options.MemLimitInMB == UINT64_MAX ?
                                               Options.MemLimitInMB : Options.MemLimitInMB << 20));
            ResourceLimitManager::QueryStart();
            ResourceLimitManager::AddOnLimitHandler([this](bool TimeOut) -> void
                                                    {
                                                        TP->Interrupt();
                                                    });

            bool FirstIteration = true;
            bool InitialConstraintsCounted = false;
            bool DoneOneMCIteration = false;
            while (Bound <= Options.BoundLimit) {

                if (ResourceLimitManager::CheckMemOut() ||
                    ResourceLimitManager::CheckTimeOut()) {
                    HandleResourceLimit();
                }

                if (!FirstIteration && !InitialConstraintsCounted) {
                    InitialConstraintsCounted = true;
                    Stats.InitialNumAssertions = TP->GetNumAssertions();
                }

                Stats.FinalNumAssertions = TP->GetNumAssertions();

                AssertBoundsConstraint(Bound);

                auto SMTStartTime = TimeValue::GetTimeValue();
                auto TPRes = TP->CheckSatWithAssumptions(CurrentAssumptions);
                ++Stats.NumIterations;
                auto SMTEndTime = TimeValue::GetTimeValue();
                auto CurQueryTime = SMTEndTime - SMTStartTime;
                auto CurSMTTime = CurQueryTime.InMicroSeconds();
                Stats.TotalSMTTime += CurSMTTime;

                if (CurSMTTime < Stats.MinSMTTime) {
                    Stats.MinSMTTime = CurSMTTime;
                }
                if (CurSMTTime > Stats.MaxSMTTime) {
                    Stats.MaxSMTTime = CurSMTTime;
                }

                if (TPRes == TPResult::UNSATISFIABLE) {
                    ++Bound;
                    ESMC_LOG_MIN_SHORT(
                                       Out_ << "UNSAT! Relaxed bound to " << Bound << endl;
                                       );

                    if (CurrentAssumptions.size() > 0) {
                        CurrentAssumptions.pop_front();
                    }

                    // Reset z3 on bounds bump
                    if (DoneOneMCIteration) {
                        TP->Reset();
                        HandleTPReset();
                        DoneOneMCIteration = false;
                    };
                    continue;

                } else if (TPRes == TPResult::UNKNOWN &&
                           !ResourceLimitManager::CheckMemOut() &&
                           !ResourceLimitManager::CheckTimeOut()) {
                    ResourceLimitManager::QueryEnd();
                    throw ESMCError((string)"Could not solve constraints!");
                } else if (TPRes == TPResult::UNKNOWN) {
                    HandleResourceLimit();
                }

                // all good. extract a model
                auto const& Model = TP->GetModel();

                ESMC_LOG_FULL(
                              "Solver.Models",
                              Out_ << "Model:" << endl << PrintModel(Model) << endl;
                              );

                Compiler->UpdateModel(Model, InterpretedOps, AllFalsePreds);

                // Okay, we're good to model check now
                Checker->ClearAQS();
                u32 CExBound = 0;
                if (FirstIteration) {
                    CExBound = UINT32_MAX;
                    FirstIteration = false;
                } else {
                    CExBound = Options.NumCExToProcess;
                }
                auto Safe = Checker->BuildAQS(AQSConstructionMethod::BreadthFirst,
                                              Options.PrioritizeNonTentative, CExBound);
                DoneOneMCIteration = true;

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
                    ResourceLimitManager::QueryEnd();
                    Stats.SolveEndTime = TimeValue::GetTimeValue();

                    ESMC_LOG_MIN_SHORT(
                                       Out_ << "Found Correct Completion!" << endl;
                                       Out_ << "With Bound = " << Bound << ", Model:" << endl;
                                       PrintFinalSolution(Out_);
                                       PrintStats();
                                       );
                    return;
                }
            }

            ESMC_LOG_MIN_SHORT(
                               Out_ << "Exceeded max bound, bailing!" << endl;
                               );
        }


        ExpT Solver::Evaluate(const ExpT& Input)
        {
            return TP->Evaluate(Input);
        }

        void Solver::PrintOneUFFinalSolution(const vector<const UFInterpreter*>& Interps,
                                             ostream& Out)
        {
            auto Mgr = TheLTS->GetMgr();

            auto FunType =
                Mgr->LookupUninterpretedFunction(Interps[0]->GetOpCode())->As<FuncType>();
            auto const& DomTypes = FunType->GetArgTypes();
            const u32 DomSize = DomTypes.size();
            auto const& RangeType = FunType->GetEvalType();
            // auto const& FuncName = FunType->GetName();
            auto const& AllOpToExp = TheLTS->AllOpToExp;
            auto it = AllOpToExp.find(Interps[0]->GetOpCode());
            if (it == AllOpToExp.end()) {
                throw InternalError((string)"Could not resolve Op: " +
                                    to_string(Interps[0]->GetOpCode()) + " to and expression!\n" +
                                    "At: " + __FILE__ + ":" + to_string(__LINE__));
            }
            auto AppExp = it->second;

            UFInterpreter::EvalMapT CombinedEvalMap;
            for (auto const* Interp : Interps) {
                auto const& EvalMap = Interp->GetEvalMap();
                if (!Interp->IsEnabled() || EvalMap.size() == 0) {
                    continue;
                }
                CombinedEvalMap.insert(EvalMap.begin(), EvalMap.end());
            }

            if (CombinedEvalMap.size() == 0) {
                return;
            }

            Out << "Model for uninterpreted function:" << endl
                << AppExp->ToString() << " -> {" << endl;
            string IndentString = "    ";

            for (auto const& EvalPoint : CombinedEvalMap) {
                Out << IndentString;
                auto const& Point = EvalPoint.first;
                auto const& Value = EvalPoint.second;

                for (u32 i = 0; i < DomSize; ++i) {
                    auto const& Val = DomTypes[i]->SAs<ScalarType>()->ValToConst(Point[i]);
                    Out << Val << " ";
                }

                auto const& Val = RangeType->SAs<ScalarType>()->ValToConst(Value);
                Out << "-> " << Val << endl;
            }

            Out << "}" << endl << endl;
        }

        void Solver::PrintFinalSolution(ostream& Out)
        {
            auto const& AllOpToInterp = Checker->Compiler->GetUFInterpreters();
            for (auto Op : InterpretedOps) {
                auto it = AllOpToInterp.find(Op);
                if (it != AllOpToInterp.end()) {
                    PrintOneUFFinalSolution(it->second, Out);
                } else {
                    throw InternalError((string)"Weird op with code: " + to_string(Op) +
                                        " which shouldn't really exist!\nAt: " + __FILE__ +
                                        ":" + to_string(__LINE__));
                }
            }
        }

    } /* end namespace Synth */
} /* end namespace ESMC */

//
// Solver.cpp ends here
