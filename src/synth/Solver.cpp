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
        const u32 LimitOnBound = 32;

        const string Solver::BoundsVarPrefix = (string)"__SynthBound__";
        const u32 Solver::MaxBound = 32;

        Solver::Solver(LTSChecker* Checker, const SolverOptionsT& Options)
            : Options(Options),
              TP(new Z3TheoremProver()),
              TheLTS(Checker->TheLTS),
              Compiler(Checker->Compiler),
              Checker(Checker),
              Bound(0),
              GuardedCommands(TheLTS->GetGuardedCmds()),
              UnveiledNewOps(false)
        {
            // push a scope onto the theorem prover and assert
            // true
            TP->Push();
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

            auto BoundsType = Mgr->MakeType<RangeType>(0, MaxBound);
            BoundsVariable = Mgr->MakeVar(BoundsVarPrefix, BoundsType);

            // initialize the bounds assertions
            for (u32 i = 0; i < MaxBound; ++i) {
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

        inline void Solver::CheckedAssert(const ExpT& Assertion)
        {
            if (AssertedConstraints.find(Assertion) != AssertedConstraints.end()) {
                // cout << "Not asserting previously asserted constraint:" << endl
                //      << Assertion->ToString() << endl;
                return;
            }

            AssertedConstraints.insert(Assertion);
            // cout << "Asserting: " << Assertion->ToString() << endl;
            TP->Assert(Assertion, Options.UnrollQuantifiers);
        }

        Solver::~Solver()
        {
            // Nothing here
        }


        inline tuple<ExpT, ExpT, ExpT, ExpT>
        Solver::CreateIndicatorSubsts(vector<TypeRef>& ExistsQVars,
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

            ExistsQVars.push_back(CurrentArg->GetType());
            ExistsQVars.push_back(CurrentArg->GetType());
            ExistsQVars.insert(ExistsQVars.end(), OtherArgTypes.begin(), OtherArgTypes.end());

            auto SubstExp1 = Mgr->BoundSubstitute(OtherArgSubstMap1, UpdateExp);
            auto SubstExp2 = Mgr->BoundSubstitute(OtherArgSubstMap2, UpdateExp);

            return make_tuple(SubstExp1, SubstExp2, ArgBoundVar, ArgPrimeBoundVar);
        }

        inline vector<ExpT>
        Solver::CreateArgDepConstraints(const ExpT& OpExpression,
                                        const ExpT& IdentityIndicatorExp)
        {
            auto Mgr = TheLTS->GetMgr();
            vector<ExpT> Retval;
            auto const& OpArgs = GetOpArgs(OpExpression);
            for (auto const& Arg : OpArgs) {
                vector<TypeRef> ExistsQVars;
                auto SubstExpPair =
                    CreateIndicatorSubsts(ExistsQVars, OpExpression, Arg);

                auto const& SubstExp1 = get<0>(SubstExpPair);
                auto const& SubstExp2 = get<1>(SubstExpPair);

                auto QBody = Mgr->MakeExpr(LTSOps::OpEQ, SubstExp1, SubstExp2);
                QBody = Mgr->MakeExpr(LTSOps::OpNOT, QBody);

                auto Antecedent = Mgr->MakeExists(ExistsQVars, QBody);
                if (IdentityIndicatorExp != ExpT::NullPtr) {
                    Antecedent = Mgr->MakeExpr(LTSOps::OpAND, Antecedent,
                                               Mgr->MakeExpr(LTSOps::OpNOT,
                                                             IdentityIndicatorExp));
                }

                auto IndicatorUIDStr = to_string(VarDepIndicatorUIDGenerator.GetUID());
                auto IndicatorVarName =
                    (string)"__var_dep_indicator_" + IndicatorUIDStr;
                auto IndicatorExp = Mgr->MakeVar(IndicatorVarName,
                                                 Mgr->MakeType<RangeType>(0, 1));
                // AllIndicators.insert(IndicatorExp);
                auto Consequent = Mgr->MakeExpr(LTSOps::OpEQ, IndicatorExp,
                                                Mgr->MakeVal("1", IndicatorExp->GetType()));
                auto Implication = Mgr->MakeExpr(LTSOps::OpIFF, Antecedent, Consequent);

                cout << "Asserting Indicator Implication:" << endl
                     << Implication->ToString() << endl << endl;
                CurrentAssertions.insert(Implication);
                Retval.push_back(IndicatorExp);
            }
            return Retval;
        }

        inline void Solver::CreateIndicators(const ExpT& OpExpression,
                                             const ExpT& LValueExp,
                                             bool IsGuard)
        {
            auto Mgr = TheLTS->GetMgr();

            auto const& AppArgs = GetOpArgs(OpExpression);
            const u32 NumArgs = AppArgs.size();
            auto FuncCostVarName = (string)"__func_cost_" +
                to_string(FunctionCostUIDGenerator.GetUID());
            auto FuncCostVarType = Mgr->MakeType<RangeType>(0, 1 + NumArgs);
            auto FuncCostVar = Mgr->MakeVar(FuncCostVarName, FuncCostVarType);
            AllIndicators.insert(FuncCostVar);
            const u32 OpCode = OpExpression->SAs<Exprs::OpExpression>()->GetOpCode();

            vector<TypeRef> QVarTypes;
            transform(AppArgs.begin(), AppArgs.end(), back_inserter(QVarTypes),
                      [&] (const ExpT& Exp) -> TypeRef
                      {
                          return Exp->GetType();
                      });
            MgrT::SubstMapT SubstMap;
            for (u32 i = 0; i < NumArgs; ++i) {
                SubstMap[AppArgs[i]] = Mgr->MakeBoundVar(QVarTypes[i], NumArgs - i - 1);
            }

            if (LValueExp != ExpT::NullPtr && !IsGuard) {
                auto Body = Mgr->MakeExpr(LTSOps::OpEQ, OpExpression, LValueExp);
                Body = Mgr->MakeExpr(LTSOps::OpNOT, Body);
                auto SubstBody = Mgr->BoundSubstitute(SubstMap, Body);
                auto ExistsExp = Mgr->MakeExists(QVarTypes, SubstBody);
                auto IdentityVarName = "__identity_update_indicator_" +
                    to_string(IdentityUpdateUIDGenerator.GetUID());
                auto IdentityVar = Mgr->MakeVar(IdentityVarName, Mgr->MakeType<BooleanType>());
                auto IdentityConstraint = Mgr->MakeExpr(LTSOps::OpIFF, ExistsExp,
                                                        Mgr->MakeExpr(LTSOps::OpNOT, IdentityVar));
                cout << "Asserting Identity Update Constraint:" << IdentityConstraint->ToString()
                     << endl << endl;
                CurrentAssertions.insert(IdentityConstraint);

                auto&& ArgDepConstraints = CreateArgDepConstraints(OpExpression, IdentityVar);

                auto Antecedent = Mgr->MakeExpr(LTSOps::OpNOT, IdentityVar);
                auto Consequent = MakeSum(ArgDepConstraints, Mgr, FuncCostVarType);
                Consequent = Mgr->MakeExpr(LTSOps::OpADD, Consequent,
                                           Mgr->MakeVal("1", FuncCostVarType));
                Consequent = Mgr->MakeExpr(LTSOps::OpGE, FuncCostVar, Consequent);
                auto ImpliesExp = Mgr->MakeExpr(LTSOps::OpIMPLIES, Antecedent, Consequent);
                cout << "Asserting Cost Constraint:" << endl << ImpliesExp->ToString()
                     << endl << endl;

                UpdateIndicatorExps[OpCode] = FuncCostVar;
                CurrentAssertions.insert(ImpliesExp);
            } else if (IsGuard) {
                auto Body = OpExpression;
                auto SubstBody = Mgr->BoundSubstitute(SubstMap, Body);
                auto ExistsExp = Mgr->MakeExists(QVarTypes, SubstBody);
                auto AllFalseIndicatorVarName = "__all_false_" +
                    to_string(AllFalseUIDGenerator.GetUID());
                auto AllFalseVar = Mgr->MakeVar(AllFalseIndicatorVarName,
                                                Mgr->MakeType<BooleanType>());
                auto NegAllFalse = Mgr->MakeExpr(LTSOps::OpNOT, AllFalseVar);
                auto AllFalseConstraint = Mgr->MakeExpr(LTSOps::OpIMPLIES, ExistsExp, NegAllFalse);

                cout << "Asserting All False Constraint:" << AllFalseConstraint->ToString()
                     << endl << endl;
                CurrentAssertions.insert(AllFalseConstraint);

                auto&& ArgDepConstraints = CreateArgDepConstraints(OpExpression, ExpT::NullPtr);

                auto Antecedent = NegAllFalse;
                auto Consequent = MakeSum(ArgDepConstraints, Mgr, FuncCostVarType);
                Consequent = Mgr->MakeExpr(LTSOps::OpADD, Consequent,
                                           Mgr->MakeVal("1", FuncCostVarType));
                Consequent = Mgr->MakeExpr(LTSOps::OpGE, FuncCostVar, Consequent);
                auto ImpliesExp = Mgr->MakeExpr(LTSOps::OpIMPLIES, Antecedent, Consequent);
                cout << "Asserting Cost Constraint:" << endl << ImpliesExp->ToString()
                     << endl << endl;
                GuardIndicatorExps[OpCode] = FuncCostVar;
                CurrentAssertions.insert(ImpliesExp);
            } else {
                auto&& ArgDepConstraints = CreateArgDepConstraints(OpExpression, ExpT::NullPtr);
                auto SumExp = MakeSum(ArgDepConstraints, Mgr, FuncCostVarType);
                auto Constraint = Mgr->MakeExpr(LTSOps::OpGE, FuncCostVar, SumExp);
                cout << "Asserting Cost Constraint:" << endl << Constraint->ToString()
                     << endl << endl;
                UpdateIndicatorExps[OpCode] = FuncCostVar;
                CurrentAssertions.insert(Constraint);
            }
        }

        inline void Solver::CreateIndicators(i64 OpCode)
        {
            auto const& GuardOpToExp = TheLTS->GuardOpToExp;
            auto const& UpdateOpToLValue = TheLTS->UpdateOpToUpdateLValue;
            auto const& AllOpToExp = TheLTS->AllOpToExp;

            // is this an lvalue update that requires
            // identity constraints?
            auto it1 = GuardOpToExp.find(OpCode);
            auto it2 = UpdateOpToLValue.find(OpCode);
            auto it3 = AllOpToExp.find(OpCode);

            if (it3 == AllOpToExp.end()) {
                throw InternalError((string)"Could not find an expression for OpCode: " +
                                    to_string(OpCode) + ".\nAt: " + __FILE__ + ":" +
                                    to_string(__LINE__));
            }

            if (it1 != GuardOpToExp.end()) {
                // This is a guard
                CreateIndicators(it3->second, ExpT::NullPtr, true);
            } else if (it2 == UpdateOpToLValue.end()) {
                // This is an update, but is exempt
                CreateIndicators(it3->second, ExpT::NullPtr, false);
            } else {
                // Update and non-exempt
                CreateIndicators(it3->second, it2->second.second, false);
            }
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
            CurrentAssertions.insert(Constraint);
        }

        inline void Solver::CreateGuardIndicator(i64 GuardOp)
        {
            auto Mgr = TheLTS->GetMgr();

            if (Options.GBoundMethod == GuardBoundingMethodT::PointBound) {

                auto FunType = Mgr->LookupUninterpretedFunction(GuardOp)->As<FuncType>();
                auto DomainTypes = FunType->GetArgTypes();
                vector<vector<string>> DomainElems;
                const u32 NumDomainTypes = DomainTypes.size();
                transform(DomainTypes.begin(), DomainTypes.end(), back_inserter(DomainElems),
                          [&] (const TypeRef& Type) -> vector<string>
                          {
                              return Type->GetElements();
                          });

                auto&& DomainCP = CrossProduct<string>(DomainElems.begin(), DomainElems.end());
                vector<ExpT> PointIndicators;
                for (auto const& DomainTuple : DomainCP) {
                    vector<ExpT> AppArgs(NumDomainTypes);
                    for (u32 i = 0; i < NumDomainTypes; ++i) {
                        AppArgs[i] = Mgr->MakeVal(DomainTuple[i], DomainTypes[i]);
                    }
                    auto AppExp = Mgr->MakeExpr(GuardOp, AppArgs);
                    auto IndicatorVarName = (string)"__guard_point_indicator_" +
                        to_string(GuardPointUIDGenerator.GetUID());
                    auto IndicatorVar = Mgr->MakeVar(IndicatorVarName,
                                                     Mgr->MakeType<RangeType>(0, 1));
                    PointIndicators.push_back(IndicatorVar);
                    auto OneExp = Mgr->MakeVal("1", Mgr->MakeType<RangeType>(0, 1));
                    auto ImpliesExp = Mgr->MakeExpr(LTSOps::OpIMPLIES,
                                                    AppExp,
                                                    Mgr->MakeExpr(LTSOps::OpEQ,
                                                                  IndicatorVar,
                                                                  OneExp));

                    cout << "Asserting Guard Point Constraint:" << endl
                         << ImpliesExp->ToString() << endl << endl;
                    CurrentAssertions.insert(ImpliesExp);
                }

                auto SumExp = MakeSum(PointIndicators, Mgr,
                                      Mgr->MakeType<RangeType>(0, PointIndicators.size()));
                auto SumIndicatorVarName = (string)"__guard_indicator_" +
                    to_string(GuardIndicatorUIDGenerator.GetUID());
                auto SumIndicatorVar =
                    Mgr->MakeVar(SumIndicatorVarName,
                                 Mgr->MakeType<RangeType>(0, PointIndicators.size()));
                auto EQExp = Mgr->MakeExpr(LTSOps::OpEQ, SumIndicatorVar, SumExp);
                GuardIndicatorExps[GuardOp] = SumIndicatorVar;

                cout << "Asserting Guard Indicator Constraint:" << endl
                     << EQExp->ToString() << endl << endl;
                CurrentAssertions.insert(EQExp);

            } else if (Options.GBoundMethod == GuardBoundingMethodT::NonFalseBound) {

                auto IndicatorUID = GuardIndicatorUIDGenerator.GetUID();
                string IndicatorVarName = (string)"__indicator_" + to_string(IndicatorUID);
                auto FunType = Mgr->LookupUninterpretedFunction(GuardOp)->As<FuncType>();
                IndicatorVarName += (string)"_" + FunType->GetName();
                auto const& DomainTypes = FunType->GetArgTypes();
                vector<ExpT> BoundArgs;
                const u32 NumDomainTypes = DomainTypes.size();
                for (u32 i = 0; i < NumDomainTypes; ++i) {
                    auto BoundVar = Mgr->MakeBoundVar(DomainTypes[i], NumDomainTypes - i - 1);
                    BoundArgs.push_back(BoundVar);
                }
                auto AppExp = Mgr->MakeExpr(GuardOp, BoundArgs);
                auto ExistsExp = Mgr->MakeExists(DomainTypes, AppExp);
                auto IndicatorType = Mgr->MakeType<RangeType>(0, 1);

                auto IndicatorVar = Mgr->MakeVar(IndicatorVarName, IndicatorType);
                GuardIndicatorExps[GuardOp] = IndicatorVar;

                auto Implies = Mgr->MakeExpr(LTSOps::OpIMPLIES, ExistsExp,
                                             Mgr->MakeExpr(LTSOps::OpEQ, IndicatorVar,
                                                           Mgr->MakeVal("1", IndicatorType)));
                cout << "Asserting Indicator Implication:" << endl
                     << Implies->ToString() << endl << endl;
                CurrentAssertions.insert(Implies);
            } else if (Options.GBoundMethod == GuardBoundingMethodT::VarDepBound) {
                CreateIndicators(GuardOp);
            } else {
                // No bounding, return
                return;
            }
        }

        inline void Solver::MakeStateIdenticalConstraints(const ExpT& Exp)
        {
            auto Mgr = TheLTS->GetMgr();
            auto ExpAsOp = Exp->As<OpExpression>();
            assert(ExpAsOp != nullptr);
            auto const& Args = GetOpArgs(Exp);

            vector<TypeRef> ArgTypes;
            transform(Args.begin(), Args.end(), back_inserter(ArgTypes),
                      [&] (const ExpT& Exp) -> TypeRef
                      {
                          return Exp->GetType();
                      });

            vector<TypeRef> QVarTypes = ArgTypes;
            QVarTypes.insert(QVarTypes.end(), ArgTypes.begin(), ArgTypes.end());
            const u32 NumArgs = ArgTypes.size();

            MgrT::SubstMapT Subst1;
            MgrT::SubstMapT Subst2;
            for (u32 i = 0; i < NumArgs; ++i) {
                auto BoundVarExp = Mgr->MakeBoundVar(ArgTypes[i], NumArgs - i - 1);
                Subst1[Args[i]] = BoundVarExp;
            }

            for (u32 i = 0; i < NumArgs; ++i) {
                auto BoundVarExp = Mgr->MakeBoundVar(ArgTypes[i], (2 * NumArgs) - i - 1);
                Subst2[Args[i]] = BoundVarExp;
            }

            auto App1 = Mgr->BoundSubstitute(Subst1, Exp);
            auto App2 = Mgr->BoundSubstitute(Subst2, Exp);

            auto Body = Mgr->MakeExpr(LTSOps::OpEQ, App1, App2);

            auto Constraint = Mgr->MakeForAll(QVarTypes, Body);
            cout << "State Variable Identical Constraint:" << endl
                 << Constraint->ToString() << endl << endl;
            CurrentAssertions.insert(Constraint);
        }

        inline void Solver::CreateUpdateIndicator(i64 UpdateOp)
        {
            auto Mgr = TheLTS->GetMgr();
            auto const& UpdateOpToLValue = TheLTS->UpdateOpToUpdateLValue;
            auto const& StateUpdateOpToExp = TheLTS->StateUpdateOpToExp;

            auto it = UpdateOpToLValue.find(UpdateOp);
            if (it == UpdateOpToLValue.end()) {
                // This must be an exempt lvalue, but it could be a state
                // variable
                auto it2 = StateUpdateOpToExp.find(UpdateOp);
                if (it2 == StateUpdateOpToExp.end()) {
                    // This is an exempt lvalue
                    if (Options.UBoundMethod == UpdateBoundingMethodT::VarDepBound) {
                        CreateIndicators(UpdateOp);
                        return;
                    } else {
                        return;
                    }
                }

                // This is a state variable.
                auto const& AppArgs = GetOpArgs(it2->second);
                const u32 NumArgs = AppArgs.size();
                auto FuncCostVarName = (string)"__func_cost_" +
                    to_string(FunctionCostUIDGenerator.GetUID());
                auto FuncCostVarType = Mgr->MakeType<RangeType>(0, NumArgs);
                auto FuncCostVar = Mgr->MakeVar(FuncCostVarName, FuncCostVarType);
                AllIndicators.insert(FuncCostVar);

                if (Options.SBoundMethod == StateUpdateBoundingMethodT::AllSame) {
                    MakeStateIdenticalConstraints(it2->second);
                } else if (Options.SBoundMethod == StateUpdateBoundingMethodT::VarDepBound) {
                    auto&& ArgDepConstraints = CreateArgDepConstraints(it2->second, ExpT::NullPtr);
                    auto SumExp = MakeSum(ArgDepConstraints, Mgr, FuncCostVarType);
                    auto Constraint = Mgr->MakeExpr(LTSOps::OpGE, FuncCostVar, SumExp);
                    UpdateIndicatorExps[UpdateOp] = FuncCostVar;
                    CurrentAssertions.insert(Constraint);
                } else {
                    // No bounding
                    return;
                }
                return;
            }

            // This has an LValue, is not a state update and is not exempt
            if (Options.UBoundMethod == UpdateBoundingMethodT::NonIdentityBound) {

                auto IndicatorUID = UpdateIndicatorUIDGenerator.GetUID();
                string IndicatorVarName =
                    (string)"__update_indicator_" + to_string(IndicatorUID);

                auto const& UpdateExp = it->second.first;
                auto const& LValue = it->second.second;

                auto const& OpArgs = GetOpArgs(UpdateExp);
                // remove the lvalue itself from the op args
                vector<TypeRef> ArgTypes;
                transform(OpArgs.begin(), OpArgs.end(), back_inserter(ArgTypes),
                          [&] (const ExpT& Exp) -> TypeRef
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
                auto IndicatorType = Mgr->MakeType<RangeType>(0, 1);
                auto IndicatorVar = Mgr->MakeVar(IndicatorVarName, IndicatorType);
                UpdateIndicatorExps[UpdateOp] = IndicatorVar;

                auto ImpliesExp = Mgr->MakeExpr(LTSOps::OpIMPLIES, ExistsExp,
                                                Mgr->MakeExpr(LTSOps::OpEQ, IndicatorVar,
                                                              Mgr->MakeVal("1", IndicatorType)));
                cout << "Asserting Update Indicator Implication:" << endl
                     << ImpliesExp->ToString() << endl << endl;
                CurrentAssertions.insert(ImpliesExp);
            } else if (Options.UBoundMethod == UpdateBoundingMethodT::VarDepBound) {
                CreateIndicators(UpdateOp);
            } else {
                return;
            }
        }

        inline void Solver::CreateBoundsConstraints(i64 UpdateOp)
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
            CurrentAssertions.insert(QExpr);
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
                    CurrentAssertions.insert(Constraint);
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
                    cout << Constraint->ToString() << endl;
                    CurrentAssertions.insert(Constraint);
                }
            }

            cout << "End of Symmetry constraints on updates:" << endl;

            // Mark the guard and its updates as unveiled
            UnveiledGuardOps.insert(Op);
            UnveiledNewOps = true;

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
                CreateBoundsConstraints(NewUpdateOp);
            }
            UpdateCommands();
        }

        void Solver::UnveilNonCompletionGuardOp(i64 Op)
        {
            UnveiledGuardOps.insert(Op);
            InterpretedOps.insert(Op);
            CreateGuardIndicator(Op);
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
            CurrentAssertions.insert(Pred);
            auto&& SynthOps = GetSynthOps(Pred);
            for (auto const& Op : SynthOps) {
                if (UnveiledGuardOps.find(Op) != UnveiledGuardOps.end()) {
                    continue;
                }
                UnveilGuardOp(Op);
            }
        }

        inline void Solver::HandleOneSafetyViolation(const StateVec* ErrorState,
                                                     const ExpT& BlownInvariant)
        {
            auto AQS = Checker->AQS;

            // cout << "Handling one safety violation, computing shortest path... ";
            // flush(cout);

            auto PPath = AQS->FindShortestPath(ErrorState, CostFunction);
            // auto PPath = AQS->FindPath(ErrorState);

            // cout << "Done!" << endl << "Unwinding trace... ";
            // flush(cout);

            auto Trace = TraceBase::MakeSafetyViolation(PPath, Checker, BlownInvariant);
            // cout << "Trace:" << endl << endl;
            // cout << Trace->ToString(1) << endl << endl;

            // cout << "Done!" << endl
            //      << "Got trace with " << Trace->GetTraceElems().size() << " steps" << endl
            //      << "Finding invariant that was blown... ";
            // flush(cout);

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
                                        "But the invariant:\n" + BlownInvariant->ToString() +
                                        "\nwas reported to have been blown!\n" +
                                        "At: " + __FILE__ + ":" + to_string(__LINE__));
                }
            }

            // cout << "Done!" << endl << "Blown Invariant: " << endl
            //      << ActualBlownInvariant->ToString() << endl << "Computing weakest pre... ";
            // flush(cout);

            auto&& WPConditions =
                TraceAnalyses::WeakestPrecondition(this, Trace, ActualBlownInvariant);
            for (auto const& Pred : WPConditions) {
                // cout << "Obtained Safety Pre:" << endl
                //      << Pred->ToString() << endl << endl;
                MakeAssertion(Pred);
            }

            delete Trace;
        }

        inline void Solver::HandleOneDeadlockViolation(const StateVec* ErrorState)
        {
            auto Mgr = TheLTS->GetMgr();
            auto AQS = Checker->AQS;

            // cout << "Handling one deadlock violation, computing shortest path... ";
            // flush(cout);

            auto PPath = AQS->FindShortestPath(ErrorState, CostFunction);

            // cout << "Done!" << endl << "Unwinding trace... ";
            // flush(cout);

            auto Trace = TraceBase::MakeDeadlockViolation(PPath, Checker);
            // cout << "Trace:" << endl << endl;
            // cout << Trace->ToString(1) << endl << endl;

            // cout << "Done!" << endl
            //      << "Got trace with " << Trace->GetTraceElems().size() << " steps" << endl
            //      << "Computing Disjuncts... ";
            // flush(cout);

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

            // cout << "Done!" << endl << "Computing weakest pre... ";
            // flush(cout);

            auto&& WPConditions =
                TraceAnalyses::WeakestPrecondition(this,
                                                   Trace->As<SafetyViolation>(),
                                                   GoodExp);


            for (auto const& Pred : WPConditions) {
                // cout << "Obtained Pre:" << endl << Pred->ToString() << endl << endl;
                MakeAssertion(Pred);
            }

            // cout << "Done!" << endl;
            // flush(cout);

            delete Trace;
        }

        inline void Solver::HandleSafetyViolations()
        {
            auto const& ErrorStates = Checker->GetAllErrorStates();
            auto const& DeadlockFreeInvar = Checker->DeadlockFreeInvariant;
            const u32 NumTotalErrors = ErrorStates.size();
            set<ExpT> BlownInvariantsCovered;
            cout << "Found " << NumTotalErrors << " error states in all!" << endl;
            cout << "Building constraints for " << NumTotalErrors
                 << " errors..." << endl;
            cout << "   % complete";
            u32 NumErrorsHandled = 0;
            u32 PercentComplete = 0;
            for (auto const& ErrorState : ErrorStates) {
                auto SVPtr = ErrorState.first;
                auto const& BlownInvariant = ErrorState.second;
                if (BlownInvariant == DeadlockFreeInvar) {
                    HandleOneDeadlockViolation(SVPtr);
                } else {
                    HandleOneSafetyViolation(SVPtr, BlownInvariant);
                }
                NumErrorsHandled++;
                auto NewPercentComplete = (NumErrorsHandled * 100) / NumTotalErrors;
                if (NewPercentComplete != PercentComplete) {
                    PercentComplete = NewPercentComplete;
                    cout << "\b\b\b\b\b\b\b\b\b\b\b\b\b" << setw(3)
                         << setfill('0') << PercentComplete << "% complete";
                    flush(cout);
                }
            }

            cout << endl << " Done!" << endl << endl;
        }

        inline void Solver::HandleLivenessViolation(const LivenessViolation* Trace,
                                                    StateBuchiAutomaton* Monitor)
        {
            auto Predicate =
                TraceAnalyses::WeakestPreconditionForLiveness(this, Monitor, Trace);
            cout << "Obtained predicate for liveness violation:" << endl
                 << Predicate->ToString() << endl << endl;
            MakeAssertion(Predicate);
            delete Trace;
        }

        inline void Solver::AssertBoundsConstraint()
        {
            if (!UnveiledNewOps) {
                return;
            }

            UnveiledNewOps = false;

            // We need to toss out the entire stack of assertions
            // and reassert them
            TP = new Z3TheoremProver();
            TP->Push();
            Ctx = TP->GetCtx();

            auto Mgr = TheLTS->GetMgr();
            vector<ExpT> Summands;
            ExpT SumExp = nullptr;

            // Sum over ALL the indicators
            for (auto const& IndexExp : GuardIndicatorExps) {
                Summands.push_back(IndexExp.second);
            }
            for (auto const& IndexExp : UpdateIndicatorExps) {
                Summands.push_back(IndexExp.second);
            }

            if (Summands.size() == 0) {
                return;
            } else if (Summands.size() == 1) {
                SumExp = Summands[0];
            } else {
                SumExp = Mgr->MakeExpr(LTSOps::OpADD, Summands);
            }

            auto EQExp = Mgr->MakeExpr(LTSOps::OpEQ, SumExp, BoundsVariable);

            cout << "Asserting Bounds Constraint: " << endl
                 << EQExp->ToString() << endl << endl;

            TP->Assert(EQExp, Options.UnrollQuantifiers);
            for (auto const& Assertion : AssertedConstraints) {
                TP->Assert(Assertion, Options.UnrollQuantifiers);
            }

            // Redo the assumptions in the new theorem prover
            CurrentAssumptions.clear();

            // initialize the bounds assertions
            for (u32 i = 0; i < MaxBound; ++i) {
                auto CurProp = Mgr->MakeVar("__assumption_prop_var_" + to_string(i),
                                            Mgr->MakeType<BooleanType>());
                LTS::LTSLCRef LTSCtx = new LTS::LTSLoweredContext(Ctx);
                auto LoweredProp = Mgr->LowerExpr(CurProp, LTSCtx);
                CurrentAssumptions.push_back(LoweredProp);
            }

            return;
        }

        inline void Solver::AssertCurrentConstraints()
        {
            for (auto const& Pred : CurrentAssertions) {
                CheckedAssert(Pred);
            }
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

        inline void Solver::PrintStats(ostream& Out)
        {
            auto SolveTimeInMicroSecs = (Stats.SolveEndTime -
                                         Stats.SolveStartTime).InMicroSeconds();
            Out << "Solver Stats:" << endl << endl;
            Out << "----------------------------------------------------------------"
                << endl;
            Out << "Solve Time: " << ((float)SolveTimeInMicroSecs / 1000000.0)
                << " (s)" << endl;
            Out << "Initial Asserts: " << Stats.InitialNumAssertions << endl;
            Out << "Final Asserts: " << Stats.FinalNumAssertions << endl;
            Out << "Num Iterations: " << Stats.NumIterations << endl;
            Out << "Total SMT Time: " << Stats.TotalSMTTime << " (uS)" << endl;
            Out << "Min SMT Time: " << Stats.MinSMTTime << " (uS)" << endl;
            Out << "Max SMT Time: " << Stats.MaxSMTTime << " (uS)" << endl;
            Out << "Avg SMT Time: "
                << ((float)Stats.TotalSMTTime / Stats.NumIterations)
                << " (uS)" << endl;
            Out << "Final Bound: " << Bound << endl;
            Out << "----------------------------------------------------------------"
                << endl << endl;
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

            bool FirstIteration = true;
            bool InitialConstraintsCounted = false;
            while (Bound <= LimitOnBound) {

                if (ResourceLimitManager::CheckMemOut()) {
                    Stats.SolveEndTime = TimeValue::GetTimeValue();
                    cout << "Memory limit reached. Aborting with Memout!" << endl << endl;
                    PrintStats(cout);
                    ResourceLimitManager::QueryEnd();
                    exit(1);
                }
                if (ResourceLimitManager::CheckTimeOut()) {
                    Stats.SolveEndTime = TimeValue::GetTimeValue();
                    cout << "CPU Time limit reached. Aborting with Timeout" << endl << endl;
                    PrintStats(cout);
                    ResourceLimitManager::QueryEnd();
                    exit(1);
                }

                // Assert any constraints we might have
                // from the previous iteration
                AssertCurrentConstraints();
                CurrentAssertions.clear();

                if (!FirstIteration && !InitialConstraintsCounted) {
                    InitialConstraintsCounted = true;
                    Stats.InitialNumAssertions = TP->GetNumAssertions();
                }

                Stats.FinalNumAssertions = TP->GetNumAssertions();

                AssertBoundsConstraint();

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
                    cout << "UNSAT! Relaxed bound to " << Bound << endl;
                    CurrentAssumptions.pop_front();
                    continue;
                } else if (TPRes == TPResult::UNKNOWN) {
                    ResourceLimitManager::QueryEnd();
                    throw ESMCError((string)"Could not solve constraints!");
                }

                // all good. extract a model
                auto const& Model = TP->GetModel();
                cout << "Model:" << endl << Model.ToString() << endl;
                // PrintSolution();
                Compiler->UpdateModel(Model, InterpretedOps, GuardIndicatorExps);

                // Okay, we're good to model check now
                Checker->ClearAQS();
                u32 CExBound = 0;
                if (FirstIteration) {
                    CExBound = UINT32_MAX;
                    FirstIteration = false;
                } else {
                    CExBound = Options.NumCExToProcess;
                }
                auto Safe = Checker->BuildAQS(AQSConstructionMethod::BreadthFirst, CExBound);

                cout << "Model Used:" << endl;
                PrintFinalSolution(cout);
                cout << "End of model" << endl << endl;

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
                    cout << "Found Correct Completion!" << endl;
                    cout << "With Bound = " << Bound << ", Model:" << endl;
                    PrintFinalSolution(cout);
                    PrintStats(cout);
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
            auto UFTypeAsFunc = UFType->As<FuncType>();
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
                auto ModelValue = TP->Evaluate(AppExp);
                cout << "-> " << ModelValue << endl;
            }
        }

        ExpT Solver::Evaluate(const ExpT& Input)
        {
            return TP->Evaluate(Input);
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
            auto Model = TP->GetModel();
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
            cout << "The solution is the following:" << endl;
            for (auto NewOpIndicatorVar : GuardIndicatorExps) {
                auto NewOp = NewOpIndicatorVar.first;
                auto IndicatorVar = NewOpIndicatorVar.second;
                auto IndicatorValue = TP->Evaluate(IndicatorVar);
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
