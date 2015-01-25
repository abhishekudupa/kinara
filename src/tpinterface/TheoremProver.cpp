// TheoremProver.cpp ---
//
// Filename: TheoremProver.cpp
// Author: Abhishek Udupa
// Created: Tue Oct  7 18:10:46 2014 (-0400)
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

#include "TheoremProver.hpp"
#include "../uflts/LTSDecls.hpp"

namespace ESMC {
    namespace TP {

        using LTS::BooleanType;
        using LTS::ArrayType;
        using LTS::UnionType;
        using LTS::FuncType;
        using LTS::IntegerType;
        using LTS::RangeType;
        using LTS::RecordType;
        using LTS::ScalarType;

        IncompleteTheoryException::IncompleteTheoryException(const ExpT& Expression) throw ()
            : Expression(Expression)
        {
            ExceptionInfo =
                (string)"Could not determine satisfiability of expression:\n" +
                Expression->ToString() + "\nThis could be due to incompleteness " +
                "in the theorem prover";
        }

        IncompleteTheoryException::~IncompleteTheoryException() throw ()
        {
            // Nothing here
        }

        const char* IncompleteTheoryException::what() const throw ()
        {
            return ExceptionInfo.c_str();
        }

        const ExpT& IncompleteTheoryException::GetExpression() const
        {
            return Expression;
        }

        const u32 Z3TheoremProver::MaxNumAssumptions = (1 << 20);

        // Z3TheoremProver implementation
        Z3TheoremProver::Z3TheoremProver()
            : Ctx(new Z3CtxWrapper()), TheModel(Z3Model::NullModel),
              Solver(Ctx), FlashSolver(Ctx),
              LastSolveResult(TPResult::UNKNOWN)
        {
            AssumptionVec = new Z3_ast[MaxNumAssumptions];
        }

        Z3TheoremProver::Z3TheoremProver(const Z3Ctx& Ctx)
            : Ctx(Ctx), TheModel(Z3Model::NullModel),
              Solver(Ctx), FlashSolver(Ctx),
              LastSolveResult(TPResult::UNKNOWN)
        {
            AssumptionVec = new Z3_ast[MaxNumAssumptions];
        }

        Z3TheoremProver::~Z3TheoremProver()
        {
            delete[] AssumptionVec;
        }

        void Z3TheoremProver::ClearSolution()
        {
            TheModel = Z3Model::NullModel;
        }

        void Z3TheoremProver::Push()
        {
            Z3_solver_push(*Ctx, Solver);
        }

        void Z3TheoremProver::Pop(u32 NumScopes)
        {
            Z3_solver_pop(*Ctx, Solver, NumScopes);
        }

        void Z3TheoremProver::Assert(const ExpT& Assertion, bool UnrollQuantifiers)
        {
            auto Mgr = Assertion->GetMgr();
            LTS::LTSLCRef LTSCtx = new LTS::LTSLoweredContext(Ctx);
            ExpT UnrolledExp = ExpT::NullPtr;

            if (UnrollQuantifiers) {
                UnrolledExp = Mgr->UnrollQuantifiers(Assertion, true);
            } else {
                UnrolledExp = Assertion;
            }
            auto LoweredAssertion = Mgr->LowerExpr(UnrolledExp, LTSCtx);

            ESMC_LOG_FULL(
                          "TheoremProver.Assertions",
                          Out_ << "Asserting Expr:" << endl
                               << Assertion << endl;
                          );

            ESMC_LOG_FULL(
                          "TheoremProver.Unrolled",
                          Out_ << "Asserting Unrolled Expr:"
                               << UnrolledExp << endl;
                          Out_ << "Asserting Lowered Expr:" << endl
                               << LoweredAssertion.ToString() << endl;
                          );

            Z3_solver_assert(*Ctx, Solver, LoweredAssertion);
            auto const& Assumptions = LTSCtx->GetAllAssumptions();
            assert(Assumptions.size() == 1);
            for (auto const& AssumptionSet : Assumptions) {
                for (auto const& Assumption : AssumptionSet) {
                    ESMC_LOG_FULL(
                                  "TheoremProver.Assertions",
                                  Out_ << "Asserting Assumption:" << endl
                                       << Assumption << endl;
                                  );

                    Z3_solver_assert(*Ctx, Solver, Assumption);
                }
            }
        }

        void Z3TheoremProver::Assert(const vector<ExpT>& Assertions, bool UnrollQuantifiers)
        {
            for (auto const& Assertion : Assertions) {
                Assert(Assertion, UnrollQuantifiers);
            }
        }

        void Z3TheoremProver::Assert(const Z3Expr& Assertion)
        {
            ESMC_LOG_FULL(
                          "TheoremProver.Assertions",
                          Out_ << "Asserting pre-lowered assertion:" << endl
                               << Assertion << endl;
                          );

            Z3_solver_assert(*Ctx, Solver, Assertion);
        }

        TPResult Z3TheoremProver::CheckSat()
        {
            auto ASTVec = Z3_solver_get_assertions(*Ctx, Solver);
            ESMC_LOG_MIN_SHORT(
                               Out_ << "Checking SAT with "
                                    << Z3_ast_vector_size(*Ctx, ASTVec)
                                    << " assertions... ";
                               );

            auto Res = Z3_solver_check(*Ctx, Solver);

            ESMC_LOG_MIN_SHORT(
                               Out_ << "Done!" << endl;
                               );

            if (Res == Z3_L_FALSE) {
                LastSolveResult = TPResult::UNSATISFIABLE;
            } else if (Res == Z3_L_TRUE) {
                LastSolveResult = TPResult::SATISFIABLE;
            } else {
                LastSolveResult = TPResult::UNKNOWN;
            }
            LastSolveWasFlash = false;
            TheModel = Z3Model::NullModel;
            return LastSolveResult;
        }

        TPResult Z3TheoremProver::CheckSat(const ExpT& Assertion, bool UnrollQuantifiers)
        {
            Z3_solver_reset(*Ctx, FlashSolver);

            auto Mgr = Assertion->GetMgr();
            LTS::LTSLCRef LTSCtx = new LTS::LTSLoweredContext(Ctx);

            ExpT UnrolledAssertion = ExpT::NullPtr;
            if (UnrollQuantifiers) {
                UnrolledAssertion = Mgr->UnrollQuantifiers(Assertion, true);
            } else {
                UnrolledAssertion = Assertion;
            }
            auto LoweredAssertion = Mgr->LowerExpr(Assertion, LTSCtx);

            Z3_solver_assert(*Ctx, FlashSolver, LoweredAssertion);

            auto const& Assumptions = LTSCtx->GetAllAssumptions();
            for (auto const& AssumptionSet : Assumptions) {
                for (auto const& Assumption : AssumptionSet) {
                    Z3_solver_assert(*Ctx, FlashSolver, Assumption);
                }
            }

            auto Res = Z3_solver_check(*Ctx, FlashSolver);
            if (Res == Z3_L_FALSE) {
                LastSolveResult = TPResult::UNSATISFIABLE;
            } else if (Res == Z3_L_TRUE) {
                LastSolveResult = TPResult::SATISFIABLE;
            } else {
                LastSolveResult = TPResult::UNKNOWN;
            }

            LastSolveWasFlash = true;
            TheModel = Z3Model::NullModel;
            return LastSolveResult;
        }

        TPResult Z3TheoremProver::CheckSatWithAssumptions(const vector<Z3Expr>& Assumptions)
        {
            const u32 NumAssumptions = Assumptions.size();
            for (u32 i = 0; i < NumAssumptions; ++i) {
                AssumptionVec[i] = Assumptions[i];
            }

            auto ASTVec = Z3_solver_get_assertions(*Ctx, Solver);
            ESMC_LOG_MIN_SHORT(

                               Out_ << "Checking SAT with "
                                    << Z3_ast_vector_size(*Ctx, ASTVec)
                                    << " assertions on stack... ";
                               );

            auto Res = Z3_solver_check_assumptions(*Ctx, Solver,
                                                   NumAssumptions,
                                                   AssumptionVec);
            ESMC_LOG_MIN_SHORT(
                               Out_ << "Done!" << endl;
                               );

            if (Res == Z3_L_TRUE) {
                LastSolveResult = TPResult::SATISFIABLE;
            } else if (Res == Z3_L_FALSE) {
                LastSolveResult = TPResult::UNSATISFIABLE;
            } else {
                LastSolveResult = TPResult::UNKNOWN;
            }

            LastSolveWasFlash = false;
            TheModel = Z3Model::NullModel;
            return LastSolveResult;
        }

        TPResult Z3TheoremProver::CheckSatWithAssumptions(const deque<Z3Expr>& Assumptions)
        {
            const u32 NumAssumptions = Assumptions.size();
            u32 i = 0;
            for (auto const& Assumption : Assumptions) {
                AssumptionVec[i] = Assumption;
                ++i;
            }

            auto ASTVec = Z3_solver_get_assertions(*Ctx, Solver);

            ESMC_LOG_MIN_SHORT(
                               Out_ << "Checking SAT with "
                                    << Z3_ast_vector_size(*Ctx, ASTVec)
                                    << " assertions on stack... " << endl;
                               );

            auto Res = Z3_solver_check_assumptions(*Ctx, Solver,
                                                   NumAssumptions,
                                                   AssumptionVec);
            ESMC_LOG_MIN_SHORT(
                               Out_ << "Done!" << endl;
                               );

            if (Res == Z3_L_TRUE) {
                LastSolveResult = TPResult::SATISFIABLE;
            } else if (Res == Z3_L_FALSE) {
                LastSolveResult = TPResult::UNSATISFIABLE;
            } else {
                LastSolveResult = TPResult::UNKNOWN;
            }

            LastSolveWasFlash = false;
            TheModel = Z3Model::NullModel;
            return LastSolveResult;
        }

        void Z3TheoremProver::Interrupt()
        {
            Z3_interrupt(*Ctx);
        }

        u64 Z3TheoremProver::GetNumAssertions() const
        {
            auto ASTVec = Z3_solver_get_assertions(*Ctx, Solver);
            return Z3_ast_vector_size(*Ctx, ASTVec);
        }

        ExpT Z3TheoremProver::Evaluate(const ExpT& Exp)
        {
            if (LastSolveResult != TPResult::SATISFIABLE) {
                throw ESMCError((string)"Z3TheoremProver::Evaluate() called, but " +
                                "last solve was not satisfiable. No model to evaluate " +
                                "expression over!");
            }

            auto ExpType = Exp->GetType();
            if (!ExpType->Is<ScalarType>()) {
                throw ESMCError((string)"Z3TheoremProver::Evaluate() called " +
                                "on non-scalar typed expression. This is not " +
                                "currently supported!");
            }

            // Get the model if not already done
            if (TheModel == Z3Model::NullModel) {
                TheModel = Z3Model(Ctx, Z3_solver_get_model(*Ctx,
                                                            (LastSolveWasFlash ?
                                                             FlashSolver : Solver)), this);
            }

            auto Mgr = Exp->GetMgr();

            LTS::LTSLCRef LTSCtx = new LTS::LTSLoweredContext(Ctx);
            auto LoweredExpr = Mgr->LowerExpr(Exp, LTSCtx);
            Z3_ast OutAst = nullptr;
            auto EvalRes = Z3_model_eval(*Ctx, TheModel, LoweredExpr, true, &OutAst);
            if (EvalRes == Z3_FALSE) {
                throw ESMCError((string)"Could not evaluate the expression in the model.\n" +
                                "Expression: " + Exp->ToString());
            }
            Z3Expr EvalLExpr(Ctx, OutAst);
            return Mgr->RaiseExpr(EvalLExpr, LTSCtx);
        }

        const Z3Model& Z3TheoremProver::GetModel()
        {
            if (LastSolveResult != TPResult::SATISFIABLE) {
                throw ESMCError((string)"Z3TheoremProver::GetModel() called, but " +
                                "last solve was not satisfiable. No model to return.");
            }
            if (TheModel == Z3Model::NullModel) {
                TheModel = Z3Model(Ctx, Z3_solver_get_model(*Ctx, (LastSolveWasFlash ?
                                                                   FlashSolver : Solver)),
                                   const_cast<Z3TheoremProver*>(this));
            }
            return TheModel;
        }

        const Z3Ctx& Z3TheoremProver::GetCtx() const
        {
            return Ctx;
        }

        const Z3Solver& Z3TheoremProver::GetSolver() const
        {
            return Solver;
        }

    } /* end namespace TP */
} /* end namespace ESMC */

//
// TheoremProver.cpp ends here
