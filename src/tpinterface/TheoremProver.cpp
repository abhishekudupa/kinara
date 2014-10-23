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

namespace ESMC {
    namespace TP {

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


        TheoremProver::TheoremProver(const string& Name)
            : Name(Name)
        {
            AssertionStack.push(vector<ExpT>());
        }

        TheoremProver::~TheoremProver()
        {
            // Nothing here
        }

        TPResult TheoremProver::GetLastSolveResult() const
        {
            return LastSolveResult;
        }

        const string& TheoremProver::GetName() const 
        {
            return Name;
        }

        void TheoremProver::ClearSolution() const
        {
            LastSolveResult = TPResult::UNKNOWN;
        }

        void TheoremProver::Push() const
        {
            AssertionStack.push(vector<ExpT>());
        }

        vector<ExpT> TheoremProver::Pop() const
        {
            auto Retval = AssertionStack.top();
            AssertionStack.pop();
            return Retval;
        }

        void TheoremProver::Pop(u32 NumScopes) const
        {
            for (u32 i = 0; i < NumScopes; ++i) {
                Pop();
            }
        }

        // Assumes that quantifiers are unrolled already
        void TheoremProver::Assert(const ExpT& Assertion,
                                   bool UnrollQuantifiers) const
        {
            if (!Assertion->GetType()->Is<Exprs::ExprBoolType>()) {
                throw ESMCError((string)"Attempted to assert a non-Boolean " + 
                                "expression.\nThe expression is:\n" + 
                                Assertion->ToString());
            }
            AssertionStack.top().push_back(Assertion);
        }
        
        // Again, assumes that quantifiers are unrolled already
        void TheoremProver::Assert(const vector<ExpT>& Assertions,
                                   bool UnrollQuantifiers) const
        {
            for (auto const& Assertion : Assertions) {
                if (!Assertion->GetType()->Is<Exprs::ExprBoolType>()) {
                    throw ESMCError((string)"Attempted to assert a non-Boolean " + 
                                    "expression.\nThe expression is:\n" + 
                                    Assertion->ToString());
                }
                AssertionStack.top().push_back(Assertion);
            }
        }

        
        // Z3TheoremProver implementation
        Z3TheoremProver::Z3TheoremProver()
            : TheoremProver("Z3TheoremProver"),
              Ctx(new Z3CtxWrapper()), TheModel(Z3Model::NullModel),
              Solver(Ctx), FlashSolver(Ctx)
        {
            // Push the default scope on
            Z3_solver_push(*Ctx, Solver);
        }

        Z3TheoremProver::Z3TheoremProver(const Z3Ctx& Ctx)
            : TheoremProver("Z3TheoremProver"),
              Ctx(Ctx), TheModel(Z3Model::NullModel),
              Solver(Ctx), FlashSolver(Ctx)
        {
            // Push the default scope on
            Z3_solver_push(*Ctx, Solver);
        }
        
        Z3TheoremProver::~Z3TheoremProver()
        {
            // Nothing here
        }

        void Z3TheoremProver::ClearSolution() const 
        {
            TheoremProver::ClearSolution();
            TheModel = Z3Model::NullModel;
        }

        void Z3TheoremProver::Push() const 
        {
            Z3_solver_push(*Ctx, Solver);
            TheoremProver::Push();
        }

        vector<ExpT> Z3TheoremProver::Pop() const
        {
            Z3_solver_pop(*Ctx, Solver, 1);
            return TheoremProver::Pop();
        }

        void Z3TheoremProver::Pop(u32 NumScopes) const
        {
            Z3_solver_pop(*Ctx, Solver, NumScopes);
            TheoremProver::Pop(NumScopes);
        }

        void Z3TheoremProver::Assert(const ExpT& Assertion,
                                     bool UnrollQuantifiers) const
        {
            auto Mgr = Assertion->GetMgr();
            LTS::LTSLCRef LTSCtx = new LTS::LTSLoweredContext(Ctx);
            ExpT UnrolledExp = ExpT::NullPtr;
            
            if (UnrollQuantifiers) {
                UnrolledExp = Mgr->UnrollQuantifiers(Assertion);
            } else {
                UnrolledExp = Assertion;
            }
            TheoremProver::Assert(Assertion);

            Z3_solver_assert(*Ctx, Solver, Mgr->LowerExpr(Assertion, LTSCtx));
            
            // Assert the constraints from the lowered context as well
            auto const& Assumptions = LTSCtx->GetAllAssumptions();
            for (auto const& AssumptionSet : Assumptions) {
                for (auto const& Assumption : AssumptionSet) {
                    Z3_solver_assert(*Ctx, Solver, Assumption);
                }
            }
        }

        void Z3TheoremProver::Assert(const vector<ExpT>& Assertions,
                                     bool UnrollQuantifiers) const
        {
            for (auto const& Assertion : Assertions) {
                Assert(Assertion, UnrollQuantifiers);
            }
        }

        TPResult Z3TheoremProver::CheckSat() const
        {
            auto Res = Z3_solver_check(*Ctx, Solver);
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

        TPResult Z3TheoremProver::CheckSat(const ExpT& Assertion,
                                           bool UnrollQuantifiers) const
        {
            Z3_solver_reset(*Ctx, FlashSolver);

            auto Mgr = Assertion->GetMgr();
            LTS::LTSLCRef LTSCtx = new LTS::LTSLoweredContext(Ctx);

            ExpT UnrolledAssertion = ExpT::NullPtr;
            if (UnrollQuantifiers) {
                UnrolledAssertion = Mgr->UnrollQuantifiers(Assertion);
            } else {
                UnrolledAssertion = Assertion;
            }
            auto LoweredAssertion = Mgr->LowerExpr(Assertion, LTSCtx);

            cout << "Checking Lowered Assertion For SAT:" << endl
                 << LoweredAssertion.ToString() << endl;

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
        
        ExpT Z3TheoremProver::Evaluate(const ExpT& Exp) const
        {
            if (LastSolveResult != TPResult::SATISFIABLE) {
                throw ESMCError((string)"Z3TheoremProver::Evaluate() called, but " + 
                                "last solve was not satisfiable. No model to evaluate " + 
                                "expression over!");
            }

            auto ExpType = Exp->GetType();
            if (!ExpType->Is<Exprs::ExprScalarType>()) {
                throw ESMCError((string)"Z3TheoremProver::Evaluate() called " + 
                                "on non-scalar typed expression. This is not " + 
                                "currently supported!");
            }

            // Get the model if not already done
            if (TheModel == Z3Model::NullModel) {
                TheModel = Z3Model(Ctx, Z3_solver_get_model(*Ctx, 
                                                         (LastSolveWasFlash ? 
                                                          FlashSolver : Solver)), 
                                   const_cast<Z3TheoremProver*>(this));
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

        const Z3Model& Z3TheoremProver::GetModel() const
        {
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
