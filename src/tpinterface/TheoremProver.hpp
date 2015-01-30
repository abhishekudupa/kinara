// TheoremProver.hpp ---
//
// Filename: TheoremProver.hpp
// Author: Abhishek Udupa
// Created: Mon Oct  6 10:51:31 2014 (-0400)
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

#if !defined ESMC_THEOREM_PROVER_HPP_
#define ESMC_THEOREM_PROVER_HPP_

#include <stack>
#include <vector>

#include <z3.h>

#include "../common/ESMCFwdDecls.hpp"
#include "../containers/RefCountable.hpp"
#include "../uflts/LTSDecls.hpp"
#include "../uflts/LTSTermSemanticizer.hpp"
#include "../containers/SmartPtr.hpp"

#include "Z3Objects.hpp"

namespace ESMC {
    namespace TP {

        using ESMC::LTS::ExpT;
        using ESMC::LTS::LExpT;

        enum class TPResult {
            SATISFIABLE, UNSATISFIABLE, UNKNOWN
        };

        class IncompleteTheoryException : public exception
        {
        private:
            ExpT Expression;
            string ExceptionInfo;

        public:
            IncompleteTheoryException(const ExpT& Expression) throw ();
            virtual ~IncompleteTheoryException() throw ();
            virtual const char* what() const throw () override;
            const ExpT& GetExpression() const;
        };

        class Z3TheoremProver : public RefCountable
        {
        private:
            Z3Ctx Ctx;
            Z3Model TheModel;
            Z3Solver Solver;
            Z3Solver FlashSolver;
            bool LastSolveWasFlash;
            Z3_ast* AssumptionVec;
            TPResult LastSolveResult;
            u32 IncSolverTimeout;
            vector<Z3Expr> ImmutableAssertions;

            static const u32 MaxNumAssumptions;

            inline void SetIncSolverTimeout();

        public:
            Z3TheoremProver(u32 IncSolverTimeout = UINT32_MAX);
            Z3TheoremProver(const Z3Ctx& Ctx);
            virtual ~Z3TheoremProver();

            void ClearSolution();
            void Push();
            void Pop(u32 NumScopes = 1);

            void Assert(const ExpT& Assertion, bool Immutable, bool UnrollQuantifiers);
            void Assert(const vector<ExpT>& Assertions, bool Immutable,
                        bool UnrollQuantifiers);
            void Assert(const Z3Expr& Assertion, bool Immutable);

            // Creates a new context and solver, translates
            // all assertions from the current solver into the
            // new context/solver, reasserts the translated constraints
            // into the new solver and returns
            // Used to clear all learned lemmas
            void Reset(u32 NewIncSolverTimeout = 0);

            // Same as reset, but retains only the assertions
            // marked immutable during the call to Assert
            void ResetToImmutable(u32 NewIncSolverTimeout = 0);

            TPResult CheckSat();
            TPResult CheckSatWithAssumptions(const vector<Z3Expr>& Assumptions);
            TPResult CheckSatWithAssumptions(const deque<Z3Expr>& Assumptions);
            TPResult CheckSat(const ExpT& Assertion, bool UnrollQuantifiers);

            void Interrupt();

            u64 GetNumAssertions() const;

            ExpT Evaluate(const ExpT& Exp);

            const Z3Model& GetModel();
            const Z3Ctx& GetCtx() const;
            const Z3Solver& GetSolver() const;
        };

    } /* end namespace TP */
} /* end namespace ESMC */

#endif /* ESMC_THEOREM_PROVER_HPP_ */

//
// TheoremProver.hpp ends here
