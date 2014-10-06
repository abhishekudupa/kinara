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

#include "../common/FwdDecls.hpp"
#include "../containers/RefCountable.hpp"
#include "../uflts/LTSTypes.hpp"
#include "../uflts/LTSTermSemanticizer.hpp"

namespace ESMC {
    namespace TP {

        using ESMC::LTS::ExpT;
        using ESMC::LTS::LExpT;
        using ESMC::LTS::Detail::Z3Ctx;

        enum class TPResult {
            SATISFIABLE, UNSATISFIABLE, UNKNOWN
        };

        class Z3Solver
        {
        protected:
            Z3Ctx Ctx;
            
        private:
            Z3_solver Solver;
            
        public:
            Z3Solver();
            Z3Solver(const Z3Solver& Other);
            Z3Solver(Z3Ctx Ctx, Z3_solver Solver);
            Z3Solver(Z3Ctx Ctx);
            Z3Solver(Z3Solver&& Other);
            virtual ~Z3Solver();

            Z3Solver& operator = (Z3Solver Other);
            bool operator == (const Z3Solver& Other) const;

            string ToString() const;
            
            operator Z3_solver () const;
            Z3_solver GetSolver() const;

            static Z3Solver NullSolver;
        };

        class Z3Model
        {
        protected:
            Z3Ctx Ctx;
           
        private:
            Z3_model Model;

        public:
            Z3Model();
            Z3Model(const Z3Model& Other);
            Z3Model(Z3Ctx Ctx, Z3_model Model);
            Z3Model(Z3Model&& Other);
            virtual ~Z3Model();

            Z3Model& operator = (Z3Model Other);
            bool operator == (const Z3Model& Other) const;

            string ToString() const;

            operator Z3_model () const;
            Z3_model GetModel() const;

            static Z3Model NullModel;
        };

        class TheoremProver : public RefCountable
        {
        private:
            string Name;

        protected:
            stack<vector<ExpT>> AssertionStack;
            Z3Model TheModel;
            TPResult LastSolveResult;

        public:
            TheoremProver(const string& Name);
            virtual ~TheoremProver();
            
            virtual void Push();
            virtual vector<ExpT> Pop() const;
            virtual void Pop(u32 NumScopes) const;

            virtual void Assert(const ExpT& Assertion);
            virtual void Assert(const vector<ExpT>& Assertions);
            
            virtual TPResult CheckSat() const;
            virtual TPResult CheckValidity() const;

            // Ignores all the assertions on the stack
            virtual TPResult CheckSat(const ExpT& Assertion) const;
            virtual TPResult CheckValidity(const ExpT& Assertion) const;

            virtual ExpT Evaluate(const ExpT& Exp) const;
            virtual
        };

    } /* end namespace TP */
} /* end namespace ESMC */

#endif /* ESMC_THEOREM_PROVER_HPP_ */

// 
// TheoremProver.hpp ends here
