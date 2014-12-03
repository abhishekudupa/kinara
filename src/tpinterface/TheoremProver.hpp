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

        class TheoremProver : public RefCountable
        {
        private:
            string Name;

        protected:
            mutable stack<vector<ExpT>> AssertionStack;
            mutable TPResult LastSolveResult;

        public:
            TheoremProver(const string& Name);
            virtual ~TheoremProver();

            TPResult GetLastSolveResult() const;
            const string& GetName() const;

            virtual void ClearSolution() const;

            virtual void Push() const;
            virtual vector<ExpT> Pop() const;
            virtual void Pop(u32 NumScopes) const;

            virtual void Assert(const ExpT& Assertion,
                                bool UnrollQuantifiers) const;
            virtual void Assert(const vector<ExpT>& Assertions,
                                bool UnrollQuantifiers) const;

            virtual TPResult CheckSat() const = 0;
            // Ignores all the assertions on the stack
            virtual TPResult CheckSat(const ExpT& Assertion,
                                      bool UnrollQuantifiers) const = 0;

            // Evaluates only scalar typed expressions
            virtual ExpT Evaluate(const ExpT& Exp) const = 0;

            template <typename T, typename... ArgTypes>
            static inline TPRef MakeProver(ArgTypes&&... Args)
            {
                TPRef Retval = new T(forward<ArgTypes>(Args)...);
                return Retval;
            }

            template <typename T>
            inline T* As()
            {
                return dynamic_cast<T*>(this);
            }

            template <typename T>
            inline const T* As() const
            {
                return dynamic_cast<const T*>(this);
            }

            template <typename T>
            inline T* SAs()
            {
                return static_cast<T*>(this);
            }

            template <typename T>
            inline const T* SAs() const
            {
                return static_cast<const T*>(this);
            }

            template <typename T>
            inline bool Is() const
            {
                return (dynamic_cast<const T*>(this) != nullptr);
            }
        };

        class Z3TheoremProver : public TheoremProver
        {
        private:
            Z3Ctx Ctx;
            mutable Z3Model TheModel;
            Z3Solver Solver;
            Z3Solver FlashSolver;
            mutable bool LastSolveWasFlash;

        public:
            Z3TheoremProver();
            Z3TheoremProver(const Z3Ctx& Ctx);
            virtual ~Z3TheoremProver();

            virtual void ClearSolution() const override;
            virtual void Push() const override;
            virtual vector<ExpT> Pop() const override;
            virtual void Pop(u32 NumScopes) const override;

            virtual void Assert(const ExpT& Assertion,
                                bool UnrollQuantifiers = false) const override;
            virtual void Assert(const vector<ExpT>& Assertions,
                                bool UnrollQuantifiers = false) const override;

            void Assert(const Z3Expr& Assertion) const;

            virtual TPResult CheckSat() const override;
            virtual TPResult CheckSat(const ExpT& Assertion,
                                      bool UnrollQuantifiers = false) const override;

            virtual ExpT Evaluate(const ExpT& Exp) const override;

            const Z3Model& GetModel() const;
            const Z3Ctx& GetCtx() const;
            const Z3Solver& GetSolver() const;
        };

    } /* end namespace TP */
} /* end namespace ESMC */

#endif /* ESMC_THEOREM_PROVER_HPP_ */

//
// TheoremProver.hpp ends here
