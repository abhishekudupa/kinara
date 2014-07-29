// UFEFSM.hpp --- 
// 
// Filename: UFEFSM.hpp
// Author: Abhishek Udupa
// Created: Wed Jul 23 19:33:33 2014 (-0400)
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

#if !defined ESMC_UF_EFSM_HPP_
#define ESMC_UF_EFSM_HPP_

#include "../common/FwdDecls.hpp"

namespace ESMC {
    namespace LTS {

        class UFLTS;
        typedef Exprs::ExprTypeRef ExprTypeRef;
        typedef Exprs::Expr<E, S> ExpT;

        // The class for an I/O EFSM which can contain uninterpreted functions
        class UFEFSM
        {
            friend class UFLTS;
            
        private:
            UFLTS* TheLTS;
            bool Finalized;
            vector<ExpT> Params;
            ExpT& Constraint;

        public:
            UFEFSM(UFLTS* TheLTS, 
                   const string& Name,
                   const vector<ExpT>& Params,
                   const ExpT& Constraint);

            ~UFEFSM();
            
            void AddInputMsg(const ExprTypeRef& MType);
            void AddInputMsg(const ExprTypeRef& MType,
                             const vector<ExpT>& Params,
                             const ExpT& Constraint);

            void AddOutputMsg(const ExprTypeRef& MType);
            void AddOutputMsg(const ExprTypeRef& MType,
                              const vector<ExpT>& Params,
                              const ExpT& Constraint);

            void AddState(const string& StateName,
                          bool Initial = false,
                          bool Final = false,
                          bool Accepting = false,
                          bool Error = false);

            // Add an internal, non-initial, non-final,
            // non-accepting, non-error state
            string AddState();
            
            void AddInputTransition(const string& InitState,
                                    const string& FinalState,
                                    const ExpT& Guard,
                                    const vector<Assignment<E, S>>& Updates,
                                    const string& MessageName,
                                    const LTSTypeRef& MessageType);

            void AddOutputTransition(const string& InitState,
                                     const string& FinalState,
                                     const ExpT& Guard,
                                     const vector<Assignment<E, S>>& Updates,
                                     const string& MessageName,
                                     const LTSTypeRef& MessageType,
                                     i32 FairnessSet = -1);

            void AddInternalTransition(const string& InitState,
                                       const string& FinalState,
                                       const ExpT& Guard,
                                       const vector<Assignment<E, S>>& Updates,
                                       const string& MessageName,
                                       const LTSTypeRef& MessageType,
                                       i32 FairnessSet = -1);

        };

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_UF_EFSM_HPP_ */

// 
// UFEFSM.hpp ends here
