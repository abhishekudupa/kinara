// LTSAnalyses.hpp --- 
// 
// Filename: Analyses.hpp
// Author: Abhishek Udupa
// Created: Fri Jul 11 11:49:35 2014 (-0400)
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

#if !defined ESMC_LTS_ANALYSES_HPP_
#define ESMC_LTS_ANALYSES_HPP_

#include "../common/FwdDecls.hpp"
#include "../expr/Expressions.hpp"
#include "../uflts/LTSTypes.hpp"
#include "../uflts/LTSFairnessSet.hpp"


namespace ESMC {
    namespace Analyses {

        using namespace LTS;

        class SubstitutorForWP : VisitorBaseT
        {
        private:
            MgrT* Mgr;
            MgrT::SubstMapT Subst;
            vector<ExpT> SubstStack;

        public:
            SubstitutorForWP(MgrT* Mgr, const MgrT::SubstMapT& Subst);
            virtual ~SubstitutorForWP();

            virtual void VisitVarExpression(const VarExpT* Exp) override;
            virtual void VisitConstExpression(const ConstExpT* Exp) override;
            inline virtual void VisitBoundVarExpression(const BoundVarExpT* Exp)
                override;
            virtual void VisitOpExpression(const OpExpT* Exp) override;
            virtual void VisitEQuantifiedExpression(const EQExpT* Exp)
                override;
            virtual void VisitAQuantifiedExpression(const AQExpT* Exp)
                override;

            static ExpT Do(MgrT* Mgr,
                           const ExpT& Exp,
                           const MgrT::SubstMapT& SubstMap);
        };

        class TraceAnalyses
        {
        public:
            static vector<ExpT> GetAllScalarLeaves(ExpT InitialExp);
            static set<LTSFairObjRef> GetLTSFairnessObjects(LTS::LabelledTS* TheLTS);

            static set<LTSFairObjRef>
            GetLoopFairnessObjects(LTS::LabelledTS* TheLTS,
                                   MC::LivenessViolation* LivenessViolation);

            static MgrT::SubstMapT
            TransitionSubstitutionsGivenTransMsg(const vector<LTSAssignRef>& Updates,
                                                 MgrT::SubstMapT SubstMapForTransMsg);

            static vector<GCmdRef> GuardedCommandsFromTrace(MC::TraceBase* Trace);

            static MgrT::SubstMapT 
            GetSubstitutionsForTransMsg(const vector<LTSAssignRef>& updates);

            static bool IsGuardedCommandEnabled(LTS::LabelledTS* TheLTS, 
                                                const MC::StateVec* StateVector, 
                                                LTS::GCmdRef GuardedCommand);

            static map<pair<LTS::EFSMBase*, vector<LTS::ExpT> >, string>
            AutomataStatesFromStateVector(LTS::LabelledTS* TheLTS, 
                                          const MC::StateVec* StateVector);

            static LTS::ExpT WeakestPrecondition(LTS::ExpT InitialPhi, 
                                                 MC::TraceBase* Trace);

            static LTS::ExpT 
            WeakestPreconditionForLiveness(LTS::LabelledTS* TheLTS, 
                                           MC::StateBuchiAutomaton* Monitor, 
                                           MC::LivenessViolation* LivenessViolation);
            
            static LTS::ExpT 
            SymbolicExecution(LTS::ExpT Phi, 
                              MC::TraceBase* Trace, 
                              vector<MgrT::SubstMapT>& symbolic_states);

            static vector<LTS::ExpT> 
            SymbolicExecution(LTS::LabelledTS* TheLTS, 
                              MC::TraceBase* Trace, 
                              vector<vector<MgrT::SubstMapT>>& symbolic_states);

            static map<pair<LTS::EFSMBase*, vector<LTS::ExpT>>, string>
            GuardedCommandInitialStates(LTS::GCmdRef GuardedCommand);
        };
    } /* end namespace Analyses */
} /* end namespace ESMC */

#endif /* ESMC_LTS_ANALYSES_HPP_ */

//
// LTSAnalyses.hpp ends here
