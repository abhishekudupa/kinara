// LTSMonitors.hpp --- 
// 
// Filename: LTSMonitors.hpp
// Author: Abhishek Udupa
// Created: Fri Aug 15 12:12:39 2014 (-0400)
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

#if !defined ESMC_LTS_MONITORS_HPP_
#define ESMC_LTS_MONITORS_HPP_

#include "LTSTypes.hpp"
#include "LTSState.hpp"
#include "SymbolTable.hpp"

#include "LTSAutomaton.hpp"
#include "LTSEFSMBase.hpp"

namespace ESMC {
    namespace LTS {
        class MonitorBase : public virtual AutomatonBase
        {
        public:
            MonitorBase(LabelledTS* TheLTS, const string& Name,
                        const vector<ExpT>& Params, const ExpT& Constraint);
            virtual ~MonitorBase();

        };

        class SafetyMonitor : public MonitorBase, public EFSMBase
        {
        public:
            SafetyMonitor(LabelledTS* TheLTS, const string& Name,
                          const vector<ExpT>& Params, const ExpT& Constraint);
            virtual ~SafetyMonitor();

            virtual void AddFairnessSet(const string& Name, FairSetFairnessType Fairness) override;

            virtual void AddOutputMsg(const ExprTypeRef& MessageType,
                                      const vector<ExpT>& Params = vector<ExpT>()) override;

            virtual void AddOutputMsgs(const vector<ExpT>& NewParams, const ExpT& Constraint,
                                       const ExprTypeRef& MessageType,
                                       const vector<ExpT>& MessageParams) override;

            virtual void AddOutputTransition(const string& InitState,
                                             const string& FinalState,
                                             const ExpT& Guard,
                                             const vector<LTSAssignRef>& Updates,
                                             const string& MessageName,
                                             const ExprTypeRef& MessageType,
                                             const vector<ExpT>& MessageParams,
                                             const set<string>& AddToFairnessSets = 
                                             set<string>()) override;

            virtual void AddOutputTransitions(const vector<ExpT>& TransParams,
                                              const ExpT& Constraint,
                                              const string& InitState,
                                              const string& FinalState,
                                              const ExpT& Guard,
                                              const vector<LTSAssignRef>& Updates,
                                              const string& MessageName,
                                              const ExprTypeRef& MessageType,
                                              const vector<ExpT>& MessageParams,
                                              LTSFairnessType MessageFairness,
                                              SplatFairnessType SplatFairness,
                                              const string& SplatFairnessName) override;

            virtual void AddInternalTransition(const string& InitState,
                                               const string& FinalState,
                                               const ExpT& Guard,
                                               const vector<LTSAssignRef>& Updates,
                                               const set<string>& AddToFairnessSets = 
                                               set<string>()) override;

            virtual void AddInternalTransitions(const vector<ExpT>& TransParams,
                                                const ExpT& Constraint,
                                                const string& InitState,
                                                const string& FinalState,
                                                const ExpT& Guard,
                                                const vector<LTSAssignRef>& Updates,
                                                LTSFairnessType MessageFairness,
                                                SplatFairnessType SplatFairness,
                                                const string& SplatFairnessName) override;

        };

        class BuchiMonitor : public MonitorBase
        {
        private:
            vector<BuchiTransRef> Transitions;

        public:
            BuchiMonitor(LabelledTS* TheLTS, const string& Name,
                         const vector<ExpT>& Params, const ExpT& Constraint);
            virtual ~BuchiMonitor();

            virtual void AddTransition(const string& InitState, 
                                       const string& FinalState,
                                       const ExpT& Guard);

            void AddFairnessByName(const string& AutomatonName, 
                                   const string& FairnessName,
                                   const vector<ExpT>& Params);

            void AddFairnessesByName(const string& NewParams,
                                     const ExpT& Constraint,
                                     const string& AutomatonName,
                                     const vector<ExpT> Params);

            virtual const vector<BuchiTransRef>& GetTransitions() const;
            virtual string ToString() const override;
        };

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LTS_MONITORS_HPP_ */

// 
// LTSMonitors.hpp ends here
