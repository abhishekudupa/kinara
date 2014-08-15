// LTSMonitors.cpp --- 
// 
// Filename: LTSMonitors.cpp
// Author: Abhishek Udupa
// Created: Fri Aug 15 12:15:37 2014 (-0400)
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

#include "LTSMonitors.hpp"
#include "LTSUtils.hpp"
#include "LabelledTS.hpp"
#include "LTSState.hpp"
#include "LTSTransitions.hpp"
#include "LTSFairnessSet.hpp"

namespace ESMC {
    namespace LTS {

        
        MonitorBase::MonitorBase(LabelledTS* TheLTS, const string& Name,
                                 const vector<ExpT>& Params, const ExpT& Constraint)
            : AutomatonBase(TheLTS, Name, Params, Constraint)
        {
            // Nothing here
        }

        MonitorBase::~MonitorBase()
        {
            // Nothing here
        }


        SafetyMonitor::SafetyMonitor(LabelledTS* TheLTS, const string& Name,
                                     const vector<ExpT>& Params, const ExpT& Constraint)
            : AutomatonBase(TheLTS, Name, Params, Constraint),
              MonitorBase(TheLTS, Name, Params, Constraint),
              EFSMBase(TheLTS, Name, Params, Constraint)
        {
            // Nothing here
        }

        SafetyMonitor::~SafetyMonitor()
        {
            // Nothing here
        }

        void SafetyMonitor::AddFairnessSet(const string& Name, FairSetFairnessType Fairness)
        {
            throw ESMCError((string)"Cannot add fairness to monitors");
        }

        void SafetyMonitor::AddOutputMsg(const ExprTypeRef& MessageType,
                                         const vector<ExpT>& Params)
        {
            throw ESMCError((string)"Cannot add output message to monitors");
        }

         void SafetyMonitor::AddOutputMsgs(const vector<ExpT>& NewParams,
                                           const ExpT& Constraint,
                                           const ExprTypeRef& MessageType,
                                           const vector<ExpT>& MessageParams)
         {
             throw ESMCError((string)"Cannot add output messages to monitors");
         }


        void SafetyMonitor::AddOutputTransition(const string& InitState,
                                                const string& FinalState,
                                                const ExpT& Guard,
                                                const vector<LTSAssignRef>& Updates,
                                                const string& MessageName,
                                                const ExprTypeRef& MessageType,
                                                const vector<ExpT>& MessageParams,
                                                const set<string>& AddToFairnessSets)
        {
            throw ESMCError((string)"Cannot add output transitions to monitors");
        }

        void SafetyMonitor::AddOutputTransitions(const vector<ExpT>& TransParams,
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
                                                 const string& SplatFairnessName)
        {
            throw ESMCError((string)"Cannot add output transitions to monitors");
        }

        void SafetyMonitor::AddInternalTransition(const string& InitState,
                                           const string& FinalState,
                                           const ExpT& Guard,
                                           const vector<LTSAssignRef>& Updates,
                                           const set<string>& AddToFairnessSets)
        {
            throw ESMCError((string)"Cannot add internal transitions to monitors");
        }

        void SafetyMonitor::AddInternalTransitions(const vector<ExpT>& TransParams,
                                                   const ExpT& Constraint,
                                                   const string& InitState,
                                                   const string& FinalState,
                                                   const ExpT& Guard,
                                                   const vector<LTSAssignRef>& Updates,
                                                   LTSFairnessType MessageFairness,
                                                   SplatFairnessType SplatFairness,
                                                   const string& SplatFairnessName)
        {
            throw ESMCError((string)"Cannot add internal transitions to monitors");
        }

        BuchiMonitor::BuchiMonitor(LabelledTS* TheLTS, const string& Name,
                                   const vector<ExpT>& Params, const ExpT& Constraint)
            : AutomatonBase(TheLTS, Name, Params, Constraint),
              MonitorBase(TheLTS, Name, Params, Constraint)
        {
            for (auto const& Param : Params) {
                if (!Param->GetType()->Is<ExprSymmetricType>()) {
                    throw ESMCError((string)"Buchi monitors can only be parametrized by " + 
                                    "symmetric types");
                }
            }
        }

        BuchiMonitor::~BuchiMonitor()
        {
            // Nothing here
        }

        void BuchiMonitor::AddTransition(const string& InitState,
                                         const string& FinalState,
                                         const ExpT& Guard)
        {
            CheckState(InitState);
            CheckState(FinalState);
            TheLTS->CheckExpr(Guard);
            auto IS = States[InitState];
            auto FS = States[FinalState];

            Transitions.push_back(new BuchiMonitorTransition(this, IS, FS, Guard));
        }

        void BuchiMonitor::AddFairnessByName(const string& AutomatonName, 
                                             const string& FairnessName, 
                                             const vector<ExpT>& Params)
        {
            // TODO: Implement me
        }

        void BuchiMonitor::AddFairnessesByName(const string& NewParams, 
                                               const ExpT& Constraint, 
                                               const string& AutomatonName, 
                                               const vector<ExpT> Params)
        {
            // TODO: Implement me
        }

        const vector<BuchiTransRef>& BuchiMonitor::GetTransitions() const
        {
            return Transitions;
        }

        string BuchiMonitor::ToString() const 
        {
            ostringstream sstr;

            sstr << "buchimonitor {" << endl;
            for (auto const& Trans : Transitions) {
                sstr << Trans->ToString(4) << endl;
            }
            sstr << "}" << endl;

            return sstr.str();
        }

    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// LTSMonitors.cpp ends here
