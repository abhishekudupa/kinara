// LTSEFSM.cpp --- 
// 
// Filename: LTSEFSM.cpp
// Author: Abhishek Udupa
// Created: Fri Aug  8 20:32:45 2014 (-0400)
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

#include "LTSEFSM.hpp"
#include "LTSUtils.hpp"
#include "LabelledTS.hpp"
#include "LTSState.hpp"
#include "LTSTransitions.hpp"
#include "LTSFairnessSet.hpp"

namespace ESMC {
    namespace LTS {

        GeneralEFSM::GeneralEFSM(LabelledTS* TheLTS, const string& Name,
                                 const vector<ExpT>& Params, 
                                 const ExpT& Constraint,
                                 LTSFairnessType Fairness)
              : EFSMBase(TheLTS, Name, Params, Constraint, Fairness)
        {
            // Nothing here
        }

        GeneralEFSM::~GeneralEFSM()
        {
            // Nothing here
        }
            
        DetEFSM::DetEFSM(LabelledTS* TheLTS, const string& Name,
                         const vector<ExpT>& Params, const ExpT& Constraint,
                         LTSFairnessType Fairness)
            : EFSMBase(TheLTS, Name, Params, Constraint, Fairness)
        {
            // Nothing here
        }


        DetEFSM::~DetEFSM()
        {
            // Nothing here
        }

        void DetEFSM::AddInputTransition(const string& InitState,
                                         const string& FinalState,
                                         const ExpT& Guard,
                                         const vector<LTSAssignRef>& Updates,
                                         const string& MessageName,
                                         const ExprTypeRef& MessageType,
                                         const vector<ExpT>& MessageParams)
        {
            // TODO: Check for determinism
            EFSMBase::AddInputTransition(InitState, FinalState, Guard, 
                                         Updates, MessageName, MessageType, 
                                         MessageParams);
        }

        void DetEFSM::AddInputTransitions(const vector<ExpT>& TransParams,
                                          const ExpT& Constraint,
                                          const string& InitState,
                                          const string& FinalState,
                                          const ExpT& Guard,
                                          const vector<LTSAssignRef>& Updates,
                                          const string& MessageName,
                                          const ExprTypeRef& MessageType,
                                          const vector<ExpT>& MessageParams)
        {
            // TODO: Check for determinism
            EFSMBase::AddInputTransitions(TransParams, Constraint,
                                          InitState, FinalState, Guard,
                                          Updates, MessageName, MessageType,
                                          MessageParams);
        }

        void DetEFSM::AddOutputTransition(const string& InitState,
                                          const string& FinalState,
                                          const ExpT& Guard,
                                          const vector<LTSAssignRef>& Updates,
                                          const string& MessageName,
                                          const ExprTypeRef& MessageType,
                                          const vector<ExpT>& MessageParams,
                                          const set<string>& AddToFairnessSets)
        {
            // TODO: Check for determinism
            EFSMBase::AddOutputTransition(InitState, FinalState, Guard,
                                          Updates, MessageName, MessageType,
                                          MessageParams, AddToFairnessSets);
        }

        void DetEFSM::AddOutputTransitions(const vector<ExpT>& TransParams, 
                                           const ExpT& Constraint, 
                                           const string& InitState, 
                                           const string& FinalState, 
                                           const ExpT &Guard, 
                                           const vector<LTSAssignRef>& Updates, 
                                           const string& MessageName, 
                                           const ExprTypeRef& MessageType, 
                                           const vector<ExpT>& MessageParams, 
                                           LTSFairnessType FairnessKind,
                                           SplatFairnessType SplatFairness,
                                           const string& SplatFairnessName)
        {
            // TODO: Check for determinism
            EFSMBase::AddOutputTransitions(TransParams, Constraint, InitState, 
                                           FinalState, Guard, Updates, MessageName, 
                                           MessageType, MessageParams, 
                                           FairnessKind, SplatFairness, 
                                           SplatFairnessName);
        }

        void DetEFSM::AddInternalTransition(const string& InitState,
                                            const string& FinalState,
                                            const ExpT& Guard,
                                            const vector<LTSAssignRef>& Updates,
                                            const set<string>& AddToFairnessSets)
        {
            // TODO: Check for determinism
            EFSMBase::AddInternalTransition(InitState, FinalState, Guard, 
                                            Updates, AddToFairnessSets);
        }

        void DetEFSM::AddInternalTransitions(const vector<ExpT>& TransParams,
                                             const ExpT& Constraint,
                                             const string& InitState,
                                             const string& FinalState,
                                             const ExpT& Guard,
                                             const vector<LTSAssignRef>& Updates,
                                             LTSFairnessType FairnessKind,
                                             SplatFairnessType SplatFairness,
                                             const string& SplatFairnessName)
        {
            // TODO: Check for determinism
            EFSMBase::AddInternalTransitions(TransParams, Constraint, InitState, 
                                             FinalState, Guard, Updates, 
                                             FairnessKind, SplatFairness, 
                                             SplatFairnessName);
        }
        
    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// LTSEFSM.cpp ends here
