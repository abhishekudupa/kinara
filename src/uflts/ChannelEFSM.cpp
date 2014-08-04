// ChannelEFSM.cpp --- 
// 
// Filename: ChannelEFSM.cpp
// Author: Abhishek Udupa
// Created: Sun Aug  3 14:35:32 2014 (-0400)
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

#include "ChannelEFSM.hpp"
#include "UFLTSExtension.hpp"
#include "UFLTS.hpp"
#include "UFEFSM.hpp"

namespace ESMC {
    namespace LTS {

        // Channel EFSM implementation
        ChannelEFSM::ChannelEFSM(UFLTS* TheLTS, const string& Name, 
                                 const vector<ExpT>& Params,
                                 const ExpT& Constraint,
                                 u32 Capacity, bool Ordered, bool Lossy, 
                                 bool Duplicating, bool Blocking)
            : TheEFSM(new UFEFSM(TheLTS, Name, Params, Constraint)),
              TheLTS(TheLTS), Name(Name), Capacity(Capacity), 
              Ordered(Ordered), Lossy(Lossy), Duplicating(Duplicating), 
              Blocking(Blocking)
        {
            auto Mgr = TheLTS->GetMgr();
            // Sanity checks on params
            if (Blocking && !Lossy) {
                throw ESMCError((string)"Only Lossy channels can be declared Blocking");
            }
            TheEFSM->AddState("ChanInitState");
            if (Lossy) {
                TheEFSM->AddState("LossDecideState");
            }
            TheEFSM->FreezeStates();
            ValType = TheLTS->GetUnifiedMType();
            CountType = Mgr->MakeType<Exprs::ExprRangeType>(0, Capacity);
            ArrayType = Mgr->MakeType<Exprs::ExprArrayType>(CountType, ValType);

            TheEFSM->AddVariable("MsgCount", CountType);
            TheEFSM->AddVariable("MsgBuffer", ArrayType);
            if (Lossy) {
                TheEFSM->AddVariable("LastMsg", ValType);
            }
        }

        ChannelEFSM::~ChannelEFSM()
        {
            // Nothing here
        }

        const string& ChannelEFSM::GetName() const
        {
            return Name;
        }
        
        inline void ChannelEFSM::MakeInputTransition(const ExprTypeRef& MType, 
                                                     const vector<ExpT>& MParams, 
                                                     MessageFairnessType Fairness)
        {
            
        }

        inline void ChannelEFSM::MakeOutputTransition(const ExprTypeRef& MType, 
                                                      const vector<ExpT>& MParams, 
                                                      MessageFairnessType Fairness)
        {
            
        }

        void ChannelEFSM::AddMessage(const ExprTypeRef& MType,
                                     const vector<ExpT>& MParams,
                                     MessageFairnessType Fairness)
        {
            TheEFSM->AddInputMsg(MType, MParams);
            auto TypeAsRec = MType->SAs<Exprs::ExprRecordType>();
            TheEFSM->AddOutputMsg(TheLTS->GetMessageType(TypeAsRec->GetName()), 
                                  MParams);

            MakeInputTransition(MType, MParams, Fairness);
            MakeOutputTransition(MType, MParams, Fairness);
        }

        void ChannelEFSM::AddMessages(const ExprTypeRef& Type,
                                      const vector<ExpT>& Params,
                                      const ExpT& Constraint,
                                      MessageFairnessType Fairness)
        {
            // TODO
        }

        UFEFSM* ChannelEFSM::GetEFSM()
        {
            return TheEFSM;
        }

    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// ChannelEFSM.cpp ends here
