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
#include "FrozenEFSM.hpp"

namespace ESMC {
    namespace LTS {

        // Channel EFSM implementation
        ChannelEFSM::ChannelEFSM(UFLTS* TheLTS, const string& Name, 
                                 const vector<ExpT>& Params,
                                 const ExpT& Constraint,
                                 u32 Capacity, bool Ordered, bool Lossy, 
                                 bool Duplicating, bool Blocking)
            : TheLTS(TheLTS), Name(Name), Params(Params), Constraint(Constraint),
              Capacity(Capacity), Ordered(Ordered), Lossy(Lossy),
              Duplicating(Duplicating), Blocking(Blocking)
        {
            CheckParams(Params, Constraint, SymTab, TheLTS->GetMgr());
            // Sanity checks on params
            if (Blocking && !Lossy) {
                throw ESMCError((string)"Only Lossy channels can be declared Blocking");
            }
            if (FiniteLoss && !Lossy) {
                throw ESMCError((string)"Only Lossy channels can be declared FiniteLoss");
            }
            if (FiniteDup && !Duplicating) {
                throw ESMCError((string)"Only Duplicating channels can be declared FiniteDup");
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

        void ChannelEFSM::AddMessage(const ExprTypeRef& Type)
        {
            if (!Type->Is<Exprs::ExprRecordType>()) {
                throw ESMCError((string)"Only record types can be messages");
            }
            if (!TheLTS->CheckMessageType(MType)) {
                throw ESMCError((string)"Message Type: " + MType->ToString() + 
                                " has not been declared in the LTS");
            }
            Messages.insert(Type);
        }

        void ChannelEFSM::AddMessage(const ExprTypeRef& Type,
                                     const vector<ExpT>& Params,
                                     const ExpT& Constraint)
        {
            
            if (!Type->Is<Exprs::ExprParametricType>()) {
                throw ESMCError((string)"Only parametric types can be used as parametric " + 
                                "messages");
            }
            SymTab.Push();
            CheckParams(Params, Constraint, SymTab, TheLTS->GetMgr());
            SymTab.Pop();
            PMessages.insert(Detail::ParametrizedMessage(Type, Params, Constraint));
        }

        vector<FrozenEFSM*> ChannelEFSM::ToEFSM()
        {
            
        }

    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// ChannelEFSM.cpp ends here
