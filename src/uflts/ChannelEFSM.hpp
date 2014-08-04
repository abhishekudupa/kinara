// ChannelEFSM.hpp --- 
// 
// Filename: ChannelEFSM.hpp
// Author: Abhishek Udupa
// Created: Sun Aug  3 14:32:24 2014 (-0400)
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

#if !defined ESMC_CHANNEL_EFSM_HPP_
#define ESMC_CHANNEL_EFSM_HPP_

#include "../common/FwdDecls.hpp"
#include "LTSUtils.hpp"

namespace ESMC {
    namespace LTS {

        enum class MessageFairnessType {
            NONE, PLAIN, NOT_ALWAYS_LOST, NOT_ALWAYS_DUP,
            NOT_ALWAYS_LOST_DUP
        };

        // A class for channels
        // Really just a wrapper around 
        // UFEFSM, which instantiates 
        // transitions as messages are added
        class ChannelEFSM
        {
        private:
            UFEFSM* TheEFSM;
            UFLTS* TheLTS;
            string Name;
            u32 Capacity;
            bool Ordered;
            bool Lossy;
            bool Duplicating;
            bool Blocking;

            ExprTypeRef ValType;
            ExprTypeRef CountType;
            ExprTypeRef ArrayType;

            ExpT ArrayExp;
            ExpT CountExp;
            ExpT LastMsgExp;

            // helper functions
            inline void MakeInputTransition(const ExprTypeRef& MType,
                                            const vector<ExpT>& MParams,
                                            MessageFairnessType Fairness);

            inline void MakeOutputTransition(const ExprTypeRef& MType,
                                             const vector<ExpT>& MParams,
                                             MessageFairnessType Fairness);

        public:
            ChannelEFSM(UFLTS* TheLTS, 
                        const string& Name,
                        const vector<ExpT>& Params,
                        const ExpT& Constraint,
                        u32 Capacity, bool Ordered, 
                        bool Lossy, bool Duplicating,
                        bool Blocking);
            ~ChannelEFSM();

            const string& GetName() const;

            void AddMessage(const ExprTypeRef& MType,
                            const vector<ExpT>& MParams,
                            const MessageFairnessType = MessageFairnessType::NONE);

            void AddMessages(const ExprTypeRef& MType,
                             const vector<ExpT>& MParams,
                             const ExpT& MConstraint,
                             const MessageFairnessType = MessageFairnessType::NONE);
            
            UFEFSM* GetEFSM();
        };

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_CHANNEL_EFSM_HPP_ */

// 
// ChannelEFSM.hpp ends here
