// LabelledTS.hpp --- 
// 
// Filename: LabelledTS.hpp
// Author: Abhishek Udupa
// Created: Fri Aug  8 20:35:51 2014 (-0400)
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

#if !defined ESMC_LABELLED_TS_HPP_
#define ESMC_LABELLED_TS_HPP_

#include "LTSTypes.hpp"

namespace ESMC {
    namespace LTS {

        class LabelledTS 
        {
        private:
            MgrT* Mgr;
            bool Frozen;
            bool MsgsFrozen;
            bool AutomataFrozen;
            map<string, ExprTypeRef> MsgTypes;
            map<string, ExprTypeRef> ParametricMsgTypes;
            map<ExprTypeRef, ExprTypeRef> TypeToPrimed;
            ExprTypeRef UnifiedMsgType;
            vector<EFSMBase*> AllEFSMs;
            vector<EFSMBase*> ActualEFSMs;
            vector<ChannelEFSM*> ChannelEFSMs;
            vector<SafetyMonitor*> SafetyMonitors;
            vector<BuchiMonitor*> BuchiMonitors;

        public:
            
            MgrT* GetMgr() const;
            bool CheckMessageType(const ExprTypeRef& MsgType) const;
            const ExprTypeRef& GetUnifiedMType() const;
        };

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LABELLED_TS_HPP_ */

// 
// LabelledTS.hpp ends here
