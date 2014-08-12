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
#include "SymbolTable.hpp"

namespace ESMC {
    namespace LTS {

        class LabelledTS 
        {
        private:
            MgrT* Mgr;
            bool Frozen;
            bool MsgsFrozen;
            bool AutomataFrozen;
            map<string, ExprTypeRef> SymmTypes;
            map<string, ExprTypeRef> MsgTypes;
            map<string, ExprTypeRef> ParametricMsgTypes;
            map<ExprTypeRef, ExprTypeRef> TypeToPrimed;
            ExprTypeRef UnifiedMsgType;
            map<string, EFSMBase*> AllEFSMs;
            map<string, EFSMBase*> ActualEFSMs;
            map<string, ChannelEFSM*> ChannelEFSMs;
            map<string, SafetyMonitor*> SafetyMonitors;
            map<string, BuchiMonitor*> BuchiMonitors;
            vector<ExpT> StateVector;
            

            SymbolTable SymTab;

            inline void AssertFrozen() const;
            inline void AssertNotFrozen() const;
            inline void AssertMsgsFrozen() const;
            inline void AssertMsgsNotFrozen() const;
            inline void AssertAutomataFrozen() const;
            inline void AssertAutomataNotFrozen() const;

        public:
            LabelledTS();
            virtual ~LabelledTS();
            
            // typestate functions
            void FreezeMsgs();
            void FreezeAutomata();
            void Freeze();

            // All symmetric types must be declared here
            const ExprTypeRef& MakeSymmType(const string& Name, u32 Size);
            // All message types must be declared here
            const ExprTypeRef& MakeMsgType(const string& Name,
                                           const vector<pair<string, ExprTypeRef>>& Members,
                                           bool IncludePrimed = false);
            // All parametric message types must be declared here
            const ExprTypeRef& MakeMsgTypes(const vector<ExpT>& Params,
                                            const ExpT& Constraint,
                                            const string& Name,
                                            const vector<pair<string, ExprTypeRef>>& Members,
                                            bool IncludePrimed = false);
            

            MgrT* GetMgr() const;
            bool CheckMessageType(const ExprTypeRef& MsgType) const;
            const ExprTypeRef& GetUnifiedMType() const;

            const ExprTypeRef& GetPrimedType(const ExprTypeRef& Type) const;
            void CheckExpr(const ExpT& Expr) const;

            EFSMBase* MakeGenEFSM(const string& Name, const vector<ExpT>& Params,
                                  const ExpT& Constraint, LTSFairnessType Fairness);
            EFSMBase* MakeDetEFSM(const string& Name, const vector<ExpT>& Params,
                                  const ExpT& Constraint, LTSFairnessType Fairness);

            ChannelEFSM* MakeChannel(const string& Name, const vector<ExpT>& Params,
                                     const ExpT& Constraint, u32 Capacity, 
                                     bool Lossy, bool Ordered, bool Duplicating, 
                                     bool Blocking, LTSFairnessType Fairness);

            void AddInitStates(const vector<InitStateRef>& InitStates);
        };

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LABELLED_TS_HPP_ */

// 
// LabelledTS.hpp ends here










