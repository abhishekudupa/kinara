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
            friend class ChannelEFSM;
            friend class AutomatonBase;
            friend class EFSMBase;
            friend class MC::LTSChecker;
            friend class MC::LTSCompiler;
            friend class MC::StateVecPrinter;

            static const string ProductMsgName;

            MgrT* Mgr;
            bool Frozen;
            bool MsgsFrozen;
            bool AutomataFrozen;

            unordered_map<string, ExprTypeRef> NamedTypes;
            map<string, ExprTypeRef> SymmTypes;
            map<string, ExprTypeRef> MsgTypes;
            map<string, ExprTypeRef> ParametricMsgTypes;
            map<ExprTypeRef, ExprTypeRef> TypeToPrimed;
            ExprTypeRef UnifiedMsgType;
            map<string, EFSMBase*> AllEFSMs;
            map<string, EFSMBase*> ActualEFSMs;
            map<string, ChannelEFSM*> ChannelEFSMs;
            set<ExprTypeRef> UsedSymmTypes;

            map<ExprTypeRef, vector<ExprTypeRef>> ParamTypeInsts;
            map<ExprTypeRef, vector<ExpT>> PInstToParams;
            map<ExprTypeRef, ExprTypeRef> PInstToParamType;
            // MsgID -> Permutation -> MsgID
            vector<vector<u32>> MsgCanonMap;
            // MsgID -> TypeName
            vector<string> MsgTypeMap;

            vector<ExpT> StateVectorVars;
            // Map from automaton name to the set of valid parameter
            // instantiations
            map<string, set<vector<ExpT>>> ValidAutomata;
            
            // Channel buffers that need to be sorted
            // on every transition
            vector<pair<ExpT, ExpT>> ChanBuffersToSort;

            u32 StateVectorSize;
            vector<vector<LTSAssignRef>> InitStateGenerators;
            vector<GCmdRef> GuardedCommands;

            SymbolTable SymTab;

            inline void AssertFrozen() const;
            inline void AssertNotFrozen() const;
            inline void AssertMsgsFrozen() const;
            inline void AssertMsgsNotFrozen() const;
            inline void AssertAutomataFrozen() const;
            inline void AssertAutomataNotFrozen() const;

            inline void CheckTypeName(const string& Name) const;

            inline void CheckConsistency() const;
            inline GCmdRef MakeGuardedCommand(const vector<LTSTransRef>& ProductTrans) const;
            inline MgrT::SubstMapT ApplyPerm(const vector<vector<ExpT>>& ParamElems, 
                                             const vector<u32>& Perm);
            inline void MakeMsgCanonMap();

            ExpT InvariantExp;
            ExpT FinalCondExp;

            // Interface only for friends
            MgrT* GetMgr() const;

        public:
            LabelledTS();
            virtual ~LabelledTS();

            // typestate functions
            void FreezeMsgs();
            void FreezeAutomata();
            void Freeze();

            const string& GetProductMsgName() const;
            const vector<vector<u32>>& GetMsgCanonMap() const;
            const vector<string>& GetMsgTypeMap() const;

            // Accessors
            const vector<vector<LTSAssignRef>>& GetInitStateGenerators() const;
            const vector<GCmdRef>& GetGuardedCmds() const;
            const vector<ExpT>& GetStateVectorVars() const;
            const set<ExprTypeRef>& GetUsedSymmTypes() const;

            // methods for creating expressions
            // and types.
            // we create all expressions VIA the LTS, 
            // this avoids us from having to check expressions,
            // etc.
            ExprTypeRef MakeBoolType();
            ExprTypeRef MakeRangeType(i64 Low, i64 High);
            ExprTypeRef MakeRecordType(const string& Name, 
                                       const vector<pair<string, ExprTypeRef>>& Members);
            ExprTypeRef MakeArrayType(const ExprTypeRef& IndexType,
                                      const ExprTypeRef& ValueType);
            ExprTypeRef MakeEnumType(const string& Name, const set<string>& Members);
            ExprTypeRef MakeFieldAccessType();
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
            const ExprTypeRef& GetNamedType(const string& TypeName) const;

            // Get state var type for a named EFSM
            const ExprTypeRef& GetEFSMType(const string& EFSMName) const;

            // Expressions
            ExpT MakeTrue();
            ExpT MakeFalse();
            ExpT MakeVar(const string& Name, const ExprTypeRef& Type);
            ExpT MakeBoundVar(i64 Idx, const ExprTypeRef& Type);
            ExpT MakeVal(const string& Value, const ExprTypeRef& Type);
            ExpT MakeOp(i64 OpCode, const vector<ExpT>& Operands); 
            ExpT MakeOp(i64 OpCode, const ExpT& Operand1); 
            ExpT MakeOp(i64 OpCode, const ExpT& Operand1, const ExpT& Operand2); 
            ExpT MakeOp(i64 OpCode, const ExpT& Operand1, const ExpT& Operand2,
                        const ExpT& Operand3);
            ExpT MakeOp(i64 OpCode, const ExpT& Operand1, const ExpT& Operand2,
                        const ExpT& Operand3, const ExpT& Operand4);

            ExpT MakeExists(const vector<ExprTypeRef>& QVarTypes, const ExpT& Body);
            ExpT MakeForAll(const vector<ExprTypeRef>& QVarTypes, const ExpT& Body);
            i64 MakeUF(const string& Name, const vector<ExprTypeRef>& Domain,
                       const ExprTypeRef& Range);
            

            bool CheckMessageType(const ExprTypeRef& MsgType) const;
            const ExprTypeRef& GetUnifiedMType() const;

            const ExprTypeRef& GetPrimedType(const ExprTypeRef& Type) const;

            EFSMBase* MakeGenEFSM(const string& Name, const vector<ExpT>& Params,
                                  const ExpT& Constraint, LTSFairnessType Fairness);
            EFSMBase* MakeDetEFSM(const string& Name, const vector<ExpT>& Params,
                                  const ExpT& Constraint, LTSFairnessType Fairness);

            ChannelEFSM* MakeChannel(const string& Name, const vector<ExpT>& Params,
                                     const ExpT& Constraint, u32 Capacity, 
                                     bool Lossy, bool Ordered, bool Duplicating, 
                                     bool Blocking, LTSFairnessType Fairness);

            void AddInitStates(const vector<InitStateRef>& InitStates);
            void AddInvariant(const ExpT& Invariant);

            const ExpT& GetInvariant() const;
            const ExpT& GetFinalCond() const;
            const vector<pair<ExpT, ExpT>>& GetChanBuffersToSort() const;
        };

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LABELLED_TS_HPP_ */

// 
// LabelledTS.hpp ends here
