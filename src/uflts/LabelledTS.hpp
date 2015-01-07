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

#include <tuple>

#include "LTSDecls.hpp"
#include "../decls/SymbolTable.hpp"

namespace ESMC {
    namespace LTS {

        using namespace Decls;

        class LabelledTS
        {
        private:
            friend class ChannelEFSM;
            friend class AutomatonBase;
            friend class EFSMBase;
            friend class DetEFSM;
            friend class IncompleteEFSM;
            friend class MC::LTSChecker;
            friend class MC::LTSCompiler;
            friend class MC::StateVecPrinter;
            friend class MC::BuchiAutomatonBase;
            friend class MC::StateBuchiAutomaton;
            friend class Synth::Solver;
            friend class ESMC::Analyses::TraceAnalyses;

            static const string ProductMsgName;

            MgrT* Mgr;
            bool Frozen;
            bool MsgsFrozen;
            bool AutomataFrozen;

            unordered_map<string, TypeRef> NamedTypes;
            map<string, TypeRef> SymmTypes;
            map<string, TypeRef> MsgTypes;
            map<string, TypeRef> ParametricMsgTypes;
            map<TypeRef, TypeRef> TypeToPrimed;
            TypeRef UnifiedMsgType;
            map<string, EFSMBase*> AllEFSMs;
            map<string, EFSMBase*> ActualEFSMs;
            map<string, ChannelEFSM*> ChannelEFSMs;
            set<TypeRef> UsedSymmTypes;

            map<TypeRef, vector<TypeRef>> ParamTypeInsts;
            map<TypeRef, vector<ExpT>> PInstToParams;
            map<TypeRef, TypeRef> PInstToParamType;
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
            // The actual channel, the expression and the instance id
            vector<tuple<ChannelEFSM*, ExpT, u32>> ChanBuffersToSort;

            u32 StateVectorSize;
            vector<ISGenRef> InitStateGenerators;
            vector<GCmdRef> GuardedCommands;

            SymbolTable SymTab;
            UIDGenerator AutomatonClassIDGen;

            void AssertFrozen() const;
            void AssertNotFrozen() const;
            void AssertMsgsFrozen() const;
            void AssertMsgsNotFrozen() const;
            void AssertAutomataFrozen() const;
            void AssertAutomataNotFrozen() const;

            void CheckTypeName(const string& Name) const;

            void CheckConsistency() const;
            GCmdRef MakeGuardedCommand(const vector<LTSTransRef>& ProductTrans) const;
            MgrT::SubstMapT ApplyPerm(const vector<vector<ExpT>>& ParamElems,
                                             const vector<u08>& Perm);
            void MakeMsgCanonMap();
            void InstantiateInitState(const InitStateRef& InitState);
            u32 GetAutomataClassUID();
            bool HasMsgLValue(const ExpT& Exp) const;
            vector<GCmdRef> ElimMsgFromCommands(const vector<GCmdRef>& Commands) const;

            ExpT InvariantExp;
            ExpT FinalCondExp;

            unordered_map<i64, pair<ExpT, ExpT>> UpdateOpToUpdateLValue;

            unordered_map<i64, ExpT> GuardOpToExp;
            unordered_map<i64, set<ExpT>> GuardSymmetryConstraints;
            unordered_map<i64, set<ExpT>> GuardMutualExclusiveSets;
            unordered_map<i64, set<ExpT>> GuardOpToUpdates;
            unordered_map<i64, set<ExpT>> GuardOpToUpdateSymmetryConstraints;
            unordered_map<i64, ExpT> StateUpdateOpToExp;
            unordered_map<i64, ExpT> AllOpToExp;

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
            const set<TypeRef>& GetUsedSymmTypes() const;
            u32 GetStateVectorSize() const;

            // methods for creating expressions
            // and types.
            // we create all expressions VIA the LTS,
            // this avoids us from having to check expressions,
            // etc.
            TypeRef MakeBoolType();
            TypeRef MakeRangeType(i64 Low, i64 High);
            TypeRef MakeRecordType(const string& Name,
                                       const vector<pair<string, TypeRef>>& Members);
            TypeRef MakeArrayType(const TypeRef& IndexType,
                                      const TypeRef& ValueType);
            TypeRef MakeEnumType(const string& Name, const set<string>& Members);
            TypeRef MakeFieldAccessType();
            // All symmetric types must be declared here
            const TypeRef& MakeSymmType(const string& Name, u32 Size);
            // All message types must be declared here
            const TypeRef& MakeMsgType(const string& Name,
                                           const vector<pair<string, TypeRef>>& Members,
                                           bool IncludePrimed = false);
            // All parametric message types must be declared here
            const TypeRef& MakeMsgTypes(const vector<ExpT>& Params,
                                            const ExpT& Constraint,
                                            const string& Name,
                                            const vector<pair<string, TypeRef>>& Members,
                                            bool IncludePrimed = false);
            const TypeRef& GetNamedType(const string& TypeName) const;

            // Get state var type for a named EFSM
            const TypeRef& GetEFSMType(const string& EFSMName) const;

            // Expressions
            ExpT MakeTrue();
            ExpT MakeFalse();
            ExpT MakeVar(const string& Name, const TypeRef& Type);
            ExpT MakeBoundVar(i64 Idx, const TypeRef& Type);
            ExpT MakeVal(const string& Value, const TypeRef& Type);
            ExpT MakeOp(i64 OpCode, const vector<ExpT>& Operands);
            ExpT MakeOp(i64 OpCode, const ExpT& Operand1);
            ExpT MakeOp(i64 OpCode, const ExpT& Operand1, const ExpT& Operand2);
            ExpT MakeOp(i64 OpCode, const ExpT& Operand1, const ExpT& Operand2,
                        const ExpT& Operand3);
            ExpT MakeOp(i64 OpCode, const ExpT& Operand1, const ExpT& Operand2,
                        const ExpT& Operand3, const ExpT& Operand4);

            ExpT MakeExists(const vector<TypeRef>& QVarTypes, const ExpT& Body);
            ExpT MakeForAll(const vector<TypeRef>& QVarTypes, const ExpT& Body);
            i64 MakeUF(const string& Name, const vector<TypeRef>& Domain,
                       const TypeRef& Range);


            bool CheckMessageType(const TypeRef& MsgType) const;
            const TypeRef& GetUnifiedMType() const;

            const TypeRef& GetPrimedType(const TypeRef& Type) const;

            template <typename T, typename... ArgTypes>
            inline T*
            MakeEFSM(const string& Name, const vector<ExpT>& Params,
                     const ExpT& Constraint, LTSFairnessType Fairness,
                     ArgTypes&&... Args)
            {
                AssertMsgsFrozen();
                AssertAutomataNotFrozen();
                if (AllEFSMs.find(Name) != AllEFSMs.end()) {
                    throw ESMCError((string)"A machine named \"" + Name + "\" has already " +
                                    "been created in the LTS");
                }

                auto Retval = new T(this, Name, Params, Constraint, Fairness);
                AllEFSMs[Name] = Retval;
                ActualEFSMs[Name] = Retval;
                return Retval;
            }

            GeneralEFSM* MakeGenEFSM(const string& Name, const vector<ExpT>& Params,
                                     const ExpT& Constraint, LTSFairnessType Fairness);
            DetEFSM* MakeDetEFSM(const string& Name, const vector<ExpT>& Params,
                                  const ExpT& Constraint, LTSFairnessType Fairness);
            vector<EFSMBase*> GetEFSMs(const function<bool(const EFSMBase*)>& MatchPred) const;

            ChannelEFSM* MakeChannel(const string& Name, const vector<ExpT>& Params,
                                     const ExpT& Constraint, u32 Capacity,
                                     bool Lossy, bool Ordered, bool Duplicating,
                                     bool Blocking, LTSFairnessType Fairness);

            void AddInitStates(const vector<InitStateRef>& InitStates);
            void AddInvariant(const ExpT& Invariant);

            const ExpT& GetInvariant() const;
            const ExpT& GetFinalCond() const;

            const vector<tuple<ChannelEFSM*, ExpT, u32>>& GetChanBuffersToSort() const;
        };

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LABELLED_TS_HPP_ */

//
// LabelledTS.hpp ends here
