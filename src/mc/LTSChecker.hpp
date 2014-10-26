// LTSChecker.hpp --- 
// 
// Filename: LTSChecker.hpp
// Author: Abhishek Udupa
// Created: Sun Aug 17 17:32:54 2014 (-0400)
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

#if !defined ESMC_LTS_CHECKER_HPP_
#define ESMC_LTS_CHECKER_HPP_

#include <boost/functional/hash.hpp>

#include "../common/FwdDecls.hpp"
#include "../uflts/LTSTypes.hpp"

#include "AQStructure.hpp"

namespace ESMC {
    namespace MC {

        using namespace ESMC::LTS;
        using ESMC::Symm::Canonicalizer;

        namespace Detail {
            
            class DFSStackEntry
            {
            private:
                StateVec* State;
                i64 LastFired;

            public:
                DFSStackEntry();
                DFSStackEntry(StateVec* State);
                DFSStackEntry(StateVec* State, u32 LastFired);
                DFSStackEntry(const DFSStackEntry& Other);
                ~DFSStackEntry();

                DFSStackEntry& operator = (const DFSStackEntry& Other);
                bool operator == (const DFSStackEntry& Other) const;

                i64& GetLastFired();
                i64 GetLastFired() const;
                const StateVec* GetState() const;
                StateVec* GetState();
                
                void SetLastFired(i64 NewLastFired);
            };

            class PStatePEdgePairHasher
            {
            public:
                inline u64 
                operator () (const pair<const ProductState*, const ProductEdge*>& ThePair) const
                {
                    u64 Retval = 0;
                    boost::hash_combine(Retval, ThePair.first);
                    boost::hash_combine(Retval, ThePair.second);
                    return Retval;
                }
            };

            class FairnessChecker
            {
                friend class ESMC::MC::TraceBase;
                friend class ESMC::MC::LTSChecker;

            private:
                LTSFairSetRef FairSet;
                // How many instances do I have
                u32 NumInstances;
                // Are we tracking a strong fairness
                bool IsStrong;
                // The system index set
                SystemIndexSet* SysIdxSet;
                // Status bits
                bool Enabled;
                bool Executed;
                bool Disabled;
                // The class (process class) id that this
                // fairness belongs to
                u32 ClassID;
                // Guarded commands I need to respond to
                // for each tracked index
                vector<vector<bool>> GCmdsToRespondTo;
                // Same info as above, but in the form of 
                // sets
                vector<unordered_set<u32>> GCmdIDsToRespondTo;
                // Set of states where I'm enabled, but 
                // not taken
                unordered_set<const ProductState*> EnabledStates;
                LTSChecker* Checker;

                // Info about how various fairnesses were
                // satisfied
                vector<bool> EnabledPerInstance;
                vector<bool> ExecutedPerInstance;
                vector<bool> DisabledPerInstance;

            public:
                FairnessChecker(const LTSFairSetRef& FairSet,
                                SystemIndexSet* SysIdxSet,
                                const vector<GCmdRef>& GuardedCommands,
                                LTSChecker* Checker);
                ~FairnessChecker();
                
                void ResetFairness();
                void ResetFull();
                void ProcessSCCState(const ProductState* State,
                                     const ProductEdgeSetT& Edges,
                                     u32 TrackedIndex);

                bool IsFair() const;
                bool IsStrongFairness() const;
                bool IsEnabled() const;
                bool IsDisabled() const;
                bool IsExecuted() const;

                bool IsEnabled(u32 InstanceID) const;
                bool IsDisabled(u32 InstanceID) const;
                bool IsExecuted(u32 InstanceID) const;

                const unordered_set<const ProductState*>& GetEnabledStates() const;
                const unordered_set<u32>& GetCmdIDsToRespondTo(u32 InstanceID) const;
            };

        } /* end namespace Detail */

        class LTSChecker
        {
            friend class Detail::FairnessChecker;
            friend class TraceBase;
            friend class Synth::Solver;

        private:
            LabelledTS* TheLTS;
            StateFactory* Factory;
            Canonicalizer* TheCanonicalizer;
            StateVec* ZeroState;
            StateVecPrinter* Printer;
            LTSCompiler* Compiler;
            AQStructure* AQS;
            ProductStructure* ThePS;
            vector<GCmdRef> GuardedCommands;
            u32 NumGuardedCmds;
            // Total number of processes
            u32 NumProcesses;
            // The system index set
            SystemIndexSet* SysIdxSet;
            // Fairness Checkers by class id
            vector<vector<Detail::FairnessChecker*>> FairnessCheckers;
            map<string, BuchiAutomatonBase*> AllBuchiAutomata;
            map<string, StateBuchiAutomaton*> StateBuchiAutomata;
            map<string, MsgBuchiAutomaton*> MsgBuchiAutomata;

            // Set of invariant expressions for bounds, etc.
            set<ExpT> BoundsInvariants;
            ExpT DeadlockFreeInvariant;

            // returns <Cmd, false> if successful
            // returns <null, false> if no more commands
            // returns <null, true> if exception
            inline const GCmdRef& GetNextEnabledCmd(StateVec* State, i64& LastFired,
                                                    bool& Exception);
            inline void DoDFS(StateVec* Root);
            inline void DoBFS(const vector<StateVec*>& Roots);

            inline void ConstructProduct(StateBuchiAutomaton* Monitor);
            inline vector<const ProductState*> GetAcceptingSCCs();
            
            inline void DoThreadedBFS(const ProductState* SCCRoot,
                                      u32 IndexID);

            inline bool CheckSCCFairness(const ProductState* SCCRoot, 
                                         vector<const ProductState*>& UnfairStates);

            inline unordered_set<const ProductState*> ExpandSCC(const ProductState* SCCRoot);
            inline vector<TraceBase*> MakeFairTrace(const ProductState* SCCRoot);
            
            inline set<ExpT> GatherTermsInIndex(const ExpT& Exp);

            void FoldTransMsgExp(const ExpT& Exp, const vector<GCmdRef>& Updates,
                                 u32 CurrentIndex);

            inline void MakeIndexTermInvariants(const ExpT& Precondition,
                                                const set<ExpT>& IndexTerms);
            inline void MakeBoundsInvariants();

        public:
            LTSChecker(LabelledTS* TheLTS);
            virtual ~LTSChecker();

            vector<TraceBase*> BuildAQS(const Z3Model& Model = Z3Model::NullModel,
                                        AQSConstructionMethod Method = 
                                        AQSConstructionMethod::BreadthFirst);
            void ClearAQS();

            StateBuchiAutomaton* MakeStateBuchiMonitor(const string& Name, 
                                                       const vector<ExpT>& SymmIndices,
                                                       const ExpT& Constraint);

            vector<TraceBase*> CheckLiveness(const string& BuchiMonitorName);

            LabelledTS* GetLTS() const;
            LTSCompiler* GetCompiler() const;

            // Methods to assist synthesis
            vector<TraceBase*> GetAllDeadlocks();
            vector<TraceBase*> GetAllSafetyViolations();
        };
        
        // Returns false if exception encountered
        extern bool ApplyUpdates(const vector<LTSAssignRef>& Updates, 
                                 const StateVec* InputState, 
                                 StateVec *OutputState);

        // Returns null if exception encountered
        extern StateVec* ExecuteCommand(const GCmdRef& Cmd,
                                        const StateVec* InputState);

        // returns <null, true> if command cannot be executed
        // returns <statevec, true> if command CAN be executed
        // returns <null, false> if command CAN be executed
        // but results in an exception when actually executing it
        // or if an error was encountered evaluating the guard of Cmd
        extern StateVec* TryExecuteCommand(const GCmdRef& Cmd,
                                           const StateVec* InputState,
                                           bool& Exception);

    } /* end namespace MC */
} /* end namespace ESMC */


#endif /* ESMC_LTS_CHECKER_HPP_ */

// 
// LTSChecker.hpp ends here
