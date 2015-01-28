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
#include <deque>
#include <boost/heap/fibonacci_heap.hpp>

#include "../common/ESMCFwdDecls.hpp"
#include "../uflts/LTSDecls.hpp"
#include "../utils/BitSet.hpp"

#include "AQStructure.hpp"

namespace ESMC {
    namespace MC {

        using namespace ESMC::LTS;
        using ESMC::Symm::Canonicalizer;

        enum class BFSPrioritizationMethodT {
            None, Bucketed, PrioQueue
        };

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

            // A configurable BFS queue that can use a prio queue
            // or a normal deque. Used to prioritize searches along
            // minimally tainted paths
            class BFSQueueT
            {
            private:
                deque<StateVec*>* BFSDeque;
                vector<deque<StateVec*>*>* PrioQueues;
                static const u32 MaxNumBuckets;
                u32 NumElems;

            public:
                BFSQueueT(bool UsePrioQueue = false);
                ~BFSQueueT();
                void Push(StateVec* State, u32 TaintLevel = 0);
                StateVec* Pop(u32& TaintLevel);
                u32 Size() const;

                template <typename ForwardIterator>
                inline void Push(const ForwardIterator& Begin,
                                 const ForwardIterator& End, u32 TaintLevel)
                {
                    for (auto it = Begin; it != End; ++it) {
                        Push(*it, TaintLevel);
                    }
                }
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

            class FairnessChecker : public Stringifiable
            {
                friend class ESMC::MC::TraceBase;
                friend class ESMC::MC::LTSChecker;

            private:
                // Info that needs to be reset before each ThreadedBFS
                bool Enabled;
                bool Disabled;
                bool Executed;
                // Set of states where I'm enabled, but not taken.
                // Needed for removing unfair states from SCCs.
                unordered_set<const ProductState*> EnabledStates;
                // end of info that needs to reset before each ThreadedBFS

                // The fairness set that I check satisfaction of
                LTSFairSetRef FairSet;
                // How many instances do I have
                u32 NumInstances;
                // Are we tracking a strong fairness
                bool IsStrong;
                // The global system index set
                SystemIndexSet* SysIdxSet;
                // The fairness class ID of the fairness set
                u32 ClassID;
                // Guarded commands I need to respond to for each instance
                vector<BitSet> GCmdsToRespondTo;
                // Same info as above, but in the form of sets
                vector<unordered_set<u32>> GCmdIDsToRespondTo;
                LTSChecker* Checker;


                // Fields that help in trace generation
                // These fields need to be reset for each SCC

                // The instances that have already satisfied in the trace so far
                // Info about how instances were satisfied
                BitSet EnabledPerInstance;
                BitSet ExecutedPerInstance;
                BitSet DisabledPerInstance;
                // End of fields that need to be reset for each SCC

                // This field is used only in trace generation
                mutable BitSet SatisfiedInTrace;
                // End of fields to help in trace generation

            public:
                FairnessChecker(const LTSFairSetRef& FairSet,
                                SystemIndexSet* SysIdxSet,
                                const vector<GCmdRef>& GuardedCommands,
                                LTSChecker* Checker);
                ~FairnessChecker();

                void InitializeForNewSCC();
                void InitializeForNewThread();

                // Reset everything as if just constructed
                void Reset();
                void AcceptSCCState(const ProductState* State,
                                     const ProductEdgeSetT& Edges,
                                     u32 CurrentTrackedIndex,
                                     u32 OriginalTrackedIndex);

                bool IsEnabled() const;
                bool IsExecuted() const;
                bool IsDisabled() const;

                bool IsFair() const;
                bool IsStrongFairness() const;
                virtual string ToString(u32 Verbosity = 0) const override;

                const unordered_set<const ProductState*>& GetEnabledStates() const;


                // Methods to help in trace generation
                void ClearTraceSatisfactionBits() const;
                bool IsInstanceSatisfiedInTrace(u32 Instance) const;
                void SetInstanceSatisfiedInTrace(u32 Instance) const;
                bool CheckInstanceSatisfaction(u32 Instance) const;
                bool CheckInstanceSatisfaction(u32 Instance,
                                               u32 CmdID,
                                               const ProductState* ReachedState) const;

                // Permutes the info stored in this checker
                // This is currently IRREVERSIBLE!
                // (no use case yet for undoing this)
                // Of course, one can remember the permutation
                // originally applied and then apply the
                // corresponding inverse permutation to reverse it!
                void Permute(const vector<u08>& Permutation);
                bool IsEnabled(u32 Instance) const;
                bool IsDisabled(u32 Instance) const;
                bool IsExecuted(u32 Instance) const;
                // end of methods to help in trace generation
            };

        } /* end namespace Detail */

        class LTSChecker
        {
            friend class Detail::FairnessChecker;
            friend class TraceBase;
            friend class Synth::Solver;

        public:
            typedef unordered_map<const StateVec*, ExpT,
                                  Detail::StateVecPtrHasher,
                                  Detail::StateVecPtrEquals> ErrorStateSetT;

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
            // Total number of fairness objects
            u32 NumFairnessObjects;
            // Total number of fairness sets
            u32 NumFairnessSets;
            // The system index set
            SystemIndexSet* SysIdxSet;
            // Fairness Checkers by class id
            vector<vector<Detail::FairnessChecker*>> FairnessCheckers;
            map<string, BuchiAutomatonBase*> AllBuchiAutomata;
            map<string, StateBuchiAutomaton*> StateBuchiAutomata;
            map<string, MsgBuchiAutomaton*> MsgBuchiAutomata;
            vector<string> BuchiMonitorNames;

            // Lowered invariant from the LTS
            ExpT LoweredInvariant;
            ExpT DeadlockFreeInvariant;
            ExpT LoweredDLFInvariant;

            // The set of all error states, mapping to the
            // invariant expression that was blown
            ErrorStateSetT ErrorStates;

            // A set of commands that need to be tested
            // these are all the commands that are "fully"
            // interpreted, i.e., have been "unlocked" and
            // have a model supporting them
            set<u32> InterpretedCommands;

            // returns <Cmd, false> if successful
            // returns <null, false> if no more commands
            // returns <null, true> if exception
            inline const GCmdRef& GetNextEnabledCmd(StateVec* State, i64& LastFired,
                                                    bool& Exception, ExpT& NEPred);

            inline bool RecordErrorState(const StateVec* ErrorState,
                                         const ExpT& BlownInvariant,
                                         u32 MaxErrors);

            inline void DoDFS(StateVec* Root, u32 NumErrors);

            inline void DoBFS(const vector<StateVec*>& Roots, u32 NumErrors,
                              bool PrioritizeNonTentative = false);

            inline void ConstructProduct(StateBuchiAutomaton* Monitor);
            inline vector<const ProductState*>
            GetAcceptingSCCsFromInitState(const ProductState* InitState,
                                          u32& LowestUnusedIndex,
                                          u32& LowestUnusedSCCID);
            inline vector<const ProductState*> GetAcceptingSCCs();

            inline void DoThreadedBFS(const ProductState* SCCRoot,
                                      u32 IndexID);

            inline bool CheckSCCFairness(const ProductState* SCCRoot,
                                         vector<const ProductState*>& UnfairStates);

            inline unordered_set<const ProductState*> ExpandSCC(const ProductState* SCCRoot);

        public:
            LTSChecker(LabelledTS* TheLTS);
            virtual ~LTSChecker();

            // Returns true if no errors encountered
            // false otherwise
            bool BuildAQS(AQSConstructionMethod Method =
                          AQSConstructionMethod::BreadthFirst,
                          bool PrioritizeNonTentative = false,
                          u32 NumErrors = UINT32_MAX);


            void ClearAQS();

            StateBuchiAutomaton* MakeStateBuchiMonitor(const string& Name,
                                                       const vector<ExpT>& SymmIndices,
                                                       const ExpT& Constraint);

            TraceBase* CheckLiveness(const string& BuchiMonitorName);

            LabelledTS* GetLTS() const;
            LTSCompiler* GetCompiler() const;
            AQStructure* GetAQS() const;
            ProductStructure* GetPS() const;

            const ErrorStateSetT& GetAllErrorStates() const;
            TraceBase* MakeTraceToError(const StateVec* ErrorState);
            const vector<string>& GetBuchiMonitorNames() const;
        };

        // Returns false if exception encountered
        extern bool ApplyUpdates(const vector<LTSAssignRef>& Updates,
                                 const StateVec* InputState,
                                 StateVec* OutputState,
                                 ExpT& NEPred);

        // Returns null if exception encountered
        extern StateVec* ExecuteCommand(const GCmdRef& Cmd,
                                        const StateVec* InputState,
                                        ExpT& NEPred);

        // returns <null, true> if command cannot be executed
        // returns <statevec, true> if command CAN be executed
        // returns <null, false> if command CAN be executed
        // but results in an exception when actually executing it
        // or if an error was encountered evaluating the guard of Cmd
        extern StateVec* TryExecuteCommand(const GCmdRef& Cmd,
                                           const StateVec* InputState,
                                           bool& Exception,
                                           ExpT& NEPred);

    } /* end namespace MC */
} /* end namespace ESMC */


#endif /* ESMC_LTS_CHECKER_HPP_ */

//
// LTSChecker.hpp ends here
