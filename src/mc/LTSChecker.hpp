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

            class FairnessChecker
            {
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
                vector<vector<u32>> GCmdIDsToRespondTo;
                // Set of states where I'm enabled, but 
                // not taken
                unordered_set<const ProductState*> EnabledStates;

            public:
                FairnessChecker(const LTSFairSetRef& FairSet,
                                SystemIndexSet* SysIdxSet,
                                const vector<GCmdRef>& GuardedCommands);
                ~FairnessChecker();
                
                void Reset();
                void ProcessSCCState(const ProductState* State,
                                     const ProductEdgeSetT& Edges,
                                     u32 TrackedIndex);

                bool IsFair() const;
                bool IsStrongFairness() const;
                const unordered_set<const ProductState*>& GetEnabledStates() const;
            };

        } /* end namespace Detail */

        class LTSChecker
        {
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
            map<string, BuchiAutomaton*> OmegaAutomata;

            inline const GCmdRef& GetNextEnabledCmd(StateVec* State, i64& LastFired);
            inline void DoDFS(StateVec* Root);
            inline void ApplyUpdates(const vector<LTSAssignRef>& Updates,
                                     const StateVec* InputState,
                                     StateVec* OutputState) const;
            inline void ConstructProduct(BuchiAutomaton* Monitor);
            inline vector<const ProductState*> GetAcceptingSCCs();
            
            inline void DoThreadedBFS(const ProductState* SCCRoot,
                                      u32 IndexID);

            inline bool CheckSCCFairness(const ProductState* SCCRoot, 
                                         vector<const ProductState*>& UnfairStates);

        public:
            LTSChecker(LabelledTS* TheLTS);
            virtual ~LTSChecker();

            void BuildAQS();
            void ClearAQS();

            BuchiAutomaton* MakeBuchiMonitor(const string& Name, 
                                             const vector<ExpT>& SymmIndices,
                                             const ExpT& Constraint);
            void CheckLiveness(const string& BuchiMonitorName);
        };

    } /* end namespace MC */
} /* end namespace ESMC */


#endif /* ESMC_LTS_CHECKER_HPP_ */

// 
// LTSChecker.hpp ends here
