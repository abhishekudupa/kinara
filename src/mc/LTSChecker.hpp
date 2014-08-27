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
            vector<GCmdRef> GuardedCommands;
            u32 NumGuardedCmds;
            map<string, BuchiAutomaton*> OmegaAutomata;

            inline const GCmdRef& GetNextEnabledCmd(StateVec* State, i64& LastFired);
            inline void DoDFS(StateVec* Root);
            inline void ApplyUpdates(const vector<LTSAssignRef>& Updates,
                                     const StateVec* InputState,
                                     StateVec* OutputState) const;

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
