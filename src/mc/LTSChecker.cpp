// LTSChecker.cpp --- 
// 
// Filename: LTSChecker.cpp
// Author: Abhishek Udupa
// Created: Tue Aug 19 04:03:22 2014 (-0400)
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

#include <stack>
#include <deque>

#include "../uflts/LabelledTS.hpp"
#include "../symmetry/SymmCanonicalizer.hpp"
#include "../uflts/LTSAssign.hpp"
#include "../uflts/LTSTransitions.hpp"
#include "../uflts/LTSFairnessSet.hpp"
#include "../uflts/LTSEFSMBase.hpp"

#include "LTSChecker.hpp"
#include "Compiler.hpp"
#include "StateVec.hpp"
#include "StateVecPrinter.hpp"
#include "AQStructure.hpp"
#include "OmegaAutomaton.hpp"
#include "IndexSet.hpp"

namespace ESMC {
    namespace MC {

        using namespace ESMC::LTS;
        using ESMC::Symm::Canonicalizer;

        namespace Detail {

            DFSStackEntry::DFSStackEntry()
                : State(nullptr), LastFired(-1)
            {
                // Nothing here
            }

            DFSStackEntry::DFSStackEntry(StateVec* State)
                : State(State), LastFired(-1)
            {
                // Nothing here
            }

            DFSStackEntry::DFSStackEntry(StateVec* State, u32 LastFired)
                : State(State), LastFired(LastFired)
            {
                // Nothing here
            }

            DFSStackEntry::DFSStackEntry(const DFSStackEntry& Other)
                : State(Other.State), LastFired(Other.LastFired)
            {
                // Nothing here
            }

            DFSStackEntry::~DFSStackEntry()
            {
                // Nothing here
            }

            DFSStackEntry& DFSStackEntry::operator = (const DFSStackEntry& Other)
            {
                if (&Other == this) {
                    return *this;
                }
                State = Other.State;
                LastFired = Other.LastFired;
                return *this;
            }

            bool DFSStackEntry::operator == (const DFSStackEntry& Other) const
            {
                return (State == Other.State && LastFired == Other.LastFired);
            }

            i64& DFSStackEntry::GetLastFired() 
            {
                return LastFired;
            }

            i64 DFSStackEntry::GetLastFired() const
            {
                return LastFired;
            }

            const StateVec* DFSStackEntry::GetState() const
            {
                return State;
            }

            StateVec* DFSStackEntry::GetState()
            {
                return State;
            }

            void DFSStackEntry::SetLastFired(i64 NewLastFired)
            {
                LastFired = NewLastFired;
            }

        } /* end namespace Detail */

        using namespace Detail;

        LTSChecker::LTSChecker(LabelledTS* TheLTS)
            : TheLTS(TheLTS), AQS(nullptr), ThePS(nullptr)
        {
            Compiler = new LTSCompiler();
            Compiler->CompileLTS(TheLTS);
            Factory = new StateFactory(TheLTS->StateVectorSize,
                                       TheLTS->GetUnifiedMType()->GetByteSize());
            Printer = new StateVecPrinter(TheLTS, Compiler);
            TheCanonicalizer = new Canonicalizer(TheLTS, Printer);
            GuardedCommands = TheLTS->GetGuardedCmds();
            ZeroState = Factory->MakeState();
            NumGuardedCmds = GuardedCommands.size();

            NumProcesses = 0;
            NumEnExBits = 0;

            for (auto const& NameEFSM : TheLTS->AllEFSMs) {
                auto EFSM = NameEFSM.second;
                NumProcesses += EFSM->GetNumInstances();

                auto const& AllFairnesses = EFSM->GetAllFairnessSets();
                auto const& AllFairnessSets = AllFairnesses->GetFairnessSets();

                for (auto const& NameFSet : AllFairnessSets) {
                    NumEnExBits += NameFSet.second->GetNumInstances();
                    FairnessSets.push_back(NameFSet.second);
                }
            }
        }

        LTSChecker::~LTSChecker()
        {
            Factory->TakeState(ZeroState);
            delete TheLTS;
            delete Compiler;
            delete Factory;
            delete TheCanonicalizer;
            delete Printer;
            if (AQS != nullptr) {
                delete AQS;
            }
            if (ThePS != nullptr) {
                delete ThePS;
            }

            for (auto const& Aut : OmegaAutomata) {
                delete Aut.second;
            }
        }

        inline void LTSChecker::ApplyUpdates(const vector<LTSAssignRef>& Updates, 
                                             const StateVec* InputState, 
                                             StateVec *OutputState) const
        {
            for (auto const& Update : Updates) {
                auto const& LHS = Update->GetLHS();
                auto const& RHS = Update->GetRHS();
                auto const& LHSInterp = LHS->ExtensionData.Interp->SAs<LValueInterpreter>();
                auto const& RHSInterp = RHS->ExtensionData.Interp;
                if (LHSInterp->IsScalar()) {
                    LHSInterp->WriteScalar(RHSInterp->EvaluateScalar(InputState),
                                           OutputState);
                } else {
                    LHSInterp->Write(RHSInterp->Evaluate(InputState),
                                     OutputState);
                }
            }
            return;
        }

        inline const GCmdRef& LTSChecker::GetNextEnabledCmd(StateVec* State, i64& LastFired)
        {
            while(++LastFired < NumGuardedCmds) {
                auto const& Guard = GuardedCommands[LastFired]->GetGuard();
                auto GRes = Guard->ExtensionData.Interp->EvaluateScalar(State);
                if (GRes != 0) {
                    return GuardedCommands[LastFired];
                }
            }
            return GCmdRef::NullPtr;
        }

        inline void LTSChecker::DoDFS(StateVec *Root)
        {
            stack<DFSStackEntry> DFSStack;
            AQS->InsertInitState(Root);
            DFSStack.push(DFSStackEntry(Root));
            auto PermSet = TheCanonicalizer->GetPermSet();

            while (DFSStack.size() > 0) {
                auto& CurEntry = DFSStack.top();
                auto State = CurEntry.GetState();
                
                // cout << "Considering State:" << endl;
                // cout << "--------------------------------------------------------" << endl;
                // Printer->PrintState(State, cout);
                // cout << "--------------------------------------------------------" << endl;

                auto& LastFired = CurEntry.GetLastFired();
                bool Deadlocked = (LastFired == -1);
                auto const& Cmd = GetNextEnabledCmd(State, LastFired);


                if (Cmd == GCmdRef::NullPtr) {
                    if (Deadlocked) {
                        // TODO: Handle deadlocks here
                    }
                    // cout << "No more successors, popping from stack!" << endl;
                    // Done exploring this state
                    DFSStack.pop();
                    continue;
                }

                // Clear out the message buffer before continuing
                State->ClearMsgBuffer();
                
                // Successors remain to be explored
                auto const& Updates = Cmd->GetUpdates();
                auto NextState = State->Clone();

                // cout << "Firing guarded command:" << endl;
                // cout << Cmd->ToString() << endl;

                for (auto const& Update : Updates) {
                    auto const& LHS = Update->GetLHS();
                    auto const& RHS = Update->GetRHS();

                    auto LHSInterp = LHS->ExtensionData.Interp->SAs<LValueInterpreter>();
                    auto RHSInterp = RHS->ExtensionData.Interp;
                    
                    LHSInterp->Update(RHSInterp, State, NextState);
                }
                
                // cout << "Got Next State (Uncanonicalized):" << endl;
                // cout << "--------------------------------------------------------" << endl;
                // Printer->PrintState(NextState, cout);
                // cout << "--------------------------------------------------------" << endl;
                
                u32 PermID;
                auto CanonState = TheCanonicalizer->Canonicalize(NextState, PermID);
                // auto CanonState = NextState;

                auto const& Invar = TheLTS->GetInvariant();
                auto Interp = Invar->ExtensionData.Interp;
                if (Interp->EvaluateScalar(CanonState) != 1) {
                    // error
                }
                
                auto ExistingState = AQS->Find(CanonState);

                if (ExistingState == nullptr) {
                    AQS->Insert(CanonState);
                    // cout << "Pushed new successor onto stack." << endl;
                    AQS->AddEdge(State, CanonState, PermID, LastFired);
                    DFSStack.push(DFSStackEntry(CanonState));
                    continue;
                } else {
                    // Successor already explored, add edge
                    // and continue
                    // cout << "Successor has been previously encountered." << endl;
                    auto InvPermID = PermSet->GetIteratorForInv(PermID).GetIndex();
                    AQS->AddEdge(State, ExistingState, InvPermID, LastFired);
                    CanonState->GetFactory()->TakeState(CanonState);
                    continue;
                }
            }
        }

        void LTSChecker::BuildAQS()
        {
            if (AQS != nullptr) {
                return;
            }
            AQS = new AQStructure();

            auto const& ISGens = TheLTS->GetInitStateGenerators();

            for (auto const& ISGen : ISGens) {
                auto CurState = Factory->MakeState();
                ApplyUpdates(ISGen, ZeroState, CurState);

                // cout << "Initial State:" << endl;
                // cout << "--------------------------------------------------------" << endl;
                // Printer->PrintState(CurState, cout);
                // cout << "--------------------------------------------------------" << endl;

                u32 CanonPerm = 0;
                auto CanonState = TheCanonicalizer->Canonicalize(CurState, CanonPerm);
                
                if (AQS->Find(CanonState) == nullptr) {
                    DoDFS(CanonState);
                } else {
                    CanonState->GetFactory()->TakeState(CanonState);
                }
            }

            cout << "AQS Built!" << endl;
            cout << "AQS contains " << AQS->GetNumStates() << " states and " 
                 << AQS->GetNumEdges() << " edges." << endl;
            cout << Factory->GetNumActiveStates() << " active states from state factory." << endl;
        }

        void LTSChecker::ClearAQS() 
        {
            if (AQS != nullptr) {
                delete AQS;
                AQS = nullptr;
            } 
            if (ThePS != nullptr) {
                delete ThePS;
                ThePS = nullptr;
            }
        }

        BuchiAutomaton* LTSChecker::MakeBuchiMonitor(const string& Name, 
                                                     const vector<ExpT>& SymmIndices, 
                                                     const ExpT &Constraint)
        {
            if (OmegaAutomata.find(Name) != OmegaAutomata.end()) {
                throw ESMCError((string)"Monitor named \"" + Name + "\" already exists " + 
                                "in the LTS Checker");
            }
            auto Retval = new BuchiAutomaton(TheLTS, Name, SymmIndices, Constraint, Compiler);
            OmegaAutomata[Name] = Retval;
            return Retval;
        }

        inline void LTSChecker::ConstructProduct(BuchiAutomaton *Monitor)
        {
            deque<ProductState*> BFSQueue;
            ThePS = new ProductStructure(NumProcesses, NumEnExBits);
            auto MonIndexSet = Monitor->GetIndexSet();
            auto PermSet = TheCanonicalizer->GetPermSet();
            
            for (u32 i = 0; i < MonIndexSet->GetNumIndexVectors(); ++i) {
                for (auto const& MonInitState : Monitor->GetInitialStates()) {
                    for (auto const& AQSInitState : AQS->GetInitStates()) {
                        ThePS->AddInitialState(AQSInitState, MonInitState, i);
                    }
                }
            }
            
            // A BFS over the initial states
            auto const& InitStates = ThePS->GetInitialStates();
            BFSQueue.insert(BFSQueue.begin(), InitStates.begin(), InitStates.end());

            while (BFSQueue.size() > 0) {
                auto CurProdState = BFSQueue.front();
                BFSQueue.pop_front();

                auto SVPtr = CurProdState->GetSVPtr();
                auto MonState = CurProdState->GetMonitorState();
                if (Monitor->IsAccepting(MonState)) {
                    CurProdState->Status.Accepting = true;
                }
                auto IndexID = CurProdState->GetIndexID();
                auto const& AQSEdges = AQS->GetEdges(SVPtr);
                auto&& MonitorNextStates = Monitor->GetNextStates(MonState, IndexID, SVPtr);
                for (auto const& Edge : AQSEdges) {
                    auto NextSVPtr = Edge->GetTarget();
                    auto PermID = Edge->GetPermutation();
                    auto PermIt = PermSet->GetIterator(Edge->GetPermutation());
                    auto NextIndexID = MonIndexSet->Permute(IndexID, PermIt.GetPerm());
                    for (auto const& NextMonState : MonitorNextStates) {
                        bool New;
                        auto NextPS = 
                            ThePS->AddState(NextSVPtr, NextMonState, NextIndexID, New);
                        if (New) {
                            BFSQueue.push_back(NextPS);
                        }
                        ThePS->AddEdge(CurProdState, NextPS, PermID, Edge->GetGCmdIndex());
                    }
                }
            }

            cout << "Product construction complete!" << endl;
            cout << "Product Structure contains " << ThePS->GetNumStates()
                 << " states and " << ThePS->GetNumEdges() << " edges." << endl;
        }

        inline void LTSChecker::CheckSCCs()
        {
            // Find the SCCs on the fly
            stack<pair<const ProductState*, u32>> DFSStack;
            stack<ProductState*> SCCStack;

            u32 CurIndex = 0;
            for (auto InitState : ThePS->GetInitialStates()) {
                InitState->DFSNum = CurIndex;
                InitState->LowLink = CurIndex;
                InitState->Status.OnStack = true;
                DFSStack.push(make_pair(InitState, 0));
                SCCStack.push(InitState);
                ++CurIndex;

                while (DFSStack.size() > 0) {
                    auto const& CurEntry = DFSStack.top();
                    auto CurState = const_cast<ProductState*>(CurEntry.first);
                    auto EdgeToExplore = CurEntry.second;
                    ++DFSStack.top().second;
                    auto const& Edges = ThePS->GetEdges(CurState);

                    if (EdgeToExplore >= Edges.size()) {
                        // We're done with this state
                        DFSStack.pop();
                        // Update my ancestor's lowlink
                        // if I have an ancestor!
                        if (DFSStack.size() > 0) {
                            auto PrevState = DFSStack.top().first;
                            PrevState->LowLink = min(PrevState->LowLink, CurState->LowLink);
                        }
                    } else {
                        // Push successor onto stack
                        // if unexplored, else update lowlink
                        auto it = Edges.begin();
                        for (u32 i = 0; i < EdgeToExplore; ++i) {
                            ++it;
                        }
                        auto CurEdge = *(it);
                        auto NextState = const_cast<ProductState*>(CurEdge->GetTarget());
                        if (NextState->DFSNum == -1) {
                            // unexplored
                            NextState->DFSNum = CurIndex;
                            NextState->LowLink = CurIndex;
                            NextState->Status.OnStack = true;
                            DFSStack.push(make_pair(NextState, 0));
                            SCCStack.push(NextState);
                            ++CurIndex;
                        } else if (NextState->Status.OnStack) {
                            // explored
                            CurState->LowLink = min(CurState->LowLink, NextState->DFSNum);
                        }
                        continue;
                    }

                    // We only get here if we've popped a state 
                    // from the DFS stack.
                    if (CurState->LowLink == CurState->DFSNum) {
                        u32 NumStatesInSCC = 0;
                        ProductState* SCCState = nullptr;
                        do {
                            SCCState = SCCStack.top();
                            SCCState->Status.InSCC = true;
                            SCCStack.pop();
                            ++NumStatesInSCC;
                        } while (SCCState != CurState);

                        if (NumStatesInSCC == 1) {
                            // Singular SCC
                        } else {
                            // TODO: Send for threaded graph resolution
                            // and fairness checks
                            cout << "Found non-singular SCC with " << NumStatesInSCC 
                                 << " states." << endl;
                        }
                    }
                }
            }
        }

        void LTSChecker::CheckLiveness(const string& BuchiMonitorName) 
        {
            auto it = OmegaAutomata.find(BuchiMonitorName);
            if (it == OmegaAutomata.end()) {
                throw ESMCError((string)"Buchi Monitor with name \"" + BuchiMonitorName + 
                                "\" was not found to check liveness property");
            }
            if (AQS == nullptr) {
                throw ESMCError((string)"AQS not built to check liveness property!");
            }
            
            auto Monitor = it->second;
            Monitor->Freeze();
            
            if (ThePS != nullptr) {
                delete ThePS;
                ThePS = nullptr;
            }

            cout << "Constructing Product..." << endl;
            ConstructProduct(Monitor);

            CheckSCCs();
        }

    } /* end namespace MC */
} /* end namespace ESMC */

// 
// LTSChecker.cpp ends here
