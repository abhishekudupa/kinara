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

#include "../uflts/LabelledTS.hpp"
#include "../symmetry/SymmCanonicalizer.hpp"
#include "../uflts/LTSAssign.hpp"
#include "../uflts/LTSTransitions.hpp"

#include "LTSChecker.hpp"
#include "Compiler.hpp"
#include "StateVec.hpp"
#include "StateVecPrinter.hpp"
#include "AQStructure.hpp"
#include "OmegaAutomaton.hpp"

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
            : TheLTS(TheLTS), AQS(nullptr)
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
            vector<DFSStackEntry> DFSStack;
            AQS->InsertInitState(Root);
            DFSStack.push_back(DFSStackEntry(Root));

            while (DFSStack.size() > 0) {
                auto& CurEntry = DFSStack.back();
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
                    DFSStack.pop_back();
                    continue;
                }

                // Clear out the message buffer before continuing
                State->ClearMsgBuffer();
                
                // Successors remain to be explored
                auto const& Updates = Cmd->GetUpdates();
                auto NextState = State->Clone();

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
                // TODO: Check for errors

                auto ExistingState = AQS->Find(CanonState);

                if (ExistingState == nullptr) {
                    AQS->Insert(CanonState);
                    // cout << "Pushed new successor onto stack." << endl;
                    // TODO: ensure that permid is inverted here
                    // or invert it ourselves
                    AQS->AddEdge(State, CanonState, PermID);
                    DFSStack.push_back(DFSStackEntry(CanonState));
                    continue;
                } else {
                    // Successor already explored, add edge
                    // and continue
                    // cout << "Successor has been previously encountered." << endl;
                    AQS->AddEdge(State, ExistingState, PermID);
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

                cout << "Initial State:" << endl;
                cout << "--------------------------------------------------------" << endl;
                Printer->PrintState(CurState, cout);
                cout << "--------------------------------------------------------" << endl;

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
            delete AQS;
            AQS = nullptr;
        }

        BuchiAutomaton* LTSChecker::MakeBuchiMonitor(const string& Name, 
                                                     const vector<ExpT>& SymmIndices, 
                                                     const ExpT &Constraint)
        {
            if (OmegaAutomata.find(Name) != OmegaAutomata.end()) {
                throw ESMCError((string)"Monitor named \"" + Name + "\" already exists " + 
                                "in the LTS Checker");
            }
            auto Retval = new BuchiMonitor(TheLTS, Name, SymmIndices, Constraint,
                                           SymmCanonicalizer->GetPermSet(), Compiler);
            OmegaAutomata[Name] = Retval;
            return Retval;
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
        }

    } /* end namespace MC */
} /* end namespace ESMC */

// 
// LTSChecker.cpp ends here
