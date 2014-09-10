// Trace.cpp --- 
// 
// Filename: Trace.cpp
// Author: Abhishek Udupa
// Created: Tue Sep  9 13:34:49 2014 (-0400)
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

#include "../uflts/LTSTransitions.hpp"
#include "../uflts/LabelledTS.hpp"
#include "../symmetry/SymmCanonicalizer.hpp"

#include "StateVec.hpp"
#include "StateVecPrinter.hpp"
#include "Trace.hpp"
#include "AQStructure.hpp"
#include "LTSChecker.hpp"
#include "StateVecPrinter.hpp"

namespace ESMC {
    namespace MC {

        using LTS::LabelledTS;
        using Symm::Canonicalizer;

        TraceBase::TraceBase(const StateVec* InitialState, StateVecPrinter* Printer)
            : InitialState(InitialState), Printer(Printer)
        {
            // Nothing here
        }

        TraceBase::~TraceBase()
        {
            
        }

        const StateVec* TraceBase::GetInitialState() const
        {
            return InitialState;
        }

        StateVecPrinter* TraceBase::GetPrinter() const
        {
            return Printer;
        }

        inline const StateVec* 
        TraceBase::UnwindPermPath(AQSPermPath* PermPath, 
                                  LTSChecker* Checker,
                                  vector<TraceElemT>& PathElems)
        {
            auto TheLTS = Checker->TheLTS;
            auto TheCanonicalizer = Checker->TheCanonicalizer;

            auto const& GuardedCommands = TheLTS->GetGuardedCmds();
            auto CurUnwoundState = PermPath->GetOrigin()->Clone();
            auto UnwoundOrigin = CurUnwoundState;
            auto const& PPathElems = PermPath->GetPathElems();
            // identity permutation = 0
            u32 InvPermAlongPath = 0;

            auto PermSet = TheCanonicalizer->GetPermSet();
            
            for (auto Edge : PPathElems) {
                auto CurInvPermIt = PermSet->GetIteratorForInv(Edge->GetPermutation());
                auto InvPermAlongPathIt = PermSet->Compose(InvPermAlongPath, 
                                                           CurInvPermIt.GetIndex());
                InvPermAlongPath = InvPermAlongPathIt.GetIndex();
                auto const& Cmd = GuardedCommands[Edge->GetGCmdIndex()];
                auto NextPermState = Edge->GetTarget();

                auto NextUnwoundState = TheCanonicalizer->ApplyPermutation(NextPermState, 
                                                                           InvPermAlongPath);

                // Now find out which command takes us from the current
                // unwound state to the next unwound state
                bool FoundCmd = false;
                for (auto const& Cmd : GuardedCommands) {
                    auto CandidateState = TryExecuteCommand(Cmd, CurUnwoundState);
                    if (CandidateState == nullptr) {
                        continue;
                    }
                    auto SortedCandidateState = TheCanonicalizer->SortChans(CandidateState);
                    CandidateState->Recycle();
                    if (SortedCandidateState->Equals(*NextUnwoundState)) {
                        FoundCmd = true;
                        SortedCandidateState->Recycle();
                        PathElems.push_back(TraceElemT(Cmd, NextUnwoundState));
                        break;
                    }
                }
                if (!FoundCmd) {
                    throw InternalError((string)"Unable to find a command to compute next " +
                                        "unwound state.\nAt: " + __FILE__ + ":" + 
                                        to_string(__LINE__));
                }

                // Do not recycle the unwound state
                // it's part of the return value
                CurUnwoundState = NextUnwoundState;
            }
            return UnwoundOrigin;
        }
        
        SafetyViolation* TraceBase::MakeSafetyViolation(const StateVec* ErrorState, 
                                                        LTSChecker* Checker)
        {
            vector<TraceElemT> PathElems;
            auto TheAQS = Checker->AQS;
            auto PPath = TheAQS->FindShortestPath(ErrorState);
            auto UnwoundInitState = UnwindPermPath(PPath, Checker, PathElems);
            return new SafetyViolation(UnwoundInitState, PathElems, Checker->Printer);
        }

        DeadlockViolation* TraceBase::MakeDeadlockViolation(const StateVec* ErrorState, 
                                                            LTSChecker* Checker)
        {
            vector<TraceElemT> PathElems;
            auto TheAQS = Checker->AQS;
            auto PPath = TheAQS->FindShortestPath(ErrorState);
            auto UnwoundInitState = UnwindPermPath(PPath, Checker, PathElems);
            return new DeadlockViolation(UnwoundInitState, PathElems, Checker->Printer);
        }


        // Takes ownership of trace elems
        // and initial state
        SafetyViolation::SafetyViolation(const StateVec* InitialState,
                                         const vector<TraceElemT>& TraceElems,
                                         StateVecPrinter* Printer)
            : TraceBase(InitialState, Printer), TraceElems(TraceElems)
        {
            // Nothing here
        }

        SafetyViolation::~SafetyViolation()
        {
            InitialState->Recycle();
            for (auto const& TraceElem : TraceElems) {
                TraceElem.second->Recycle();
            }
        }

        const vector<TraceElemT>& SafetyViolation::GetTraceElems() const
        {
            return TraceElems;
        }

        string SafetyViolation::ToString(u32 Verbosity) const
        {
            ostringstream sstr;
            sstr << "Trace to safety violation with " << TraceElems.size() 
                 << " steps:" << endl << endl;
            sstr << "Initial State (in full)" << endl;
            sstr << "-----------------------------------------------------" << endl;
            Printer->PrintState(InitialState, sstr);
            sstr << "-----------------------------------------------------" << endl << endl;

            auto PrevState = InitialState;
            for (auto const& TraceElem : TraceElems) {
                auto const& MsgType = TraceElem.first->GetMsgType();
                auto MsgTypeAsRec = MsgType->SAs<Exprs::ExprRecordType>();
                if (Verbosity < 1) {
                    sstr << "Fired Guarded Command with label: " 
                         << (MsgTypeAsRec != nullptr ? MsgTypeAsRec->GetName() : 
                             "(internal transition)") << endl;
                } else {
                    sstr << "Fired Guarded Command:" << endl;
                    sstr << TraceElem.first->ToString() << endl;
                }
                sstr << "Obtained next state (delta from previous state):" << endl;
                sstr << "-----------------------------------------------------" << endl;
                Printer->PrintState(TraceElem.second, PrevState, sstr);
                sstr << "-----------------------------------------------------" << endl << endl;
                PrevState = TraceElem.second;
            }

            sstr << "Last state of trace (in full):" << endl;
            sstr << "-----------------------------------------------------" << endl;
            Printer->PrintState(PrevState, sstr);
            sstr << "-----------------------------------------------------" << endl << endl;
            return sstr.str();
        }

        DeadlockViolation::~DeadlockViolation()
        {
            // Nothing here
        }

    } /* end namespace MC */
} /* end namespace ESMC */

// 
// Trace.cpp ends here
