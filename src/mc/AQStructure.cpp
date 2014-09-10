// AQStructure.cpp --- 
// 
// Filename: AQStructure.cpp
// Author: Abhishek Udupa
// Created: Tue Aug 19 11:51:55 2014 (-0400)
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

#include <boost/heap/fibonacci_heap.hpp>

#include "../uflts/LabelledTS.hpp"
#include "../uflts/LTSTransitions.hpp"

#include "AQStructure.hpp"

namespace ESMC {
    namespace MC {

        AQStructure::AQStructure(LabelledTS* TheLTS)
            : EdgePool(new boost::pool<>(sizeof(AQSEdge))),
              TheLTS(TheLTS)
        {
            // Nothing here
        }

        AQStructure::~AQStructure()
        {
            delete EdgePool;
        }

        LabelledTS* AQStructure::GetLTS() const
        {
            return TheLTS;
        }

        void AQStructure::AddErrorState(const StateVec* ErrorState)
        {
            if (StateHashSet.find(const_cast<StateVec*>(ErrorState)) == StateHashSet.end()) {
                throw ESMCError((string)"State not in AQS in call to " + 
                                "AQStructure::AddErrorState()");
            }
            ErrorStates.insert(ErrorState);
        }

        void AQStructure::AddDeadlockState(const StateVec* DeadlockState)
        {
            if (StateHashSet.find(const_cast<StateVec*>(DeadlockState)) == StateHashSet.end()) {
                throw ESMCError((string)"State not in AQS in call to " + 
                                "AQStructure::AddDeadlockState()");
            }
            DeadlockStates.insert(DeadlockState);
        }

        StateVec* AQStructure::Find(StateVec* SV) const
        {
            auto it = StateHashSet.find(SV);
            if (it == StateHashSet.end()) {
                return nullptr;
            } else {
                return it->first;
            }
        }

        const set<const StateVec*>& AQStructure::GetErrorStates() const
        {
            return ErrorStates;
        }

        const set<const StateVec*>& AQStructure::GetDeadlockStates() const
        {
            return DeadlockStates;
        }

        void AQStructure::Insert(StateVec* SV)
        {
            if (Find(SV)) {
                return;
            }
            StateHashSet[SV] = AQSEdgeSetT();
        }

        void AQStructure::InsertInitState(StateVec* SV)
        {
            if (Find(SV)) {
                return;
            }
            StateHashSet[SV] = AQSEdgeSetT();
            InitStates.push_back(SV);
        }

        void AQStructure::AddEdge(StateVec* Source, StateVec* Target,
                                  u32 Permutation, u32 GCmdIndex)
        {
            auto SourceIt = StateHashSet.find(Source);
            auto DestIt = StateHashSet.find(Target);

            if (SourceIt == StateHashSet.end() || 
                DestIt == StateHashSet.end()) {
                throw InternalError((string)"Attempted to add edge with one " + 
                                    "or more states not known to the AQS.\n" + 
                                    "At: " + __FILE__ + ":" + to_string(__LINE__));
            }

            auto NewEdge = 
                new (EdgePool->malloc()) AQSEdge(Target, Permutation, GCmdIndex);
            auto EdgeIt = EdgeHashSet.find(NewEdge);
            if (EdgeIt != EdgeHashSet.end()) {
                EdgePool->free(NewEdge);
                NewEdge = *EdgeIt;
            } else {
                EdgeHashSet.insert(NewEdge);
            }

            SourceIt->second.insert(NewEdge);
        }

        u64 AQStructure::GetNumStates() const
        {
            return StateHashSet.size();
        }

        u64 AQStructure::GetNumEdges() const
        {
            return EdgeHashSet.size();
        }

        const vector<StateVec*>& AQStructure::GetInitStates() const
        {
            return InitStates;
        }

        const AQSEdgeSetT& AQStructure::GetEdges(const StateVec* SV) const
        {
            auto it = StateHashSet.find(const_cast<StateVec*>(SV));
            if (it == StateHashSet.end()) {
                return EmptyEdgeSet;
            } else {
                return it->second;
            }
        }

        // Path methods for AQS
        AQSPermPath* 
        AQStructure::FindPath(const set<const StateVec*>& Origins, 
                              const StateVec* Target) const
        {
            for (auto const& Origin : Origins) {
                if (StateHashSet.find(const_cast<StateVec*>(Origin)) == StateHashSet.end()) {
                    throw ESMCError((string)"Origin state not in AQS in " + 
                                    "call to AQStructure::FindPath()");
                }
            }

            if (StateHashSet.find(const_cast<StateVec*>(Target)) == StateHashSet.end()) {
                throw ESMCError((string)"Target state not in AQS in " +
                                "call to AQStructure::FindPath()");
            }
            
            return FindPath(Origins, 
                            [=] (const StateVec* SV) -> bool 
                            { return (SV == Target); });
        }

        AQSPermPath*
        AQStructure::FindPath(const set<const StateVec*>& Origins, 
                              const function<bool(const StateVec* Target)>& TargetPred) const
        {
            for (auto const& Origin : Origins) {
                if (StateHashSet.find(const_cast<StateVec*>(Origin)) == StateHashSet.end()) {
                    throw ESMCError((string)"Origin not in AQS in " + 
                                    "call to AQStructure::FindPath()");
                }
            }

            return FindPath(Origins, 
                            [=] (const AQSEdgeSetT& Edges) -> const StateVec*
                            { 
                                for (auto Edge : Edges) {
                                    if (TargetPred(Edge->GetTarget())) {
                                        return Edge->GetTarget();
                                    }
                                }
                                return nullptr;
                            });
        }

        AQSPermPath*
        AQStructure::FindPath(const set<const StateVec*>& Origins,
                              const function<const StateVec*(const AQSEdgeSetT&)>&
                              TargetEdgePred) const
        {
            // do a bfs
            deque<const StateVec*> BFSQueue;
            unordered_set<const StateVec*> VisitedStates;
            unordered_map<const StateVec*, const StateVec*> PathPreds;
            
            for (auto Origin : Origins) {
                if (StateHashSet.find(const_cast<StateVec*>(Origin)) == StateHashSet.end()) {
                    throw ESMCError((string)"Origin State not in AQS in call " + 
                                    "to AQStructure::FindPath()");
                }
                VisitedStates.insert(Origin);
                BFSQueue.push_back(Origin);
            }

            const StateVec* ActualTarget;
            bool Found = false;
            while (BFSQueue.size() > 0) {
                auto CurState = BFSQueue.front();
                BFSQueue.pop_front();

                auto const& Edges = 
                    StateHashSet.find(const_cast<StateVec*>(CurState))->second;
                auto TargetState = TargetEdgePred(Edges);

                if (TargetState != nullptr) {
                    ActualTarget = TargetState;
                    PathPreds[TargetState] = CurState;
                    Found = true;
                    break;
                } else {
                    for (auto Edge : Edges) {
                        auto CurTarget = Edge->GetTarget();
                        if (VisitedStates.find(CurTarget) == VisitedStates.end()) {
                            PathPreds[CurTarget] = CurState;
                            BFSQueue.push_back(CurTarget);
                        }
                    }
                }
            }

            if (Found) {
                deque<AQSPermPath::PathElemType> ThePath;
                auto CurTarget = ActualTarget;

                while (Origins.find(const_cast<StateVec*>(CurTarget)) != Origins.end()) {
                    auto Predecessor = PathPreds[CurTarget];
                    auto const& PredEdges = 
                        StateHashSet.find(const_cast<StateVec*>(Predecessor))->second;
                    for (auto Edge : PredEdges) {
                        if (Edge->GetTarget() == CurTarget) {
                            ThePath.push_front(Edge);
                            break;
                        }
                    }
                    CurTarget = Predecessor;
                }
                vector<AQSPermPath::PathElemType> PathVec(ThePath.begin(), ThePath.end());
                auto Retval = new AQSPermPath(CurTarget, PathVec);
                return Retval;
            } else {
                return nullptr;
            }
        }

        AQSPermPath*
        AQStructure::FindPath(const StateVec* Target) const
        {
            return FindPath(set<const StateVec*>(InitStates.begin(), InitStates.end()), Target);
        }

        AQSPermPath* 
        AQStructure::FindPath(const function<bool (const StateVec *)> &TargetPred) const
        {
            return FindPath(set<const StateVec*>(InitStates.begin(), InitStates.end()), 
                            TargetPred);
        }

        AQSPermPath*
        AQStructure::FindPath(const function<const StateVec* (const AQSEdgeSetT&)>&
                              TargetEdgePred) const
        {
            return FindPath(set<const StateVec*>(InitStates.begin(), InitStates.end()),
                            TargetEdgePred);
        }

        AQSPermPath*
        AQStructure::FindShortestPath(const set<const StateVec*>& Origins, 
                                      const StateVec* Target) const
        {
            // No cost function given ==> unit cost for each edge ==> BFS will do :-)
            return FindPath(Origins, Target);
        }

        AQSPermPath*
        AQStructure::FindShortestPath(const set<const StateVec*>& Origins, 
                                      const function<bool(const ESMC::MC::StateVec*)>& 
                                      TargetPred) const
        {
            // Again, no cost function ==> BFS
            return FindPath(Origins, TargetPred);
        }


        AQSPermPath* 
        AQStructure::FindShortestPath(const set<const StateVec*>& Origins,
                                      const function<const StateVec*(const AQSEdgeSetT&)>&
                                      TargetEdgePred) const
        {
            return FindPath(Origins, TargetEdgePred);
        }

        AQSPermPath* 
        AQStructure::FindShortestPath(const set<const StateVec*>& Origins,
                                      const StateVec* Target,
                                      const function<u32(const StateVec*, const AQSEdge*)>&
                                      CostFunction) const
        {
            for (auto Origin : Origins) {
                if (StateHashSet.find(const_cast<StateVec*>(Origin)) == StateHashSet.end() ||
                    StateHashSet.find(const_cast<StateVec*>(Target)) == StateHashSet.end()) {
                    throw ESMCError((string)"Origin state not in AQS in " + 
                                    "call to AQStructure::FindShortestPath()");
                }
            }
            if (StateHashSet.find(const_cast<StateVec*>(Target)) == StateHashSet.end()) {
                    throw ESMCError((string)"Target state not in AQS in " + 
                                    "call to AQStructure::FindShortestPath()");                
            }
            
            return FindShortestPath(Origins, 
                                    [=] (const StateVec* SV) -> bool 
                                    { return (SV == Target); });
        }

        AQSPermPath* 
        AQStructure::FindShortestPath(const set<const StateVec*>& Origins,
                                      const function<bool(const StateVec*)>& TargetPred,
                                      const function<u32(const StateVec*, const AQSEdge*)>&
                                      CostFunction) const
        {
            for (auto Origin : Origins) {
                if (StateHashSet.find(const_cast<StateVec*>(Origin)) == StateHashSet.end()) {
                    throw ESMCError((string)"Origin not in AQS in " + 
                                    "call to AQStructure::FindShortestPath()");
                }
            }

            return FindShortestPath(Origins,
                                    [=] (const AQSEdgeSetT& Edges) -> const StateVec*
                                    {
                                        for (auto Edge : Edges) {
                                            if (TargetPred(Edge->GetTarget())) {
                                                return Edge->GetTarget();
                                            }
                                        }
                                        return nullptr;
                                    });
        }

        AQSPermPath* 
        AQStructure::FindShortestPath(const set<const StateVec*>& Origins,
                                      const function<const StateVec*(const AQSEdgeSetT&)>& 
                                      TargetEdgePred,
                                      const function<u32(const StateVec*, const AQSEdge*)>&
                                      CostFunction) const
        {
            using boost::heap::fibonacci_heap;
            using Detail::AQSFibDataT;
            using Detail::AQSFibDataCompare;
            
            typedef boost::heap::compare<AQSFibDataCompare> FibHeapCompareT;
            typedef fibonacci_heap<AQSFibDataT, FibHeapCompareT> FibHeapT;

            FibHeapT PrioQ;
            unordered_map<const StateVec*, FibHeapT::handle_type> StateToHandle;
            unordered_map<const StateVec*, const StateVec*> Predecessors;

            const StateVec* ActualTarget = nullptr;

            for (auto Origin : Origins) {
                if (StateHashSet.find(const_cast<StateVec*>(Origin)) == StateHashSet.end()) {
                    throw ESMCError((string)"Origin not in AQS in call to " + 
                                    "AQStructure::FindShortestPath()");
                }
                auto Handle = PrioQ.push(AQSFibDataT(Origin, 0));
                StateToHandle[Origin] = Handle;
            }
            
            bool ReachedTarget = false;
            while (PrioQ.size() > 0 && !ReachedTarget) {
                auto CurStateData = PrioQ.top();
                auto CurState = CurStateData.StateVector;
                PrioQ.pop();
                
                auto const& Edges = 
                    StateHashSet.find(const_cast<StateVec*>(CurState))->second;

                ActualTarget = TargetEdgePred(Edges);
 
                for (auto Edge : Edges) {
                    u32 NewDist = CurStateData.DistanceFromOrigin + 
                        CostFunction(CurState, Edge);
                    auto NSVec = Edge->GetTarget();

                    auto nsit = StateToHandle.find(NSVec);
                    u32 OldDist;
                    FibHeapT::handle_type NextStateHandle;

                    if (nsit == StateToHandle.end()) {
                        OldDist = UINT32_MAX;
                    } else {
                        NextStateHandle = nsit->second;
                        OldDist = (*NextStateHandle).DistanceFromOrigin;
                    }
                    
                    if (NewDist < OldDist) {
                        if (OldDist != UINT32_MAX) {
                            // State was already present in prio queue
                            // just decrease the distance
                            (*NextStateHandle).DistanceFromOrigin = NewDist;
                            PrioQ.decrease(NextStateHandle);
                        } else {
                            // State wasn't present in the prio queue
                            // insert it
                            auto Handle = PrioQ.push(AQSFibDataT(NSVec, NewDist));
                            StateToHandle[NSVec] = Handle;
                        }
                        // Mark predecessor
                        Predecessors[NSVec] = CurState;
                    }
                    
                    // Did we reach the desired target?
                    if (Edge->GetTarget() == ActualTarget) {
                        ReachedTarget = true;
                        break;
                    }
                }
            }

            if (!ReachedTarget) {
                // WTF? The target is not reachable?
                // Too bad then I guess.
                return nullptr;
            }

            // assemble the path
            deque<AQSPermPath::PathElemType> ThePath;
            auto CurTarget = ActualTarget;
            while (Origins.find(const_cast<StateVec*>(CurTarget)) == Origins.end()) {
                auto Predecessor = Predecessors[CurTarget];
                auto const& PredEdges = 
                    StateHashSet.find(const_cast<StateVec*>(Predecessor))->second;
                for (auto Edge : PredEdges) {
                    if (Edge->GetTarget() == CurTarget) {
                        ThePath.push_front(Edge);
                        break;
                    }
                }
                CurTarget = Predecessor;
            }
            vector<AQSPermPath::PathElemType> PathVec(ThePath.begin(), ThePath.end());
            auto Retval = new AQSPermPath(CurTarget, PathVec);
            return Retval;
        }

        AQSPermPath* 
        AQStructure::FindShortestPath(const ESMC::MC::StateVec *Target) const
        {
            return FindShortestPath(set<const StateVec*>(InitStates.begin(), InitStates.end()), 
                                    Target);
        }

        AQSPermPath* 
        AQStructure::FindShortestPath(const function<const StateVec*(const AQSEdgeSetT&)>&
                                      TargetEdgePred) const
        {
            return FindShortestPath(set<const StateVec*>(InitStates.begin(), InitStates.end()),
                                    TargetEdgePred);
        }

        AQSPermPath* 
        AQStructure::FindShortestPath(const function<bool (const StateVec*)> &TargetPred) const
        {
            return FindShortestPath(set<const StateVec*>(InitStates.begin(), InitStates.end()), 
                                    TargetPred);
        }

        AQSPermPath*
        AQStructure::FindShortestPath(const StateVec* Target,
                                      const function<u32(const StateVec*, const AQSEdge*)>&
                                      CostFunction) const
        {
            return FindShortestPath(set<const StateVec*>(InitStates.begin(), InitStates.end()),
                                    Target, CostFunction);
        }

        AQSPermPath*
        AQStructure::FindShortestPath(const function<bool(const StateVec*)>& TargetPred,
                                      const function<u32(const StateVec*, const AQSEdge*)>&
                                      CostFunction) const
        {
            return FindShortestPath(set<const StateVec*>(InitStates.begin(), InitStates.end()),
                                    TargetPred, CostFunction);
        }

        AQSPermPath*
        AQStructure::FindShortestPath(const function<const StateVec*(const AQSEdgeSetT&)>& 
                                      TargetEdgePred,
                                      const function<u32(const StateVec*, const AQSEdge*)>&
                                      CostFunction) const
        {
            return FindShortestPath(set<const StateVec*>(InitStates.begin(), InitStates.end()),
                                    TargetEdgePred, CostFunction);
        }

        ProductState::ProductState(const StateVec* SVPtr, u32 MonitorState,
                                   u32 IndexID, u32 NumProcesses)
            : SVPtr(SVPtr), MonitorState(MonitorState), IndexID(IndexID),
              Status(), DFSNum(-1), LowLink(-1),
              TrackingBits(NumProcesses, false)
        {
            // Nothing here
        }

        ProductState::~ProductState()
        {
            TrackingBits.clear();
        }

        const StateVec* ProductState::GetSVPtr() const
        {
            return SVPtr;
        }

        u32 ProductState::GetMonitorState() const
        {
            return MonitorState;
        }

        u32 ProductState::GetIndexID() const
        {
            return IndexID;
        }

        u64 ProductState::Hash() const
        {
            u64 Retval = 0;
            boost::hash_combine(Retval, SVPtr);
            boost::hash_combine(Retval, ((u64)MonitorState << 32 | (u64)IndexID));
            return Retval;
        }

        bool ProductState::operator == (const ProductState& Other) const
        {
            return (SVPtr == Other.SVPtr &&
                    MonitorState == Other.MonitorState &&
                    IndexID == Other.IndexID);
        }

        void ProductState::ClearMarkings() const
        {
            Status.InSCC = -1;
            Status.OnStack = false;
            Status.ThreadedVisted = false;
            Status.Accepting = false;
            Status.Deleted = false;
            DFSNum = -1;
            LowLink = -1;

            for (u32 i = 0; i < TrackingBits.size(); ++i) {
                TrackingBits[i] = false;
            }
        }

        void ProductState::MarkInSCC(u32 SCCID) const
        {
            Status.InSCC = SCCID;
        }

        void ProductState::MarkNotInSCC() const
        {
            Status.InSCC = -1;
        }

        bool ProductState::IsInSCC(u32 SCCID) const
        {
            return ((Status.InSCC != -1) && 
                    ((u32)Status.InSCC == SCCID));
        }

        void ProductState::MarkOnStack() const
        {
            Status.OnStack = true;
        }

        void ProductState::MarkNotOnStack() const
        {
            Status.OnStack = false;
        }

        bool ProductState::IsOnStack() const
        {
            return Status.OnStack;
        }

        void ProductState::MarkThreadedVisited() const
        {
            Status.ThreadedVisted = true;
        }

        void ProductState::MarkNotThreadedVisited() const
        {
            Status.ThreadedVisted = false;
        }

        bool ProductState::IsThreadedVisited() const
        {
            return Status.ThreadedVisted;
        }

        void ProductState::MarkAccepting() const
        {
            Status.Accepting = true;
        }

        void ProductState::MarkNotAccepting() const
        {
            Status.Accepting = false;
        }

        bool ProductState::IsAccepting() const
        {
            return Status.Accepting;
        }

        void ProductState::MarkDeleted() const
        {
            Status.Deleted = true;
        }

        void ProductState::MarkNotDeleted() const
        {
            Status.Deleted = false;
        }

        bool ProductState::IsDeleted() const
        {
            return Status.Deleted;
        }

        void ProductState::MarkTracked(u32 BitNum) const
        {
            TrackingBits[BitNum] = true;
        }

        void ProductState::MarkNotTracked(u32 BitNum) const
        {
            TrackingBits[BitNum] = false;
        }

        bool ProductState::IsTracked(u32 BitNum) const
        {
            return TrackingBits[BitNum];
        }

        void ProductState::ClearAllTracked() const
        {
            for (u32 i = 0; i < TrackingBits.size(); ++i) {
                TrackingBits[i] = false;
            }
        }

        ProductStructure::ProductStructure(u32 NumProcesses)
            : NumProcesses(NumProcesses),
              PSPool(new boost::object_pool<ProductState>()),
              PEPool(new boost::pool<>(sizeof(ProductEdge)))
        {
            // Nothing here
        }

        ProductStructure::~ProductStructure()
        {
            delete PSPool;
            delete PEPool;
        }

        ProductState* ProductStructure::AddInitialState(const StateVec *SVPtr, 
                                                        u32 MonitorState, 
                                                        u32 IndexID)
        {
            bool New;
            auto NewPS = AddState(SVPtr, MonitorState, IndexID, New);
            if (New) {
                InitialStates.push_back(NewPS);
            }
            return NewPS;
        }

        ProductState* ProductStructure::AddState(const StateVec* SVPtr,
                                                 u32 MonitorState,
                                                 u32 IndexID, bool& New)
        {
            auto NewPS = 
                new (PSPool->malloc()) 
                ProductState(SVPtr, MonitorState, IndexID, NumProcesses);

            auto it = PSHashSet.find(NewPS);
            if (it == PSHashSet.end()) {
                PSHashSet[NewPS] = EmptyEdgeSet;
                New = true;
                return NewPS;
            } else {
                PSPool->destroy(NewPS);
                NewPS = it->first;
                New = false;
                return NewPS;
            }
        }

        void ProductStructure::AddEdge(ProductState* Source, ProductState* Target,
                                       u32 Permutation, u32 GCmdIndex)
        {
            auto SourceIt = PSHashSet.find(Source);
            auto TargetIt = PSHashSet.find(Target);
            
            if (SourceIt == PSHashSet.end() || TargetIt == PSHashSet.end()) {
                throw InternalError((string)"Source and/or target state not known " + 
                                    "to ProductStructure while adding edge.\nAt: " + 
                                    __FILE__ + ":" + to_string(__LINE__));
            }
            auto NewEdge = 
                new (PEPool->malloc()) ProductEdge(Target, Permutation, GCmdIndex);
            auto EdgeIt = EdgeHashSet.find(NewEdge);
            if (EdgeIt == EdgeHashSet.end()) {
                EdgeHashSet.insert(NewEdge);
            } else {
                PEPool->free(NewEdge);
                NewEdge = *EdgeIt;
            }

            SourceIt->second.insert(NewEdge);
        }

        const vector<ProductState*>& ProductStructure::GetInitialStates() const
        {
            return InitialStates;
        }

        const ProductEdgeSetT& ProductStructure::GetEdges(ProductState* State) const
        {
            auto it = PSHashSet.find(State);
            if (it == PSHashSet.end()) {
                return EmptyEdgeSet;
            } else {
                return it->second;
            }
        }

        u32 ProductStructure::GetNumEdges() const
        {
            return EdgeHashSet.size();
        }

        u32 ProductStructure::GetNumStates() const
        {
            return PSHashSet.size();
        }

        void ProductStructure::ClearAllMarkings() const
        {
            for (auto const& PS : PSHashSet) {
                PS.first->ClearMarkings();
            }
        }

        u32 ProductStructure::GetNumProcesses() const
        {
            return NumProcesses;
        }

        void 
        ProductStructure::ApplyToAllStates(const function<void (const ESMC::MC::ProductState *)>& 
                                           Func) const
        {
            for (auto const& PS : PSHashSet) {
                Func(PS.first);
            }
        }

    } /* end namespace MC */
} /* end namespace ESMC */

// 
// AQStructure.cpp ends here
