// AQStructure.hpp --- 
// 
// Filename: AQStructure.hpp
// Author: Abhishek Udupa
// Created: Tue Aug 19 11:30:01 2014 (-0400)
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

#if !defined ESMC_AQ_STRUCTURE_HPP_
#define ESMC_AQ_STRUCTURE_HPP_

#include <new>
#include <sparse_hash_set>
#include <sparse_hash_map>
#include <set>

#include <boost/pool/pool.hpp>
#include <boost/pool/object_pool.hpp>
#include <boost/functional/hash.hpp>

#include "../common/FwdDecls.hpp"

#include "StateVec.hpp"
#include "Trace.hpp"

namespace ESMC {
    namespace MC {

        using google::sparse_hash_set;
        using google::sparse_hash_map;
        using LTS::LabelledTS;
        using LTS::GCmdRef;

        namespace Detail {

            class StateVecPtrHasher
            {
            public:
                inline u64 operator () (const StateVec* Ptr) const
                {
                    return Ptr->Hash();
                }
            };

            class StateVecPtrEquals
            {
            public:
                inline bool operator () (const StateVec* Ptr1,
                                         const StateVec* Ptr2) const
                {
                    return Ptr1->Equals(*Ptr2);
                }
            };

        } /* end namespace Detail */

        
        template <typename STATETYPE>
        class AnnotatedEdge
        {
        private:
            const STATETYPE* Target;
            u32 Permutation;
            u32 GCmdIndex;

        public:
            inline AnnotatedEdge()
                : Target(nullptr), Permutation(0), GCmdIndex(0)
            {
                // Nothing here
            }

            inline AnnotatedEdge(const STATETYPE* Target, u32 Permutation,
                                 u32 GCmdIndex)
                : Target(Target), Permutation(Permutation),
                  GCmdIndex(GCmdIndex)
            {
                // Nothing here
            }

            inline AnnotatedEdge(const AnnotatedEdge& Other)
                : Target(Other.Target), Permutation(Other.Permutation),
                  GCmdIndex(Other.GCmdIndex)
            {
                // Nothing here
            }

            inline ~AnnotatedEdge()
            {
                // Nothing here
            }
            
            inline AnnotatedEdge& operator = (const AnnotatedEdge& Other)
            {
                if (&Other == this) {
                    return *this;
                }
                Target = Other.Target;
                Permutation = Other.Permutation;
                GCmdIndex = Other.GCmdIndex;
                return *this;
            }

            inline bool operator == (const AnnotatedEdge& Other) const
            {
                return (Target == Other.Target && 
                        Permutation == Other.Permutation &&
                        GCmdIndex == Other.GCmdIndex);
            }

            inline const STATETYPE* GetTarget() const
            {
                return Target;
            }

            inline u32 GetPermutation() const
            {
                return Permutation;
            }

            inline u32 GetGCmdIndex() const
            {
                return GCmdIndex;
            }

            inline u64 Hash() const
            {
                u64 Retval = 0;
                boost::hash_combine(Retval, Target);
                boost::hash_combine(Retval, (((u64)Permutation << 32) | (u64)GCmdIndex));
                return Retval;
            }
        };


        namespace Detail {
            
            template <typename STATETYPE>
            class AnnotatedEdgeHash
            {
            public:
                inline u64 operator () (const AnnotatedEdge<STATETYPE>& Edge) const
                {
                    return Edge.Hash();
                }
            };

            template <typename STATETYPE>
            class AnnotatedEdgePtrHasher
            {
            public:
                inline u64 operator () (const AnnotatedEdge<STATETYPE>* Edge) const
                {
                    return Edge->Hash();
                }
            };

            template <typename STATETYPE>
            class AnnotatedEdgePtrEquals
            {
            public:
                inline bool operator () (const AnnotatedEdge<STATETYPE>* Ptr1, 
                                         const AnnotatedEdge<STATETYPE>* Ptr2) const
                {
                    return (*Ptr1 == *Ptr2);
                }
            };

            typedef AnnotatedEdgePtrEquals<StateVec> AQSEdgePtrEquals;
            typedef AnnotatedEdgePtrHasher<StateVec> AQSEdgePtrHasher;

            // Fibonacci heap data structure
            struct AQSFibDataT
            {
                const StateVec* StateVector;
                u32 DistanceFromOrigin;

                inline AQSFibDataT(const StateVec* SV, u32 DistanceFromOrigin)
                    : StateVector(SV), DistanceFromOrigin(DistanceFromOrigin)
                {
                    // Nothing here
                }
            };

            struct AQSFibDataCompare
            {
                // Use a > comparator, since boost fib heaps are max-heaps
                // rather than the text-book min-heaps.
                inline bool operator () (const AQSFibDataT& Data1, 
                                         const AQSFibDataT& Data2) const
                {
                    return (Data1.DistanceFromOrigin > Data2.DistanceFromOrigin);
                }
            };

        } /* end namespace Detail */

        typedef AnnotatedEdge<StateVec> AQSEdge;
        typedef sparse_hash_set<AQSEdge*, 
                                Detail::AQSEdgePtrHasher,
                                Detail::AQSEdgePtrEquals> AQSEdgeHashSetT;

        typedef sparse_hash_set<AQSEdge*> AQSEdgeSetT;

        typedef sparse_hash_map<StateVec*, AQSEdgeSetT,
                                Detail::StateVecPtrHasher,
                                Detail::StateVecPtrEquals> SVHashSetT;

        class AQStructure
        {
        private:
            SVHashSetT StateHashSet;
            AQSEdgeHashSetT EdgeHashSet;
            vector<StateVec*> InitStates;
            AQSEdgeSetT EmptyEdgeSet;
            boost::pool<>* EdgePool;
            LabelledTS* TheLTS;

            // An error state (if any)
            const StateVec* ErrorState;
            u32 ErrorDepth;
            // A deadlock state (if any)
            const StateVec* DeadlockState;
            u32 DeadlockDepth;

        public:
            AQStructure(LabelledTS* TheLTS);
            virtual ~AQStructure();

            StateVec* Find(StateVec* SV) const;
            void Insert(StateVec* SV);
            void InsertInitState(StateVec* SV);
            void AddEdge(StateVec* Source, StateVec* Target, 
                         u32 Permutation, u32 GCmdIndex);
            u64 GetNumStates() const;
            u64 GetNumEdges() const;
            const AQSEdgeSetT& GetEdges(const StateVec* SV) const;
            const vector<StateVec*>& GetInitStates() const;

            LabelledTS* GetLTS() const;

            // Tracking for error states
            void AddErrorState(const StateVec* ErrorState, u32 Depth);
            void AddDeadlockState(const StateVec* DeadlockState, u32 Depth);

            const StateVec* GetErrorState() const;
            const StateVec* GetDeadlockState() const;

            // Path finding methods
            AQSPermPath* FindPath(const set<const StateVec*>& Origins, 
                                  const StateVec* Target) const;
            AQSPermPath* FindPath(const set<const StateVec*>& Origins, 
                                  const function<bool(const StateVec*)>& TargetPred) const;
            AQSPermPath* FindPath(const set<const StateVec*>& Origins,
                                  const function<const StateVec*(const AQSEdgeSetT&)>&
                                  TargetEdgePred) const;

            // Find a path from one of the initial states
            AQSPermPath* FindPath(const StateVec* Target) const;
            AQSPermPath* FindPath(const function<bool(const StateVec*)>& TargetPred) const;
            AQSPermPath* FindPath(const function<const StateVec*(const AQSEdgeSetT&)>&
                                  TargetEdgePred) const;

            // Shortest path without cost functions
            // is the same as a BFS path, so we set up the forwards
            template <typename... ArgTypes>
            inline AQSPermPath* FindShortestPath(ArgTypes&&... Args)
            {
                return FindPath(forward<ArgTypes>(Args)...);
            }


            // Shortest paths with cost functions
            AQSPermPath* FindShortestPath(const set<const StateVec*>& Origins,
                                          const StateVec* Target,
                                          const function<u32(const StateVec*, const AQSEdge*)>&
                                          CostFunction) const;

            AQSPermPath* FindShortestPath(const set<const StateVec*>& Origins,
                                          const function<bool(const StateVec*)>& TargetPred,
                                          const function<u32(const StateVec*, const AQSEdge*)>&
                                          CostFunction) const;

            AQSPermPath* FindShortestPath(const set<const StateVec*>& Origins,
                                          const function<const StateVec*(const AQSEdgeSetT&)>& 
                                          TargetEdgePred,
                                          const function<u32(const StateVec*, const AQSEdge*)>&
                                          CostFunction) const;

            // Find a shortest path from one of the initial states
            AQSPermPath* FindShortestPath(const StateVec* Target,
                                          const function<u32(const StateVec*, const AQSEdge*)>&
                                          CostFunction) const;
            AQSPermPath* FindShortestPath(const function<bool(const StateVec*)>& TargetPred,
                                          const function<u32(const StateVec*, const AQSEdge*)>&
                                          CostFunction) const;
            AQSPermPath* FindShortestPath(const function<const StateVec*(const AQSEdgeSetT&)>& 
                                          TargetEdgePred,
                                          const function<u32(const StateVec*, const AQSEdge*)>&
                                          CostFunction) const;
        };

        // A bitfield structure for the status bits 
        // in the product state
        struct ThreadedGraphStatusT
        {
            bool OnStack : 1;
            bool ThreadedVisted : 1;
            bool Accepting : 1;
            bool Deleted : 1;
            i32 InSCC : 28;

            inline ThreadedGraphStatusT()
                : OnStack(false), ThreadedVisted(false), Accepting(false),
                  Deleted(false), InSCC(-1)
            {
                // Nothing here
            }
        };

        class ProductState
        {
        private:
            const StateVec* SVPtr;
            u32 MonitorState;
            u32 IndexID;

        public:
            // Status variables that do not affect the 
            // identity of this node
            mutable ThreadedGraphStatusT Status;

            mutable i32 DFSNum;
            mutable i32 LowLink;
            mutable vector<bool> TrackingBits;

            static ProductStructure* ThePS;

        public:
            ProductState(const StateVec* SVPtr, u32 MonitorState,
                         u32 IndexID, u32 NumProcesses);
            ~ProductState();

            const StateVec* GetSVPtr() const;
            u32 GetMonitorState() const;
            u32 GetIndexID() const;
            u64 Hash() const;
            bool operator == (const ProductState& Other) const;

            void ClearMarkings() const;

            void MarkOnStack() const;
            void MarkNotOnStack() const;
            bool IsOnStack() const;

            void MarkThreadedVisited() const;
            void MarkNotThreadedVisited() const;
            bool IsThreadedVisited() const;

            void MarkAccepting() const;
            void MarkNotAccepting() const;
            bool IsAccepting() const;
            
            void MarkDeleted() const;
            void MarkNotDeleted() const;
            bool IsDeleted() const;

            void MarkInSCC(u32 SCCID) const;
            void MarkNotInSCC() const;
            bool IsInSCC(u32 SCCID) const;

            void MarkTracked(u32 BitNum) const;
            void MarkNotTracked(u32 BitNum) const;
            void ClearAllTracked() const;
            bool IsTracked(u32 BitNum) const;
        };

        namespace Detail {

            typedef AnnotatedEdgePtrHasher<ProductState> ProductEdgePtrHasher;
            typedef AnnotatedEdgePtrEquals<ProductState> ProductEdgePtrEquals;

            class ProductStatePtrHasher
            {
            public:
                inline u64 operator () (const ProductState* Ptr) const
                {
                    return Ptr->Hash();
                }
            };

            class ProductStatePtrEquals
            {
            public:
                inline bool operator () (const ProductState* Ptr1, 
                                         const ProductState* Ptr2) const
                {
                    return (*Ptr1 == *Ptr2);
                }
            };

            struct PSFibDataT 
            {
                const ProductState* State;
                u32 DistanceFromOrigin;

                inline PSFibDataT(const ProductState* State, u32 DistanceFromOrigin)
                    : State(State), DistanceFromOrigin(DistanceFromOrigin)
                {
                    // Nothing here
                }
            };

            struct PSFibDataCompare
            {
                // Use a > comparator, since boost fib heaps are max-heaps
                // rather than the text-book min-heaps.
                inline bool operator () (const PSFibDataT& Data1, 
                                         const PSFibDataT& Data2) const
                {
                    return (Data1.DistanceFromOrigin > Data2.DistanceFromOrigin);
                }
            };

        } /* end namespace Detail */

        typedef AnnotatedEdge<ProductState> ProductEdge;
        typedef sparse_hash_set<ProductEdge*, 
                                Detail::ProductEdgePtrHasher,
                                Detail::ProductEdgePtrEquals> ProductEdgeHashSetT;

        typedef sparse_hash_set<ProductEdge*> ProductEdgeSetT;

        typedef sparse_hash_map<ProductState*, ProductEdgeSetT,
                                Detail::ProductStatePtrHasher,
                                Detail::ProductStatePtrEquals> ProductStateHashSetT;

        class ProductStructure
        {
        private:
            u32 NumProcesses;
            ProductStateHashSetT PSHashSet;
            ProductEdgeHashSetT EdgeHashSet;
            vector<ProductState*> InitialStates;
            ProductEdgeSetT EmptyEdgeSet;
            boost::object_pool<ProductState>* PSPool;
            boost::pool<>* PEPool;
            BuchiAutomatonBase* Monitor;

        public:
            ProductStructure(u32 NumProcesses, BuchiAutomatonBase* Monitor);
            ~ProductStructure();

            ProductState* AddInitialState(const StateVec* SVPtr,
                                          u32 MonitorState,
                                          u32 IndexID);

            ProductState* AddState(const StateVec* SVPtr, u32 MonitorState,
                                   u32 IndexID, bool& New);
            void AddEdge(ProductState* Source, ProductState* Target,
                         u32 Permutation, u32 GCmdIndex);

            BuchiAutomatonBase* GetMonitor() const;

            const vector<ProductState*>& GetInitialStates() const;
            const ProductEdgeSetT& GetEdges(ProductState* State) const;
            
            u32 GetNumStates() const;
            u32 GetNumEdges() const;
            void ClearAllMarkings() const;
            u32 GetNumProcesses() const;

            void ApplyToAllStates(const function<void(const ProductState*)>& Func) const;

            // Paths and shortest paths
            PSPermPath* FindPath(const set<const ProductState*>& Origins,
                                 const ProductState* Target) const;
            PSPermPath* FindPath(const set<const ProductState*>& Origins,
                                 const function<bool(const ProductState*)>& TargetPred) const;
            PSPermPath* FindPath(const set<const ProductState*>& Origins,
                                 const function<const ProductState*(const ProductEdgeSetT&)>&
                                 TargetEdgePred) const;
            // From the initial states
            PSPermPath* FindPath(const ProductState* Target) const;
            PSPermPath* FindPath(const function<bool(const ProductState*)>& TargetPred) const;
            PSPermPath* FindPath(const function<const ProductState*(const ProductEdgeSetT&)>&
                                 TargetEdgePred) const;

            // Shortest paths without cost functions = BFS = FindPath
            template <typename... ArgTypes>
            inline PSPermPath* FindShortestPath(ArgTypes&&... Args)
            {
                return FindPath(forward<ArgTypes>(Args)...);
            }

            // Actual shortest paths with cost functions            
            PSPermPath* FindShortestPath(const set<const ProductState*>& Origins,
                                         const ProductState* Target,
                                         const function<u32(const ProductState*, 
                                                            const ProductEdge*)>&
                                         CostFunction) const;
            PSPermPath* FindShortestPath(const set<const ProductState*>& Origins,
                                         const function<bool(const ProductState*)>& TargetPred,
                                         const function<u32(const ProductState*, 
                                                            const ProductEdge*)>&
                                         CostFunction) const;
            PSPermPath* FindShortestPath(const set<const ProductState*>& Origins,
                                         const function<const ProductState*(const ProductEdgeSetT&)>&
                                         TargetEdgePred,
                                         const function<u32(const ProductState*, 
                                                            const ProductEdge*)>&
                                         CostFunction) const;
            // From the initial states
            PSPermPath* FindShortestPath(const ProductState* Target,
                                         const function<u32(const ProductState*, 
                                                            const ProductEdge*)>&
                                         CostFunction) const;
            PSPermPath* FindShortestPath(const function<bool(const ProductState*)>& TargetPred,
                                         const function<u32(const ProductState*, 
                                                            const ProductEdge*)>&
                                         CostFunction) const;
            PSPermPath* FindShortestPath(const function<const ProductState*(const ProductEdgeSetT&)>&
                                         TargetEdgePred,
                                         const function<u32(const ProductState*, 
                                                            const ProductEdge*)>&
                                         CostFunction) const;
        };

    } /* end namespace MC */
} /* end namespace ESMC */

#endif /* ESMC_AQ_STRUCTURE_HPP_ */

// 
// AQStructure.hpp ends here
