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

#include <boost/pool/pool.hpp>

#include "../common/FwdDecls.hpp"

#include "StateVec.hpp"

namespace ESMC {
    namespace MC {

        using google::sparse_hash_set;
        using google::sparse_hash_map;

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

        public:
            AQStructure();
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
        };

        // A bitfield structure for the status bits 
        // in the product state
        struct ThreadedGraphStatusT
        {
            bool InSCC : 1;
            bool OnStack : 1;
            bool Singular : 1;
            bool Accepting : 1;

            inline ThreadedGraphStatusT()
                : InSCC(false), OnStack(false),
                  Singular(false), Accepting(false)
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
            mutable vector<bool> EnExBits;

        public:
            ProductState(const StateVec* SVPtr, u32 MonitorState,
                         u32 IndexID, u32 NumProcesses, u32 NumEnExBits);
            ~ProductState();

            const StateVec* GetSVPtr() const;
            u32 GetMonitorState() const;
            u32 GetIndexID() const;
            u64 Hash() const;
            bool operator == (const ProductState& Other) const;

            void ClearMarkings() const;
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
            u32 NumEnExBits;
            ProductStateHashSetT PSHashSet;
            ProductEdgeHashSetT EdgeHashSet;
            vector<ProductState*> InitialStates;
            ProductEdgeSetT EmptyEdgeSet;
            boost::pool<>* PSPool;
            boost::pool<>* PEPool;

        public:
            ProductStructure(u32 NumProcesses, u32 NumEnExBits);
            ~ProductStructure();

            ProductState* AddInitialState(const StateVec* SVPtr,
                                          u32 MonitorState,
                                          u32 IndexID);

            ProductState* AddState(const StateVec* SVPtr, u32 MonitorState,
                                   u32 IndexID, bool& New);
            void AddEdge(ProductState* Source, ProductState* Target,
                         u32 Permutation, u32 GCmdIndex);

            const vector<ProductState*>& GetInitialStates() const;
            const ProductEdgeSetT& GetEdges(ProductState* State) const;
            
            u32 GetNumStates() const;
            u32 GetNumEdges() const;
            void ClearAllMarkings() const;
            u32 GetNumProcesses() const;
        };

    } /* end namespace MC */
} /* end namespace ESMC */

#endif /* ESMC_AQ_STRUCTURE_HPP_ */

// 
// AQStructure.hpp ends here
