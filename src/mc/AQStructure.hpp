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

#include <unordered_set>
#include <unordered_map>
#include <boost/functional/hash.hpp>

#include "../common/FwdDecls.hpp"

#include "StateVec.hpp"

namespace ESMC {
    namespace MC {

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


        class AQSEdge
        {
        private:
            const StateVec* Target;
            u32 Permutation;

        public:
            AQSEdge();
            AQSEdge(const StateVec* Target, u32 Permutation);
            AQSEdge(const AQSEdge& Other);
            ~AQSEdge();
            
            AQSEdge& operator = (const AQSEdge& Other);
            bool operator == (const AQSEdge& Other) const;

            const StateVec* GetTarget() const;
            u32 GetPermutation() const;
        };

        namespace Detail {

            class AQSEdgeHash
            {
            public:
                inline u64 operator () (const AQSEdge& Edge) const
                {
                    u64 Retval = 0;
                    boost::hash_combine(Retval, Edge.GetTarget());
                    boost::hash_combine(Retval, Edge.GetPermutation());
                    return Retval;
                }
            };

        } /* end namespace Detail */

        typedef unordered_set<AQSEdge, Detail::AQSEdgeHash> AQSEdgeSetT;


        typedef unordered_map<StateVec*, AQSEdgeSetT,
                              Detail::StateVecPtrHasher,
                              Detail::StateVecPtrEquals> SVHashSetT;

        class AQStructure
        {
        private:
            SVHashSetT StateSet;
            vector<StateVec*> InitStates;

        public:
            AQStructure();
            virtual ~AQStructure();

            StateVec* Find(StateVec* SV) const;
            void Insert(StateVec* SV);
            void InsertInitState(StateVec* SV);
            void AddEdge(StateVec* Source, StateVec* Target, u32 Permutation);
        };

    } /* end namespace MC */
} /* end namespace ESMC */

#endif /* ESMC_AQ_STRUCTURE_HPP_ */

// 
// AQStructure.hpp ends here
