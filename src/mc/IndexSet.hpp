// IndexSet.hpp --- 
// 
// Filename: IndexSet.hpp
// Author: Abhishek Udupa
// Created: Fri Aug 29 01:17:44 2014 (-0400)
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

#if !defined ESMC_INDEX_SET_HPP_
#define ESMC_INDEX_SET_HPP_

#include <boost/pool/pool.hpp>
#include <unordered_map>

#include "../uflts/LTSTypes.hpp"

#include "../common/FwdDecls.hpp"

namespace ESMC {
    namespace MC {

        using LTS::ExprTypeRef;
        using LTS::ExpT;
        using Symm::PermutationSet;

        class IndexSet;

        class IndexVector
        {
        private:
            u08* TheIndexVector;
            u32 Size;

        public:
            IndexVector(u08* TheIndexVector, u32 Size);
            ~IndexVector();

            i32 Compare(const IndexVector* Other) const;
            u64 Hash() const;
            u08& operator [] (u32 Index);
            u08 operator [] (u32 Index) const;
            IndexVector* Clone() const;
        };

        class IndexVectorPtrHasher
        {
        public:
            inline u64 operator () (const IndexVector* IndexVec) const
            {
                return IndexVec->Hash();
            }
        };

        class IndexVectorPtrEquals
        {
        public:
            inline bool operator () (const IndexVector* IndexVec1,
                                     const IndexVector* IndexVec2) const
            {
                return (IndexVec1->Compare(IndexVec2) == 0);
            }
        };

        class IndexVectorPtrLessThan
        {
        public:
            inline bool operator () (const IndexVector* IndexVec1,
                                     const IndexVector* IndexVec2) const
            {
                return (IndexVec1->Compare(IndexVec2) < 0);
            }
        };

        // Manages the index vectors
        // for ONE process, which could be 
        // parametrized by multiple symmetric types
        class ProcessIndexSet
        {
        private:
            vector<u32> TypeOffsets;
            u32 IndexVectorSize;
            u32 NumIndexVectors;
            // scratchpad for permutations
            mutable IndexVector* WorkingIV;

            vector<IndexVector*> IDToIndexVec;
            unordered_map<IndexVector*, u32,
                          IndexVectorPtrHasher,
                          IndexVectorPtrEquals> IndexVecToID;

        public:
            ProcessIndexSet(const vector<vector<ExpT>>& ParamInsts);
            ~ProcessIndexSet();

            u32 Permute(u32 IndexID, const vector<u08>& Permutation) const;
            u32 GetNumIndexVectors() const;
        };

        class SystemIndexSet
        {
        private:
            vector<ProcessIndexSet*> ProcessIdxSets;
            vector<u32> Multipliers;
            // Scratchpad
            mutable vector<u32> Scratchpad;

        public:
            SystemIndexSet(const vector<vector<vector<ExpT>>>& ParamInsts);
            ~SystemIndexSet();

            u32 Permute(u32 IndexID, const vector<u08>& Permutation) const;
        };

    } /* end namespace MC */
} /* end namespace ESMC */

#endif /* ESMC_INDEX_SET_HPP_ */

// 
// IndexSet.hpp ends here












