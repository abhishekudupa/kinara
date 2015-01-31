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
#include <sparse_hash_set>

#include "../uflts/LTSDecls.hpp"

#include "../common/ESMCFwdDecls.hpp"

namespace ESMC {
namespace MC {

using LTS::TypeRef;
using LTS::ExpT;
using LTS::LabelledTS;
using Symm::PermutationSet;
using LTS::EFSMBase;

class IndexSet;

class IndexVector
{
private:
    u08* TheIndexVector;
    u32 Size;

public:
    IndexVector(u08* TheIndexVector, u32 Size);
    ~IndexVector();

    u32 GetSize() const;
    i32 Compare(const IndexVector* Other) const;
    u64 Hash() const;
    u08& operator [] (u32 Index);
    u08 operator [] (u32 Index) const;
    IndexVector* Clone() const;

    u08* GetVector();
    const u08* GetVector() const;
};

namespace Detail {

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

} /* end namespace Detail */

// Manages the index vectors
// for ONE process, which could be
// parametrized by multiple symmetric types
class ProcessIndexSet
{
private:
    vector<u32> TypeOffsets;
    u32 IndexVectorSize;
    u32 NumIndexVectors;
    i32 ClassID;
    // scratchpad for permutations
    mutable IndexVector* WorkingIV;
    map<vector<ExpT>, IndexVector*> ParamVecToIndexVec;
    unordered_map<IndexVector*, vector<ExpT>,
                  Detail::IndexVectorPtrHasher,
                  Detail::IndexVectorPtrEquals> IndexVecToParamVec;

    vector<IndexVector*> IDToIndexVec;
    unordered_map<IndexVector*, u32,
                  Detail::IndexVectorPtrHasher,
                  Detail::IndexVectorPtrEquals> IndexVecToID;

public:
    ProcessIndexSet(const vector<vector<ExpT>>& ParamInsts, i32 ClassID = -1);
    ~ProcessIndexSet();

    i32 GetClassID() const;
    u32 Permute(u32 IndexID, const vector<u08>& Permutation) const;
    u32 GetNumIndexVectors() const;
    const IndexVector* GetIndexVector(u32 IndexID) const;
    u32 GetIndexID(const IndexVector* IndexVec) const;
    const vector<ExpT>& GetParamVecForIndexID(u32 IndexID) const;
    const vector<ExpT>& GetParamVecForIndexVec(const IndexVector* IndexVec) const;
    u32 GetIndexIDForParamVec(const vector<ExpT>& ParamVec) const;
    const IndexVector* GetIndexVecForParamVec(const vector<ExpT>& ParamVec) const;
};

// At any point of time, one index from the set of
// all process indices is tracked. Used in threaded
// graph construction/simulation
class SystemIndexSet
{
private:
    vector<ProcessIndexSet*> ProcessIdxSets;
    vector<u32> DomainSizes;
    u32 NumTrackedIndices;
    vector<pair<ProcessIndexSet*, u32>> IndexToPIdx;
    // Low, High for a class id
    vector<pair<u32, u32>> ClassIDBounds;

public:
    SystemIndexSet(const vector<vector<vector<ExpT>>>& ProcessParamInsts);
    ~SystemIndexSet();

    u32 Permute(u32 IndexID, const vector<u08>& Permutation) const;
    u32 GetNumTrackedIndices() const;

    u32 GetClassID(u32 IndexID) const;
    i32 GetIndexForClassID(u32 IndexID, u32 ClassID) const;
    i32 GetIndexIDForClassIndex(u32 ClassIndex, u32 ClassID) const;
};

} /* end namespace MC */
} /* end namespace ESMC */

#endif /* ESMC_INDEX_SET_HPP_ */

//
// IndexSet.hpp ends here
