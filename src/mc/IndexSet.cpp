// IndexSet.cpp --- 
// 
// Filename: IndexSet.cpp
// Author: Abhishek Udupa
// Created: Fri Aug 29 01:45:29 2014 (-0400)
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

#include <string.h>
#include <boost/functional/hash.hpp>
#include "../hash/SpookyHash.hpp"

#include "../utils/CombUtils.hpp"
#include "../uflts/LTSEFSMBase.hpp"

#include "IndexSet.hpp"

namespace ESMC {
    namespace MC {

        using LTS::ExprTypeRef;
        using LTS::ExpT;
        using Symm::PermutationSet;
        using LTS::LTSTypeExtensionT;
        using LTS::EFSMBase;

        IndexVector::IndexVector(u08* TheIndexVector, u32 Size)
            : TheIndexVector(TheIndexVector), Size(Size)
        {
            // Nothing here
        }

        IndexVector::~IndexVector()
        {
            free(TheIndexVector);
        }

        u32 IndexVector::GetSize() const
        {
            return Size;
        }

        i32 IndexVector::Compare(const IndexVector* Other) const
        {
            auto Cmp = Size - Other->Size;
            if (Cmp != 0) {
                return Cmp;
            }

            return memcmp(TheIndexVector, Other->TheIndexVector, Size);
        }

        u64 IndexVector::Hash() const
        {
            return SpookyHash::SpookyHash::Hash64(TheIndexVector, Size, Size);
        }

        u08& IndexVector::operator [] (u32 Index)
        {
            return TheIndexVector[Index];
        }

        u08 IndexVector::operator [] (u32 Index) const
        {
            return TheIndexVector[Index];
        }

        IndexVector* IndexVector::Clone() const
        {
            auto NewIndexVec = (u08*)malloc(sizeof(u08*) * Size);
            memcpy(NewIndexVec, TheIndexVector, Size);
            return new IndexVector(NewIndexVec, Size);
        }

        u08* IndexVector::GetVector()
        {
            return TheIndexVector;
        }

        const u08* IndexVector::GetVector() const
        {
            return TheIndexVector;
        }

        ProcessIndexSet::ProcessIndexSet(const vector<vector<ExpT>>& ParamInsts,
                                         i32 ClassID)
            : ClassID(ClassID)
        {

            if (ParamInsts.size() == 1 && ParamInsts[0].size() == 0) {
                // Non parametrized process
                IndexVectorSize = 1;
                NumIndexVectors = 1;
                auto TheIV = (u08*)malloc(sizeof(u08));
                TheIV[0] = 0;
                auto TheIndexVec = new IndexVector(TheIV, IndexVectorSize);
                ParamVecToIndexVec[ParamInsts[0]] = TheIndexVec;
                IndexVecToParamVec[TheIndexVec] = ParamInsts[0];
                IDToIndexVec.push_back(TheIndexVec);
                IndexVecToID[TheIndexVec] = 0;
                auto TempIV = (u08*)malloc(sizeof(u08) * IndexVectorSize);
                WorkingIV = new IndexVector(TempIV, IndexVectorSize);
                return;
            }

            // The general case when there's more than one instance
            IndexVectorSize = ParamInsts[0].size();
            NumIndexVectors = ParamInsts.size();

            auto TempIV = (u08*)malloc(sizeof(u08) * IndexVectorSize);
            WorkingIV = new IndexVector(TempIV, IndexVectorSize);

            // figure out the type offsets
            for (auto const& Exp : ParamInsts[0]) {
                auto const& Type = Exp->GetType();
                auto Ext = Type->GetExtension<LTS::LTSTypeExtensionT>();
                TypeOffsets.push_back(Ext->TypeOffset);
            }

            for (u32 j = 0; j < NumIndexVectors; ++j) {
                auto const& ParamInst = ParamInsts[j];
                auto CurIV = (u08*)malloc(sizeof(u08) * IndexVectorSize);
                for (u32 i = 0; i < IndexVectorSize; ++i) {
                    auto CurExp = ParamInst[i]->As<Exprs::ConstExpression>();
                    auto const& CurVal = CurExp->GetConstValue();
                    auto const& CurType = CurExp->GetConstType()->As<Exprs::ExprScalarType>();
                    
                    auto IndexVal = CurType->ConstToVal(CurVal) - 1;
                    CurIV[i] = (u08)IndexVal;
                }
                auto TheIV = new IndexVector(CurIV, IndexVectorSize);
                ParamVecToIndexVec[ParamInst] = TheIV;
                IndexVecToParamVec[TheIV] = ParamInst;
                IDToIndexVec.push_back(TheIV);
                IndexVecToID[TheIV] = j;
            }
        }

        ProcessIndexSet::~ProcessIndexSet()
        {
            for (auto const& IV : IDToIndexVec) {
                delete IV;
            }
            delete WorkingIV;
        }

        i32 ProcessIndexSet::GetClassID() const
        {
            return ClassID;
        }

        u32 ProcessIndexSet::Permute(u32 IndexID, const vector<u08>& Perm) const
        {
            if (NumIndexVectors == 1) {
                return IndexID;
            }
            auto OrigIV = IDToIndexVec[IndexID];
            for (u32 i = 0; i < IndexVectorSize; ++i) {
                (*WorkingIV)[i] = Perm[TypeOffsets[i] + (*OrigIV)[i]];
            }
            auto it = IndexVecToID.find(WorkingIV);
            if (it == IndexVecToID.end()) {
                throw InternalError((string)"Could not resolve index vector.\nAt: " + 
                                    __FILE__ + ":" + to_string(__LINE__));
            }
            return it->second;
        }

        u32 ProcessIndexSet::GetNumIndexVectors() const
        {
            return NumIndexVectors;
        }

        const IndexVector* ProcessIndexSet::GetIndexVector(u32 IndexID) const
        {
            return IDToIndexVec[IndexID];
        }

        u32 ProcessIndexSet::GetIndexID(const IndexVector *IndexVec) const
        {
            auto it = IndexVecToID.find(const_cast<IndexVector*>(IndexVec));
            return it->second;
        }

        
        SystemIndexSet::SystemIndexSet(const vector<vector<vector<ExpT>>>& ProcessParamInsts)
            : NumTrackedIndices(0)
        {
            u32 ClassID = 0;
            for (auto const& ProcessParamInst : ProcessParamInsts) {
                auto CurPIdxSet = new ProcessIndexSet(ProcessParamInst, ClassID);
                ProcessIdxSets.push_back(CurPIdxSet);
                auto CurSize = CurPIdxSet->GetNumIndexVectors();
                DomainSizes.push_back(CurSize);
                for (u32 i = 0; i < CurSize; ++i) {
                    IndexToPIdx.push_back(make_pair(CurPIdxSet, NumTrackedIndices));
                }
                ClassIDBounds.push_back(make_pair(NumTrackedIndices, 
                                                  NumTrackedIndices + CurSize - 1));
                ++ClassID;
                NumTrackedIndices += CurSize;
            }
        }

        SystemIndexSet::~SystemIndexSet()
        {
            for (auto PIdxSet : ProcessIdxSets) {
                delete PIdxSet;
            }
        }

        u32 SystemIndexSet::Permute(u32 IndexID, const vector<u08>& Permutation) const
        {
            // Get the index set and offset
            auto const& IdxOffset = IndexToPIdx[IndexID];
            auto PIdxSet = IdxOffset.first;
            auto Offset = IdxOffset.second;

            auto Retval = PIdxSet->Permute(IndexID - Offset, Permutation);
            return Retval + Offset;
        }

        u32 SystemIndexSet::GetNumTrackedIndices() const
        {
            return NumTrackedIndices;
        }

        u32 SystemIndexSet::GetClassID(u32 IndexID) const
        {
            return (u32)(IndexToPIdx[IndexID].first->GetClassID());
        }

        i32 SystemIndexSet::GetIndexForClassID(u32 IndexID, u32 ClassID) const
        {
            auto const& Bounds = ClassIDBounds[ClassID];
            if (IndexID >= Bounds.first && IndexID <= Bounds.second) {
                return (IndexID - Bounds.first);
            } else {
                return -1;
            }
        }

    } /* end namespace MC */
} /* end namespace ESMC */

// 
// IndexSet.cpp ends here
