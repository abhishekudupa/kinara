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

#include "IndexSet.hpp"

namespace ESMC {
    namespace MC {

        using LTS::ExprTypeRef;
        using LTS::ExpT;
        using Symm::PermutationSet;
        using LTS::LTSTypeExtensionT;

        IndexVector::IndexVector(u08* TheIndexVector, u32 Size)
            : TheIndexVector(TheIndexVector), Size(Size)
        {
            // Nothing here
        }

        IndexVector::~IndexVector()
        {
            free(TheIndexVector);
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

        ProcessIndexSet::ProcessIndexSet(const vector<vector<ExpT>>& ParamInsts)
        {
            NumIndexVectors = ParamInsts.size();

            vector<ExprTypeRef> IndexTypes;
            map<ExprTypeRef, map<string, u32>> ValueMaps;

            for (auto const& Exp : ParamInsts[0]) {
                auto const& Type = Exp->GetType();
                IndexTypes.push_back(Type);
                auto TypeExt = Type->GetExtension<LTSTypeExtensionT>();
                auto TypeOffset = TypeExt->TypeOffset;

                IndexTypes.push_back(Type);
                TypeOffsets.push_back(TypeOffset);
                IndexVectorSize++;

                // Populate the value maps
                auto const& Values = Type->GetElements();
                u32 i = 0;
                for (auto const& Value : Values) {
                    ValueMaps[Type][Value] = i;
                    ++i;
                }
            }
            
            // Populate the index vectors
            
            auto&& CPVecs = CrossProduct<ExpT>(ParamInsts.begin(), ParamInsts.end());
            u32 CurID = 0;
            for (auto const& CPTuple : CPVecs) {
                u08* CurIV = (u08*)malloc(sizeof(u08) * IndexVectorSize);
                for (u32 i = 0; i < CPTuple.size(); ++i) {
                    CurIV[i] = ValueMaps[CPTuple[i]->GetType()][CPTuple[i]->ToString()];
                }
                auto CurIndexVec = new IndexVector(CurIV, IndexVectorSize);
                IDToIndexVec.push_back(CurIndexVec);
                IndexVecToID[CurIndexVec] = CurID;
                ++CurID;
            }

            auto TempIV = (u08*)malloc(sizeof(u08) * IndexVectorSize);
            WorkingIV = new IndexVector(TempIV, IndexVectorSize);
        }

        ProcessIndexSet::~ProcessIndexSet()
        {
            for (auto const& IV : IDToIndexVec) {
                delete IV;
            }
            delete WorkingIV;
        }

        u32 ProcessIndexSet::Permute(u32 IndexID, const vector<u08>& Perm) const
        {
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

        SystemIndexSet::SystemIndexSet(const vector<vector<vector<ExpT>>>& ParamInsts)
        {
            for (auto const& ParamInst : ParamInsts) {
                ProcessIdxSets.push_back(new ProcessIndexSet(ParamInst));
                Multipliers.push_back(ProcessIdxSets.back()->GetNumIndexVectors());
                Scratchpad.push_back(0);
            }
        }

        SystemIndexSet::~SystemIndexSet()
        {
            for (auto PIdxSet : ProcessIdxSets) {
                delete PIdxSet;
            }
        }

        u32 SystemIndexSet::Permute(u32 IndexID, const vector<u08>& Perm) const
        {
            auto LeftIdx = IndexID;
            
            for (u32 i = 0; i < ProcessIdxSets.size(); ++i) {
                auto CurID = LeftIdx % Multipliers[i];
                LeftIdx = LeftIdx / Multipliers[i];
                Scratchpad[i] = CurID;
            }

            // Now build up the return value
            u32 Retval = 0;
            for (u32 i = ProcessIdxSets.size(); i > 0; --i) {
                Retval = (Retval * Multipliers[i-1]) + Scratchpad[i-1];
            }
            return Retval;
        }

    } /* end namespace MC */
} /* end namespace ESMC */

// 
// IndexSet.cpp ends here
