// Permutations.cpp ---
//
// Filename: Permutations.cpp
// Author: Abhishek Udupa
// Created: Wed Aug 13 15:58:02 2014 (-0400)
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

#include <algorithm>

#include "../utils/CombUtils.hpp"

#include "Permutations.hpp"

namespace ESMC {
    namespace Symm {

        // Allow use of explicit representation for permutations
        // upto size 8. This will require less than 1MB of
        // memory for explicit representation. Beyond this
        // size, the memory usage will be too high and we
        // use a more compact representation that trades off
        // CPU time for memory usage.
        const u32 DomainPermuter::MaxExplicitSize = 8;

        inline u32 DomainPermuter::GetLehmerCodeForPerm(const vector<u08>& Perm) const
        {
            u32 Retval = 0;
            u32 Multiplier = DomMinusOneFactorial;
            // Assumes that the perm has domain size
            for (u32 i = 0; i < DomainSize; ++i) {
                auto FirstElem = Perm[i];
                u32 Pos = count_if(Perm.begin() + i, Perm.end(),
                                   [=] (u32 Elem) -> bool
                                   { return (Elem < FirstElem); });
                Retval += (Pos * Multiplier);
                if (i + 1 != DomainSize) {
                    Multiplier /= (DomainSize - i - 1);
                }
            }
            return Retval;
        }

        inline void DomainPermuter::GetPermForLehmerCode(u32 Code,
                                                         vector<u08>& OutPerm) const
        {
            u32 Multiplier = DomMinusOneFactorial;
            u32 LeftCode = Code;
            u32 j = 0;
            OutPerm = IdentityPerm;
            for (u32 i = 0; i < DomainSize; ++i) {
                u32 Digit = LeftCode / Multiplier;
                rotate(OutPerm.begin() + j, OutPerm.begin() + Digit + j,
                       OutPerm.begin() + Digit + 1 + j);
                LeftCode = LeftCode % Multiplier;
                ++j;
                if (i + 1 != DomainSize) {
                    Multiplier /= (DomainSize - i - 1);
                }
            }
        }

        inline void DomainPermuter::InvertPerm(const vector<u08>& Perm,
                                                   vector<u08>& OutPerm) const
        {
            for (u32 i = 0; i < DomainSize; ++i) {
                OutPerm[Perm[i]] = i;
            }
        }

        DomainPermuter::DomainPermuter(u32 DomainSize, u32 Offset,
                                               bool Compact)
            : DomainSize(DomainSize), PermSize((u32)Factorial(DomainSize)),
              Offset(Offset), Compact(Compact || DomainSize > MaxExplicitSize),
              DomMinusOneFactorial(Factorial(DomainSize - 1))
        {
            for (u32 i = 0; i < DomainSize; ++i) {
                IdentityPerm.push_back(i);
                CachedPerm.push_back(0);
                CachedInvPerm.push_back(0);
            }

            if (!Compact) {
                PermIdxToInvPermIdx.insert(PermIdxToInvPermIdx.end(), PermSize, 0);

                for (u32 i = 0; i < PermSize; ++i) {
                    GetPermForLehmerCode(i, CachedPerm);
                    InvertPerm(CachedPerm, CachedInvPerm);
                    u32 InvIdx = GetLehmerCodeForPerm(CachedInvPerm);
                    PermIdxToInvPermIdx[i] = InvIdx;
                    Permutations.push_back(CachedPerm);
                    PermToIdxMap[CachedPerm] = i;
                }
            }
        }

        DomainPermuter::~DomainPermuter()
        {
            // Nothing here
        }

        u32 DomainPermuter::GetDomainSize() const
        {
            return DomainSize;
        }

        u32 DomainPermuter::GetOffset() const
        {
            return Offset;
        }

        const vector<u08>& DomainPermuter::GetIdentityPerm() const
        {
            return IdentityPerm;
        }

        u32 DomainPermuter::GetPermSize() const
        {
            return PermSize;
        }

        const vector<u08>& DomainPermuter::GetPerm(u32 Idx) const
        {
            if (!Compact) {
                return Permutations[Idx];
            } else {
                GetPermForLehmerCode(Idx, CachedPerm);
                return CachedPerm;
            }
        }

        const vector<u08>& DomainPermuter::GetInvPerm(u32 Idx) const
        {
            if (!Compact) {
                return Permutations[PermIdxToInvPermIdx[Idx]];
            } else {
                GetPermForLehmerCode(Idx, CachedPerm);
                InvertPerm(CachedPerm, CachedInvPerm);
                u32 InvIdx = GetLehmerCodeForPerm(CachedInvPerm);
                GetPermForLehmerCode(InvIdx, CachedPerm);
                return CachedPerm;
            }
        }

        const vector<u08>& DomainPermuter::GetInvPerm(const vector<u08>& Perm) const
        {
            InvertPerm(Perm, CachedInvPerm);
            return CachedInvPerm;
        }

        void DomainPermuter::GetInvPerm(const vector<u08>& Perm, vector<u08>& OutPerm) const
        {
            InvertPerm(Perm, OutPerm);
        }

        void DomainPermuter::GetPerm(u32 Idx, vector<u08>& OutPerm) const
        {
            if (!Compact) {
                OutPerm = Permutations[Idx];
            } else {
                GetPermForLehmerCode(Idx, OutPerm);
            }
        }

        void DomainPermuter::GetInvPerm(u32 Idx, vector<u08>& OutPerm) const
        {
            if (!Compact) {
                OutPerm = Permutations[PermIdxToInvPermIdx[Idx]];
            } else {
                GetPermForLehmerCode(Idx, CachedPerm);
                GetInvPerm(CachedPerm, OutPerm);
            }
        }

        u32 DomainPermuter::GetPermIdx(const vector<u08>& Perm) const
        {
            if (!Compact) {
                auto it = PermToIdxMap.find(Perm);
                return it->second;
            } else {
                return GetLehmerCodeForPerm(Perm);
            }
        }

        u32 DomainPermuter::GetInvPermIdx(u32 Idx) const
        {
            if (!Compact) {
                return PermIdxToInvPermIdx[Idx];
            } else {
                GetPermForLehmerCode(Idx, CachedPerm);
                InvertPerm(CachedPerm, CachedInvPerm);
                return GetLehmerCodeForPerm(CachedInvPerm);
            }
        }

        u32 DomainPermuter::GetInvPermIdx(const vector<u08>& Perm) const
        {
            if (!Compact) {
                auto it = PermToIdxMap.find(Perm);
                return PermIdxToInvPermIdx[it->second];
            } else {
                InvertPerm(Perm, CachedPerm);
                return GetLehmerCodeForPerm(CachedPerm);
            }
        }

        PermutationSet::iterator::iterator()
            : PermSet(nullptr), Index(UINT32_MAX)
        {
            // Nothing here
        }

        PermutationSet::iterator::iterator(const iterator& Other)
            : PermSet(Other.PermSet), Index(Other.Index)
        {
            // Nothing here
        }

        PermutationSet::iterator::iterator(u32 Index, PermutationSet* PermSet)
            : PermSet(PermSet), Index(Index)
        {
            // Nothing here
        }

        PermutationSet::iterator::~iterator()
        {
            PermSet = nullptr;
            Index = UINT32_MAX;
        }

        PermutationSet::iterator& PermutationSet::iterator::operator ++ ()
        {
            if (Index < PermSet->Size) {
                ++Index;
            }
            return *this;
        }

        PermutationSet::iterator PermutationSet::iterator::operator ++ (int Dummy)
        {
            auto Retval = *this;
            if (Index < PermSet->Size) {
                ++Index;
            }
            return Retval;
        }

        PermutationSet::iterator& PermutationSet::iterator::operator -- ()
        {
            if (Index > 0) {
                --Index;
            }
            return *this;
        }

        PermutationSet::iterator PermutationSet::iterator::operator -- (int Dummy)
        {
            auto Retval = *this;
            if (Index < PermSet->Size) {
                ++Index;
            }
            return Retval;
        }

        PermutationSet::iterator& PermutationSet::iterator::operator += (u32 Addend)
        {
            Index = min(Index + Addend, PermSet->Size);
            return *this;
        }

        PermutationSet::iterator& PermutationSet::iterator::operator -= (u32 Addend)
        {
            if (Addend > Index) {
                Index = 0;
            } else {
                Index -= Addend;
            }
            return *this;
        }

        PermutationSet::iterator PermutationSet::iterator::operator + (u32 Addend) const
        {
            return iterator(min(PermSet->Size, Index + Addend), PermSet);
        }

        PermutationSet::iterator PermutationSet::iterator::operator - (u32 Addend) const
        {
            return iterator(Addend > Index ? 0 : Index - Addend, PermSet);
        }

        const vector<u08>& PermutationSet::iterator::GetPerm() const
        {
            PermSet->GetPermForIndex(Index);
            return PermSet->CachedPerm;
        }

        u32 PermutationSet::iterator::GetIndex() const
        {
            return Index;
        }

        bool PermutationSet::iterator::operator == (const PermutationSet::iterator& Other) const
        {
            return (PermSet == Other.PermSet && Index == Other.Index);
        }

        bool PermutationSet::iterator::operator != (const PermutationSet::iterator& Other) const
        {
            return (!(this->operator==(Other)));
        }

        PermutationSet::iterator&
        PermutationSet::iterator::operator = (const PermutationSet::iterator& Other)
        {
            if (&Other == this) {
                return *this;
            }
            Index = Other.Index;
            PermSet = Other.PermSet;
            return *this;
        }


        PermutationSet::PermutationSet(const vector<u32>& DomainSizes, bool Compact)
            : DomainSizes(DomainSizes), NumDomains(DomainSizes.size()),
              BeginIterator(0, this), EndIterator(0, this)
        {
            u32 Offset = 0;
            u32 TotalSize = 1;
            PermVecSize = 0;
            for (auto const& DomainSize : DomainSizes) {
                DomPermuters.push_back(new DomainPermuter(DomainSize, Offset, Compact));
                Offset += DomainSize;
                Multipliers.push_back(Factorial(DomainSize));
                TotalSize *= Factorial(DomainSize);
                PermVecSize += DomainSize;
                CachedPerm.insert(CachedPerm.end(), DomainSize, 0);
            }
            EndIterator.Index = TotalSize;
            Size = TotalSize;
            CachedIdx = UINT32_MAX;
            for (u32 i = 0; i < NumDomains; ++i) {
                CachedIndices.push_back(UINT32_MAX);
            }
        }

        inline void PermutationSet::GetPermForIndex(u32 Index)
        {
            if (Index == CachedIdx) {
                return;
            }
            // Extract components
            u32 LeftIndex = Index;
            for (u32 i = NumDomains; i > 0; --i) {
                auto CurIndex = LeftIndex % Multipliers[i-1];
                LeftIndex = LeftIndex / Multipliers[i-1];
                if (CachedIndices[i-1] == CurIndex) {
                    continue;
                }
                auto DomPerm = DomPermuters[i-1];
                auto const& CurPerm = DomPerm->GetPerm(CurIndex);
                u32 BeginOffset = DomPerm->GetOffset();
                u32 EndOffset = BeginOffset + DomainSizes[i-1];
                for (u32 j = BeginOffset; j < EndOffset; ++j) {
                    CachedPerm[j] = CurPerm[j-BeginOffset];
                }
                CachedIndices[i-1] = CurIndex;
            }
            CachedIdx = Index;
            return;
        }

        inline void PermutationSet::GetPermForIndex(u32 Index, vector<u08> &OutVec) const
        {
            if (Index == CachedIdx) {
                OutVec = CachedPerm;
            }

            OutVec = CachedPerm;
            // Extract components
            u32 LeftIndex = Index;
            for (u32 i = NumDomains; i > 0; --i) {
                auto CurIndex = LeftIndex % Multipliers[i-1];
                LeftIndex = LeftIndex / Multipliers[i-1];
                if (CachedIndices[i-1] == CurIndex) {
                    continue;
                }
                auto DomPerm = DomPermuters[i-1];
                auto const& CurPerm = DomPerm->GetPerm(CurIndex);
                u32 BeginOffset = DomPerm->GetOffset();
                u32 EndOffset = BeginOffset + DomainSizes[i-1];
                for (u32 j = BeginOffset; j < EndOffset; ++j) {
                    OutVec[j] = CurPerm[j-BeginOffset];
                }
            }
            return;
        }

        inline u32 PermutationSet::GetIndexForPerm(const vector<u08>& Perm) const
        {
            u32 RunningOffset = 0;
            u32 Index = 0;
            for (u32 i = 0; i < NumDomains; ++i) {
                Index = Index * Multipliers[i];
                vector<u08> CurVec(Perm.begin() + RunningOffset,
                                   Perm.begin() + RunningOffset + DomainSizes[i]);
                RunningOffset += DomainSizes[i];
                auto CurIndex = DomPermuters[i]->GetPermIdx(CurVec);
                Index += CurIndex;
            }
            return Index;
        }

        PermutationSet::~PermutationSet()
        {
            for (auto DomPermuter : DomPermuters) {
                delete DomPermuter;
            }
        }

        u32 PermutationSet::GetSize() const
        {
            return Size;
        }

        u32 PermutationSet::GetPermVecSize() const
        {
            return PermVecSize;
        }

        const PermutationSet::iterator& PermutationSet::Begin() const
        {
            return BeginIterator;
        }

        const PermutationSet::iterator& PermutationSet::End() const
        {
            return EndIterator;
        }

        PermutationSet::iterator PermutationSet::GetIterator(u32 Idx) const
        {
            return iterator(Idx, const_cast<PermutationSet*>(this));
        }

        PermutationSet::iterator PermutationSet::GetIteratorForInv(u32 Idx) const
        {
            u32 Retval = 0;
            u32 LeftIndex = Idx;
            vector<u32> InvIndices(NumDomains);
            for (u32 i = NumDomains; i > 0; --i) {
                auto CurIdx = LeftIndex % Multipliers[i-1];
                LeftIndex = LeftIndex / Multipliers[i-1];
                InvIndices[i-1] = DomPermuters[i-1]->GetInvPermIdx(CurIdx);
            }

            for (u32 i = 0; i < NumDomains; ++i) {
                Retval = (Retval * Multipliers[i]) + InvIndices[i];
            }
            return iterator(Retval, const_cast<PermutationSet*>(this));
        }

        PermutationSet::iterator PermutationSet::Compose(u32 PermIdx, u32 Idx)
        {
            return Compose(GetIterator(PermIdx), Idx);
        }

        PermutationSet::iterator PermutationSet::Compose(const iterator& Perm, u32 Idx)
        {
            vector<u08> OrigPerm = Perm.GetPerm();
            GetPermForIndex(Idx);
            vector<u08> Composed(OrigPerm.size());

            u32 Offset = 0;
            for (u32 i = 0; i < NumDomains; ++i) {
                for (u32 j = 0; j < DomainSizes[i]; ++j) {
                    Composed[Offset + j] = OrigPerm[Offset + CachedPerm[Offset + j]];
                }
                Offset += DomainSizes[i];
            }
            auto ComposedIdx = GetIndexForPerm(Composed);
            return iterator(ComposedIdx, this);
        }

        void PermutationSet::Print(u32 PermIdx, ostream& Out) const
        {
            vector<u08> PermVec;
            GetPermForIndex(PermIdx, PermVec);

            u32 RunningOffset = 0;
            for (u32 i = 0; i < NumDomains; ++i) {
                if (i != 0) {
                    Out << ", ";
                }
                Out << "{ ";
                for (u32 j = 0; j < DomainSizes[i]; ++j) {
                    if (j != 0) {
                        Out << ", ";
                    }
                    Out << (u32)(PermVec[RunningOffset++]);
                }
                Out << " }";
            }
        }

    } /* end namespace Symm */
} /* end namespace ESMC */

//
// Permutations.cpp ends here
