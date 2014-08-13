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

#include "Permutations.hpp"
#include <algorithm>

namespace ESMC {
    namespace Symm {

        PermutationSet::iterator::iterator()
            : PermSet(nullptr), Index(-1), Compact(false)
        {
            // Nothing here
        }

        PermutationSet::iterator::iterator(PermutationSet* PermSet, bool Compact)
            : PermSet(PermSet), Index(0), Compact(Compact)
        {
            auto const& Offsets = PermSet->Offsets;

            if (!Compact) {
                for (auto Offset : Offsets) {
                    for (u32 i = 0; i < Offset; ++i) {
                        StateVector.push_back(i);
                    }
                }
            }
        }

        PermutationSet::iterator::iterator(const iterator& Other)
            : StateVector(Other.StateVector),
              PermSet(Other.PermSet), Index(Other.Index),
              Compact(Other.Compact)
        {
            // Nothing here
        }

        PermutationSet::iterator::iterator(iterator&& Other)
            : iterator()
        {
            swap(StateVector, Other.StateVector);
            swap(PermSet, Other.PermSet);
            swap(Index, Other.Index);
            swap(Compact, Other.Compact);
        }

        PermutationSet::iterator::~iterator()
        {
            // Nothing here
        }

        inline void PermutationSet::iterator::Next()
        {
            if (*this == PermSet->EndIterator) {
                return;
            }

            if (!Compact) {
                Index++;
                return;
            }

            auto const& Offsets = PermSet->Offsets;

            auto PrevIterator = StateVector.begin();
            u32 OffsetIdx = 0;
            
            bool FoundPermutation = false;
            do {
                auto CurIterator = StateVector.begin() + Offsets[OffsetIdx];
                FoundPermutation = next_permutation(PrevIterator, CurIterator);

                // set to lexicographically lowest permutation
                u32 i = 0;
                if (!FoundPermutation) {
                    for (auto it = PrevIterator; it != CurIterator; ++it, ++i) {
                        *it = i;
                    }
                } else {
                    Index++;
                }
                PrevIterator = CurIterator;
                OffsetIdx++;
            } while (!FoundPermutation && OffsetIdx < Offsets.size());
            
            if (!FoundPermutation) {
                Index = PermSet->MaxIndex;
            }
        }

        inline void PermutationSet::iterator::Prev()
        {
            if (*this == PermSet->BeginIterator) {
                return;
            }

            if (!Compact) {
                Index--;
                return;
            }

            auto const& Offsets = PermSet->Offsets;

            auto PrevIterator = StateVector.begin();
            u32 OffsetIdx = 0;
            bool FoundPermutation = false;

            do {
                auto CurIterator = StateVector.begin() + Offsets[OffsetIdx];
                FoundPermutation = prev_permutation(PrevIterator, CurIterator);

                // set to lexicographically highest permutation
                u32 i = Offsets[OffsetIdx] - 1;
                if (!FoundPermutation) {
                    for (auto it = PrevIterator; it != CurIterator; ++it, --i) {
                        *it = i;
                    }
                } else {
                    Index--;
                }
                PrevIterator = CurIterator;
                OffsetIdx++;
            } while (!FoundPermutation && OffsetIdx < Offsets.size());
        }

        PermutationSet::iterator& PermutationSet::iterator::operator ++ ()
        {
            Next();
            return *this;
        }

        PermutationSet::iterator PermutationSet::iterator::operator ++ (int Dummy)
        {
            auto Retval = *this;
            Next();
            return Retval;
        }

        PermutationSet::iterator& PermutationSet::iterator::operator -- ()
        {
            Prev();
            return *this;
        }

        PermutationSet::iterator PermutationSet::iterator::operator -- (int Dummy)
        {
            auto Retval = *this;
            Prev();
            return Retval;
        }

        PermutationSet::iterator PermutationSet::iterator::operator + (u32 Addend) const
        {
            auto Retval = *this;
            for (u32 i = 0; i < Addend; ++i) {
                Retval.Next();
            }
            return Retval;
        }

        PermutationSet::iterator PermutationSet::iterator::operator - (u32 Addend) const
        {
            auto Retval = *this;
            for (u32 i = 0; i < Addend; --i) {
                Retval.Prev();
            }
            return Retval;
        }

        PermutationSet::iterator& PermutationSet::iterator::operator += (u32 Addend)
        {
            for (u32 i = 0; i < Addend; ++i) {
                Next();
            }
            return *this;
        }

        PermutationSet::iterator& PermutationSet::iterator::operator -= (u32 Addend)
        {
            for (u32 i = 0; i < Addend; ++i) {
                Prev();
            }
            return *this;
        }

        PermutationSet::iterator& PermutationSet::iterator::operator = (iterator Other)
        {
            swap(StateVector, Other.StateVector);
            swap(PermSet, Other.PermSet);
            swap(Index, Other.Index);
            swap(Compact, Other.Compact);
            return *this;
        }

        bool PermutationSet::iterator::operator == (const iterator& Other) const
        {
            return (PermSet == Other.PermSet && Index == Other.Index);
        }

        bool PermutationSet::iterator::operator != (const iterator& Other) const
        {
            return (!(this->operator==(Other)));
        }

        const vector<u32>& PermutationSet::iterator::GetPerm() const
        {
            if (Compact) {
                return StateVector;
            } else {
                return PermSet->Permutations[Index];
            }
        }

        u32 PermutationSet::iterator::GetIndex() const
        {
            return Index;
        }

        static inline i64 Factorial(u32 Num)
        {
            i64 Retval = 1;
            for (u32 i = 1; i <= Num; ++i) {
                Retval *= i;
            }
            return Retval;
        }

        // Implementation of permutation set
        PermutationSet::PermutationSet()
            : Compact(false), NumTypes(0),
              BeginIterator(this, false), EndIterator(this, false)
        {
            // Nothing here
        }

        PermutationSet::PermutationSet(const vector<u32>& TypeSizes, bool Compact)
            : Compact(Compact), NumTypes(TypeSizes.size()),
              BeginIterator(this, Compact), EndIterator(this, Compact)
        {
            BeginIterator.Index = 0;
            EndIterator.Index = 1;
            auto CumulativeOffset = 0;
            PermVecSize = 0;
            for (auto const& TypeSize : TypeSizes) {
                CumulativeOffset += TypeSize;
                Offsets.push_back(CumulativeOffset);
                EndIterator.Index *= Factorial(TypeSize);
                PermVecSize += TypeSize;
                for (u32 i = 0; i < TypeSize; ++i) {
                    BeginIterator.StateVector.push_back(i);
                }
            }
            
            MaxIndex = EndIterator.Index;

            if (!Compact) {
                iterator it(this, true);
                it.StateVector = BeginIterator.StateVector;
                BeginIterator.StateVector.clear();

                for (; it != EndIterator; ++it) {
                    Permutations.push_back(it.GetPerm());
                }
            }
        }

        PermutationSet::PermutationSet(const PermutationSet& Other) 
            : Compact(Other.Compact), CachedPerm(Other.CachedPerm),
              Permutations(Other.Permutations), NumTypes(Other.NumTypes),
              PermVecSize(Other.PermVecSize), Offsets(Other.Offsets), 
              BeginIterator(Other.BeginIterator), EndIterator(Other.EndIterator), 
              MaxIndex(Other.MaxIndex)
        {
            // Nothing here
        }

        PermutationSet::PermutationSet(PermutationSet&& Other)
            : PermutationSet()
        {
            swap(Compact, Other.Compact);
            swap(CachedPerm, Other.CachedPerm);
            swap(Permutations, Other.Permutations);
            swap(NumTypes, Other.NumTypes);
            swap(PermVecSize, Other.PermVecSize);
            swap(Offsets, Other.Offsets);
            swap(BeginIterator, Other.BeginIterator);
            swap(EndIterator, Other.EndIterator);
            swap(MaxIndex, Other.MaxIndex);
        }

        PermutationSet::~PermutationSet()
        {
            // Nothing here
        }

        const vector<u32>& PermutationSet::GetPerm(u32 Index) const
        {
            GetPerm(Index, CachedPerm);
            return CachedPerm;
        }

        void PermutationSet::GetPerm(u32 Index, vector<u32>& PermVec) const
        {
            if (Index > MaxIndex) {
                PermVec = vector<u32>(PermVecSize, 0);
                return;
            }
            if (Compact) {
                auto it = BeginIterator;
                it += Index;
                PermVec = it.GetPerm();
                return;
            } else {
                PermVec = Permutations[Index];
                return;
            }
        }

        const vector<u32>& PermutationSet::GetInversePerm(u32 Index) const
        {
            vector<u32> PermVec;
            GetPerm(Index, PermVec);
            GetInversePerm(PermVec, CachedPerm);
            return CachedPerm;
        }
        
        void PermutationSet::GetInversePerm(u32 Index, vector<u32>& OutPermVec) const
        {
            vector<u32> PermVec;
            GetPerm(Index, PermVec);
            GetInversePerm(PermVec, OutPermVec);
        }

        const vector<u32>& PermutationSet::GetInversePerm(const vector<u32>& PermVec) const
        {
            GetInversePerm(PermVec, CachedPerm);
            return CachedPerm;
        }

        void PermutationSet::GetInversePerm(const vector<u32> &PermVec, 
                                            vector<u32>& OutPermVec) const
        {
            u32 CurIdx = 0;
            OutPermVec = vector<u32>(PermVecSize, 0);

            for (auto Offset : Offsets) {
                for (u32 i = 0; i + CurIdx < Offset; ++i) {
                    OutPermVec[PermVec[i + CurIdx]] = i;
                }
                CurIdx = Offset;
            }
        }

        PermutationSet& PermutationSet::operator = (PermutationSet Other)
        {
            swap(Compact, Other.Compact);
            swap(CachedPerm, Other.CachedPerm);
            swap(Permutations, Other.Permutations);
            swap(NumTypes, Other.NumTypes);
            swap(PermVecSize, Other.PermVecSize);
            swap(Offsets, Other.Offsets);
            swap(BeginIterator, Other.BeginIterator);
            swap(EndIterator, Other.EndIterator);
            swap(MaxIndex, Other.MaxIndex);
            return *this;
        }
        
        const PermutationSet::iterator& PermutationSet::Begin() const
        {
            return BeginIterator;
        }

        const PermutationSet::iterator& PermutationSet::End() const
        {
            return EndIterator;
        }

        i64 PermutationSet::GetMaxIndex() const
        {
            return MaxIndex;
        }

        u32 PermutationSet::GetPermVecSize() const
        {
            return PermVecSize;
        }

        u32 PermutationSet::GetOffsetForIdx(u32 Idx) const
        {
            return Offsets[Idx];
        }

    } /* end namespace Symm */
} /* end namespace ESMC */

// 
// Permutations.cpp ends here
