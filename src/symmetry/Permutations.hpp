// Permutations.hpp --- 
// 
// Filename: Permutations.hpp
// Author: Abhishek Udupa
// Created: Mon Jul 28 12:52:11 2014 (-0400)
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

#if !defined ESMC_PERMUTATIONS_HPP_
#define ESMC_PERMUTATIONS_HPP_

#include <vector>

#include "../common/FwdDecls.hpp"

namespace ESMC {
    namespace Symm {

        class PermutationSet
        {
        public:
            // definition of an iterator class
            class iterator
            {
                friend class PermutationSet;

            private:
                vector<u32> StateVector;
                PermutationSet* PermSet;
                i64 Index;
                bool Compact;

                inline void Next();
                inline void Prev();

            public:
                iterator();
                iterator(PermutationSet* PermSet, bool Compact);
                iterator(const iterator& Other);
                iterator(iterator&& Other);
                ~iterator();

                iterator& operator ++ ();
                iterator operator ++ (int Dummy);
                
                iterator& operator -- ();
                iterator operator -- (int Dummy);
                
                iterator operator + (u32 Addend) const;
                iterator operator - (u32 Addent) const;
                
                iterator& operator += (u32 Addend);
                iterator& operator -= (u32 Addend);

                iterator& operator = (iterator Other);

                bool operator == (const iterator& Other) const;
                bool operator != (const iterator& Other) const;
                
                const vector<u32>& GetPerm() const;
                u32 GetIndex() const;
            };

            friend class iterator;

        private:
            bool Compact;
            mutable vector<u32> CachedPerm;
            // If we're not using compact representations
            vector<vector<u32>> Permutations;
            u32 NumTypes;
            u32 PermVecSize;
            vector<u32> Offsets;
            iterator BeginIterator;
            iterator EndIterator;
            i64 MaxIndex;
            
        public:
            PermutationSet();
            PermutationSet(const vector<u32>& TypeSizes, bool Compact);
            PermutationSet(const PermutationSet& Other);
            PermutationSet(PermutationSet&& Other);
            ~PermutationSet();

            PermutationSet& operator = (PermutationSet Other);

            const vector<u32>& GetPerm(u32 Index) const;
            void GetPerm(u32 Index, vector<u32>& PermVec) const;
            const vector<u32>& GetInversePerm(u32 Index) const;
            void GetInversePerm(u32 Index, vector<u32>& OutPermVec) const;

            iterator GetIterator(u32 Index) const;
            iterator GetIteratorForInv(u32 Index) const;

            const vector<u32>& GetInversePerm(const vector<u32>& PermVec) const;
            void GetInversePerm(const vector<u32>& PermVec, vector<u32>& OutPermVec) const;

            const iterator& Begin() const;
            const iterator& End() const;
            i64 GetMaxIndex() const;
            u32 GetPermVecSize() const;
            u32 GetOffsetForIdx(u32 Idx) const;
        };

    } /* end namespace Symm */
} /* end namespace ESMC */

#endif /* ESMC_PERMUTATIONS_HPP_ */

// 
// Permutations.hpp ends here
