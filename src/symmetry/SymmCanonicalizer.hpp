// SymmCanonicalizer.hpp --- 
// 
// Filename: SymmCanonicalizer.hpp
// Author: Abhishek Udupa
// Created: Sun Aug 17 17:36:42 2014 (-0400)
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

#if !defined ESMC_SYMM_CANONICALIZER_HPP_
#define ESMC_SYMM_CANONICALIZER_HPP_

#include <vector>

#include "../common/FwdDecls.hpp"

#include "Permutations.hpp"

namespace ESMC {
    namespace Symm {

        using ESMC::MC::StateFactory;
        using ESMC::MC::StateVec;
        using ESMC::Exprs::ExprTypeRef;
        using ESMC::LTS::LabelledTS;
        using ESMC::LTS::ExpT;

        extern const u32 MaxExplicitSize;

        class PermuterBase
        {
        protected:
            // Offset of the element to be permuted
            u32 Offset;
            // Offset for the type in the permutation vector
            u32 TypeOffset;
            // Size of the permutation, essentially, this means
            // apply the permutation contained in the range
            // [TypeOffset, TypeOffset + PermSize) of the 
            // permutation vector
            u32 PermSize;

        public:
            PermuterBase(u32 Offset, u32 TypeOffset, u32 PermSize);
            virtual ~PermuterBase();

            virtual void Permute(const StateVec* InStateVector,
                                 StateVec* OutStateVector,
                                 const PermutationSet::iterator& PermIter) = 0;

            u32 GetOffset() const;
            u32 GetTypeOffset() const;
            u32 GetPermSize() const;

            static PermuterBase* MakePermuter(u32 Offset, const ExprTypeRef& Type,
                                              const LabelledTS* TheLTS);
        };

        class ArrayPermuter : public PermuterBase
        {
        private:
            u32 ElemSize;
            u32 NumElems;
            vector<PermuterBase*> ElemPermuters;

        public:
            ArrayPermuter(u32 Offset, const ExprTypeRef& ArrayType,
                          const LabelledTS* TheLTS);
            virtual ~ArrayPermuter();

            u32 GetElemSize() const;
            u32 GetNumElems() const;
            const vector<PermuterBase*>& GetElemPermuters() const;

            virtual void Permute(const StateVec* InStateVector,
                                 StateVec* OutStateVector,
                                 const PermutationSet::iterator& CurPerm) override;
        };

        class RecordPermuter : public PermuterBase
        {
        private:
            vector<PermuterBase*> ElemPermuters;

        public:
            RecordPermuter(u32 Offset, const ExprTypeRef& RecordType,
                           const LabelledTS* TheLTS);
            virtual ~RecordPermuter();

            const vector<PermuterBase*>& GetElemPermuters() const;
            virtual void Permute(const StateVec* InStateVector,
                                 StateVec* OutStateVector,
                                 const PermutationSet::iterator& CurPerm) override;
        };

        class MTypePermuter : public PermuterBase
        {
        private:
            vector<vector<u32>> MsgCanonMap;
            u32 TypeSize;

        public:
            MTypePermuter(u32 Offset, const ExprTypeRef& Type, 
                          const LabelledTS* TheLTS);

            virtual ~MTypePermuter();
            virtual void Permute(const StateVec* InStateVector,
                                 StateVec* OutStateVector,
                                 const PermutationSet::iterator& CurPerm) override;
        };

        class SymmTypePermuter : public PermuterBase
        {
        public:
            SymmTypePermuter(u32 Offset, const ExprTypeRef& Type,
                             const LabelledTS* TheLTS);
            virtual ~SymmTypePermuter();

            virtual void Permute(const StateVec* InStateVector,
                                 StateVec* OutStateVector,
                                 const PermutationSet::iterator& CurPerm) override;
        };

        class NoOpPermuter : public PermuterBase
        {
        public:
            NoOpPermuter();
            virtual ~NoOpPermuter();
            
            virtual void Permute(const StateVec* InStateVector,
                                 StateVec* OutStateVector,
                                 const PermutationSet::iterator& CurPerm) override;
        };

        // Sort a channel buffer (for unordered channels)
        class ChanBufferSorter
        {
        private:
            u32 Offset;
            u32 ElemSize;
            ExpT CountExp;

        public:
            ChanBufferSorter(u32 Offset, const ExprTypeRef& ChanBufferType,
                             const ExpT& CountExp);
            ~ChanBufferSorter();

            void Sort(StateVec* OutStateVector);
        };

        class Canonicalizer
        {
        private:
            vector<PermuterBase*> Permuters;
            vector<ChanBufferSorter*> Sorters;
            PermutationSet* PermSet;

        public:
            Canonicalizer(const LabelledTS* TheLTS);
            ~Canonicalizer();
            
            StateVec* Canonicalize(const StateVec* InputVector, u32& PermID) const;
            StateVec* ApplyPermutation(const StateVec* InputVector, u32 PermID) const;
            StateVec* ApplyInvPermutation(const StateVec* InputVector, u32 PermID) const;
        };
        
    } /* end namespace Symm */
} /* end namespace ESMC */

#endif /* ESMC_SYMM_CANONICALIZER_HPP_ */

// 
// SymmCanonicalizer.hpp ends here