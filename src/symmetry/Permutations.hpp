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
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <iterator>

#include "../common/ESMCFwdDecls.hpp"

namespace ESMC {
namespace Symm {

namespace Detail {
class PermVecHasher
{
public:
    inline u64 operator () (const vector<u08>& PermVec) const
    {
        u64 Retval = 0;
        for (auto const& Elem : PermVec) {
            boost::hash_combine(Retval, Elem);
        }
        return Retval;
    }
};
} /* end namespace Detail */


static inline string PermToString(const vector<u08>& Perm)
{
    ostringstream sstr;
    sstr << "< ";
    for (auto const& Elem : Perm) {
        sstr << (u32)Elem << " ";
    }
    sstr << ">";
    return sstr.str();
}

class DomainPermuter
{
private:
    static const u32 MaxExplicitSize;

    u32 DomainSize;
    u32 PermSize;
    u32 Offset;
    vector<u08> IdentityPerm;
    bool Compact;
    const u32 DomMinusOneFactorial;
    mutable vector<u08> CachedPerm;
    mutable vector<u08> CachedInvPerm;
    vector<vector<u08>> Permutations;
    unordered_map<vector<u08>, u32, Detail::PermVecHasher> PermToIdxMap;
    vector<u32> PermIdxToInvPermIdx;

    inline void GetPermForLehmerCode(u32 Code, vector<u08>& OutPerm) const;
    inline u32 GetLehmerCodeForPerm(const vector<u08>& Perm) const;
    inline void InvertPerm(const vector<u08>& Perm, vector<u08>& OutPerm) const;

public:
    DomainPermuter(u32 DomainSize, u32 Offset,
                   bool Compact = false);
    ~DomainPermuter();

    // Accessors
    u32 GetDomainSize() const;
    u32 GetOffset() const;
    const vector<u08>& GetIdentityPerm() const;
    u32 GetPermSize() const;

    const vector<u08>& GetPerm(u32 Idx) const;
    const vector<u08>& GetInvPerm(u32 Idx) const;
    const vector<u08>& GetInvPerm(const vector<u08>& Perm) const;
    void GetInvPerm(const vector<u08>& Perm,
                    vector<u08>& OutPerm) const;

    void GetPerm(u32 Idx, vector<u08>& OutPerm) const;
    void GetInvPerm(u32 Idx, vector<u08>& OutPerm) const;
    u32 GetPermIdx(const vector<u08>& Perm) const;
    u32 GetInvPermIdx(const vector<u08>& Perm) const;
    u32 GetInvPermIdx(u32 Idx) const;
};


class PermutationSet
{
public:
    class iterator
    {
        friend class PermutationSet;
    private:
        PermutationSet* PermSet;
        u32 Index;

    public:
        iterator();
        iterator(const iterator& Other);
        iterator(u32 Index, PermutationSet* PermSet);
        ~iterator();

        iterator& operator ++ ();
        iterator operator ++ (int Dummy);

        iterator& operator -- ();
        iterator operator -- (int Dummy);

        iterator& operator += (u32 Addend);
        iterator& operator -= (u32 Addend);

        iterator operator + (u32 Addend) const;
        iterator operator - (u32 Addend) const;

        const vector<u08>& GetPerm() const;
        u32 GetIndex() const;

        bool operator == (const iterator& Other) const;
        bool operator != (const iterator& Other) const;
        iterator& operator = (const iterator& Other);
    };

    friend class iterator;

private:
    vector<u32> DomainSizes;
    vector<DomainPermuter*> DomPermuters;
    vector<u32> Multipliers;
    const u32 NumDomains;
    iterator BeginIterator;
    iterator EndIterator;
    u32 Size;
    u32 PermVecSize;
    u32 CachedIdx;
    vector<u32> CachedIndices;
    vector<u08> CachedPerm;


public:
    PermutationSet(const vector<u32>& DomainSizes, bool Compact);
    ~PermutationSet();

    u32 GetSize() const;
    u32 GetPermVecSize() const;

    iterator GetIterator(u32 Idx) const;
    iterator GetIteratorForInv(u32 Idx) const;
    iterator Compose(const iterator& Perm, u32 Idx);
    iterator Compose(u32 PermIdx, u32 Idx);

    const iterator& Begin() const;
    const iterator& End() const;

    void GetPermForIndex(u32 Index);
    void GetPermForIndex(u32 Index, vector<u08>& OutVec) const;

    u32 GetIndexForPerm(const vector<u08>& Perm) const;

    void Print(u32 PermIdx, ostream& Out) const;
};

} /* end namespace Symm */
} /* end namespace ESMC */

#endif /* ESMC_PERMUTATIONS_HPP_ */

//
// Permutations.hpp ends here
