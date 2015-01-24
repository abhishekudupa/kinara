// BitSet.hpp ---
//
// Filename: BitSet.hpp
// Author: Abhishek Udupa
// Created: Fri Jan 23 15:00:38 2015 (-0500)
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

#if !defined ESMC_BIT_SET_HPP_
#define ESMC_BIT_SET_HPP_

#include <string.h>
#include <utility>

#include "../common/ESMCFwdDecls.hpp"

namespace ESMC {

    // An efficient fixed size bit set
    // the size cannot be changed after
    // construction

    class BitSet
    {
    private:
        u32 NumBits;
        u08* BitArray;

        inline i32 Compare(const BitSet& Other) const;

    public:
        class BitRef
        {
            BitSet* TheBitSet;
            u32 BitNum;

            inline BitRef(BitSet* TheBitSet, u32 BitNum);
            inline BitRef(const BitRef& Other);
            inline ~BitRef();

            inline BitRef& operator = (const BitRef& Other);
            inline BitRef& operator = (bool Value);

            inline bool operator == (const BitRef& Other) const;
            inline bool operator != (const BitRef& Other) const;

            inline bool operator == (bool Value) const;
            inline bool operator != (bool Value) const;

            inline operator bool () const;
            inline bool operator ! () const;
        };

        inline BitSet();
        inline BitSet(u32 Size);
        inline BitSet(u32 Size, bool InitialValue);
        inline BitSet(const BitSet& Other);
        inline BitSet(BitSet&& Other);

        inline ~BitSet();

        inline BitSet& operator = (const BitSet& Other);
        inline BitSet& operator = (const BitSet&& Other);

        inline bool operator == (const BitSet& Other) const;
        inline bool operator < (const BitSet& Other) const;
        inline bool operator > (const BitSet& Other) const;
        inline bool operator <= (const BitSet& Other) const;
        inline bool operator >= (const BitSet& Other) const;
        inline bool operator != (const BitSet& Other) const;

        inline void Set(u32 BitNum);
        inline bool Test(u32 BitNum) const;
        inline void Clear(u32 BitNum);
        // returns the value of bit before flip
        inline bool Flip(u32 BitNum);

        // Gang set, clear and flip
        inline void Set();
        inline void Clear();
        inline void Flip();

        inline bool operator [] (u32 BitNum) const;
        inline BitRef operator [] (u32 BitNum);

        inline u32 Size() const;
        inline string ToString() const;
    };

    // Implementation of BitRef
    inline BitSet::BitRef::BitRef(BitSet* TheBitSet, u32 BitNum)
        : TheBitSet(TheBitSet), BitNum(BitNum)
    {
        // Nothing here
    }

    inline BitSet::BitRef::BitRef(const BitRef& Other)
        : BitRef(Other.TheBitSet, Other.BitNum)
    {
        // Nothing here
    }

    inline BitSet::BitRef::~BitRef()
    {
        TheBitSet = nullptr;
        BitNum = 0;
    }

    inline BitSet::BitRef& BitSet::BitRef::operator = (const BitRef& Other)
    {
        if (&Other == this) {
            return *this;
        }
        TheBitSet = Other.TheBitSet;
        BitNum = Other.BitNum;
    }

    inline BitSet::BitRef& BitSet::BitRef::operator = (bool Value)
    {
        if (Value) {
            TheBitSet->Set(BitNum);
        } else {
            TheBitSet->Clear(BitNum);
        }
        return *this;
    }

    inline bool BitSet::BitRef::operator == (const BitRef& Other) const
    {
        return ((bool)(*this) == (bool)(Other));
    }

    inline bool BitSet::BitRef::operator != (const BitRef& Other) const
    {
        return ((bool)(*this) != (bool)(Other));
    }

    inline bool BitSet::BitRef::operator == (bool Value) const
    {
        return ((bool)(*this) == Value);
    }

    inline bool BitSet::BitRef::operator != (bool Value) const
    {
        return ((bool)(*this) != Value);
    }

    inline BitSet::BitRef::operator bool() const
    {
        if (TheBitSet == nullptr) {
            return false;
        }
        return TheBitSet->Test(BitNum);
    }

    inline bool BitSet::BitRef::operator ! () const
    {
        return (!((bool)(*this)));
    }

    // implementation of BitSet
    inline BitSet::BitSet()
        : NumBits(0), BitArray(nullptr)
    {
        // Nothing here
    }

    inline BitSet::BitSet(u32 Size)
        : NumBits(Size)
    {
        u32 NumBytes = (Size + 7) / 8;
        BitArray = (u08*)calloc(sizeof(u08), NumBytes);
    }

    inline BitSet::BitSet(u32 Size, bool InitialValue)
        : BitSet(Size)
    {
        if (InitialValue) {
            memset(BitArray, Size / 8, 0xFF);
            // take care of the last few bits.
            if (Size % 8 != 0) {
                u32 Mask = 0;
                for (u32 i = 0; i < Size % 8; ++i) {
                    Mask <<= 1;
                    Mask += 1;
                }
                BitArray[Size / 8] |= Mask;
            }
        }
    }

    inline BitSet::BitSet(const BitSet& Other)
        : NumBits(Other.NumBits)
    {
        u32 NumBytes = (NumBits + 7) / 8;
        BitArray = (u08*)malloc(sizeof(u08) * NumBytes);
        memcpy(BitArray, Other.BitArray, NumBytes);
    }

    inline BitSet::BitSet(BitSet&& Other)
        : NumBits(0), BitArray(nullptr)
    {
        swap(NumBits, Other.NumBits);
        swap(BitArray, Other.BitArray);
    }

    inline BitSet::~BitSet()
    {
        free(BitArray);
    }

    inline BitSet& BitSet::operator = (const BitSet& Other)
    {
        if (&Other == this) {
            return *this;
        }

        free(BitArray);

        NumBits = Other.NumBits;
        u32 NumBytes = (NumBits + 7) / 8;
        BitArray = (u08*)malloc(sizeof(u08) * NumBytes);
        memcpy(BitArray, Other.BitArray, NumBytes);
        return *this;
    }

    inline BitSet& BitSet::operator = (BitSet&& Other)
    {
        swap(NumBits, Other.NumBits);
        swap(BitArray, Other.BitArray);
    }

    inline i32 BitSet::Compare(const BitSet& Other) const
    {
        i32 Diff = NumBits - Other.NumBits;
        if (Diff != 0) {
            return Diff;
        }
        return memcmp(BitArray, Other.BitArray, (NumBits + 7) / 8);
    }

    inline bool BitSet::operator == (const BitSet& Other) const
    {
        return (Compare(Other) == 0);
    }

    inline bool BitSet::operator < (const BitSet& Other) const
    {
        return (Compare(Other) < 0);
    }

    inline bool BitSet::operator > (const BitSet& Other) const
    {
        return (Compare(Other) > 0);
    }

    inline bool BitSet::operator <= (const BitSet& Other) const
    {
        return (Compare(Other) <= 0);
    }

    inline bool BitSet::operator >= (const BitSet& Other) const
    {
        return (Compare(Other) >= 0);
    }

    inline bool BitSet::operator != (const BitSet& Other) const
    {
        return (Compare(Other) != 0);
    }

    inline void BitSet::Set(u32 BitNum)
    {
        u32 Offset = BitNum / 8;
        const u32 BitNumMod8 = BitNum % 8;
        u08 Mask = 0x80;
        if (BitNumMod8 != 0) {
            Mask >>= (BitNumMod8);
        }
        BitArray[Offset] |= Mask;
    }

    inline void BitSet::Clear(u32 BitNum)
    {
        u32 Offset = BitNum / 8;
        const u32 BitNumMod8 = BitNum % 8;
        u08 Mask = 0x80;
        if (BitNumMod8 != 0) {
            Mask >>= (BitNumMod8);
        }
        BitArray[Offset] &= (~Mask);
    }

    inline bool BitSet::Test(u32 BitNum) const
    {
        u32 Offset = BitNum / 8;
        const u32 BitNumMod8 = BitNum % 8;
        u08 Mask = 0x80;
        if (BitNumMod8 != 0) {
            Mask >>= (BitNumMod8);
        }
        return (((BitArray[Offset] & Mask) == 0) ? false : true);
    }

    inline bool BitSet::Flip(u32 BitNum)
    {
        u32 Offset = BitNum / 8;
        const u32 BitNumMod8 = BitNum % 8;
        u08 Mask = 0x80;
        if (BitNumMod8 != 0) {
            Mask >>= (BitNumMod8);
        }
        bool Retval = (((BitArray[Offset] & Mask) == 0) ? false : true);
        if (Retval) {
            BitArray[Offset] &= (~Mask);
        } else {
            BitArray[Offset] |= Mask;
        }
        return Retval;
    }

    inline void BitSet::Set()
    {
        memset(BitArray, NumBits / 8, 0xFF);
        // take care of the last few bits.
        if (NumBits % 8 != 0) {
            u32 Mask = 0;
            for (u32 i = 0; i < NumBits % 8; ++i) {
                Mask <<= 1;
                Mask += 1;
            }
            BitArray[NumBits / 8] |= Mask;
        }
    }

    inline void BitSet::Clear()
    {
        memset(BitArray, (NumBits + 7) / 8, 0);
    }

    inline void BitSet::Flip()
    {
        for (u32 i = 0; i < NumBits / 8; ++i) {
            BitArray[i] = (~(BitArray[i]));
        }
        // take care of the last few bits with
        // explicit calls
        for (u32 i = 0; i < NumBits % 8; ++i) {
            Flip(NumBits / 8 + i);
        }
    }

    inline bool BitSet::operator [] (u32 BitNum) const
    {
        return Test(BitNum);
    }

    inline BitSet::BitRef BitSet::operator [] (u32 BitNum)
    {
        return BitRef(this, BitNum);
    }

    inline u32 BitSet::Size() const
    {
        return NumBits;
    }

    inline string BitSet::ToString() const
    {
        ostringstream sstr;
        sstr << "{" << endl;
        for (u32 i = 0; i < NumBits; ++i) {
            if (Test(i)) {
                sstr << " " << i << " -> 1";
            } else {
                sstr << " " << i << " -> 0";
            }
        }
        sstr << " }";
        return sstr.str();
    }

} /* end namespace ESMC */


#endif /* ESMC_BIT_SET_HPP_ */

//
// BitSet.hpp ends here
