// StateVec.hpp ---
//
// Filename: StateVec.hpp
// Author: Abhishek Udupa
// Created: Mon Jul 28 08:34:10 2014 (-0400)
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

#if !defined ESMC_STATE_VEC_HPP_
#define ESMC_STATE_VEC_HPP_

#include <boost/pool/pool.hpp>

#include "../common/ESMCFwdDecls.hpp"

namespace ESMC {
    namespace MC {

        class StateFactory;

        namespace Detail {
            struct StateVecHashStructT
            {
                bool HashValid : 1;
                u64 HashCode : 63;

                inline StateVecHashStructT()
                    : HashValid(false), HashCode(0)
                {
                    // Nothing here
                }

                inline StateVecHashStructT(u64 HashCode)
                    : HashValid(true), HashCode(HashCode)
                {
                    // Nothing here
                }

                inline StateVecHashStructT(const StateVecHashStructT& Other)
                    : HashValid(Other.HashValid), HashCode(Other.HashCode)
                {
                    // Nothing here
                }

                inline ~StateVecHashStructT()
                {
                    // Nothing here
                }
            };
        } /* end namespace Detail */

        class StateVec
        {
            friend class StateFactory;

        private:
            u08* StateBuffer;
            StateFactory* Factory;
            mutable Detail::StateVecHashStructT HashFields;

            inline StateVec(const StateVec* Other);

        public:
            StateVec(StateFactory* Factory);
            ~StateVec();

            u08 ReadByte(u32 Offset) const;
            void WriteByte(u32 Offset, u08 Value);

            u16 ReadShort(u32 Offset) const;
            void WriteShort(u32 Offset, u16 Value);

            u32 ReadWord(u32 Offset) const;
            void WriteWord(u32 Offset, u32 Value);

            u32 GetSize() const;
            bool Equals(const StateVec& Other) const;
            i32 Compare(const StateVec& Other) const;
            u64 Hash() const;
            StateVec* Clone() const;
            u08* GetStateBuffer();
            const u08* GetStateBuffer() const;
            StateFactory* GetFactory() const;
            void Set(const StateVec& Other);
            void Recycle() const;
            void MarkDirty() const;
        };

        class StateFactory
        {
            friend class StateVec;

        private:
            const u32 StateSize;
            boost::pool<>* StateVecPool;
            boost::pool<>* StateVecBufferPool;
            u32 NumActiveStates;

            u08* GetStateBuffer(bool Clear = true);
            void ReleaseStateBuffer(u08* BufferPtr);
            StateVec* MakeState(const StateVec* Other);

        public:
            StateFactory(u32 StateSize);
            ~StateFactory();
            StateVec* MakeState();
            void TakeState(const StateVec* StatePtr);
            u32 GetSize() const;
            u32 GetNumActiveStates() const;
        };

    } /* end namespace MC */
} /* end namespace ESMC */

#endif /* ESMC_STATE_VEC_HPP_ */

//
// StateVec.hpp ends here
