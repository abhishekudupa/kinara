// StateVec.cpp --- 
// 
// Filename: StateVec.cpp
// Author: Abhishek Udupa
// Created: Wed Aug 13 20:19:02 2014 (-0400)
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

#include "StateVec.hpp"
#include "../hash/SpookyHash.hpp"

namespace ESMC {
    namespace MC {

        inline StateVec::StateVec(const StateVec* Other)
            : StateBuffer(Other->Factory->GetStateBuffer(false)),
              Factory(Other->Factory)
        {
            memcpy(StateBuffer, Other->StateBuffer, Other->GetSize());
        }

        StateVec::StateVec(StateFactory* Factory)
            : StateBuffer(Factory->GetStateBuffer()),
              Factory(Factory)
        {
            // Nothing here
        }

        StateVec::~StateVec()
        {
            Factory->ReleaseStateBuffer(StateBuffer);
        }

        u08 StateVec::ReadByte(u32 Offset) const
        {
            return StateBuffer[Offset];
        }

        void StateVec::WriteByte(u32 Offset, u08 Value)
        {
            StateBuffer[Offset] = Value;
        }

        u16 StateVec::ReadShort(u32 Offset) const
        {
            u16* ActPointer = (u16*)(StateBuffer + Offset);
            return *ActPointer;
        }

        void StateVec::WriteShort(u32 Offset, u16 Value)
        {
            u16* ActPointer = (u16*)(StateBuffer + Offset);
            *ActPointer = Value;
        }

        u32 StateVec::ReadWord(u32 Offset) const
        {
            u32* ActPointer = (u32*)(StateBuffer + Offset);
            return *ActPointer;
        }

        void StateVec::WriteWord(u32 Offset, u32 Value)
        {
            u32* ActPointer = (u32*)(StateBuffer + Offset);
            *ActPointer = Value;
        }

        u08 StateVec::ReadByteMsg(u32 Offset) const
        {
            auto StateBuffer = Factory->GetMsgBuffer();
            return StateBuffer[Offset];
        }

        void StateVec::WriteByteMsg(u32 Offset, u08 Value)
        {
            auto StateBuffer = Factory->GetMsgBuffer();
            StateBuffer[Offset] = Value;
        }

        u16 StateVec::ReadShortMsg(u32 Offset) const
        {
            auto StateBuffer = Factory->GetMsgBuffer();
            u16* ActPointer = (u16*)(StateBuffer + Offset);
            return *ActPointer;
        }

        void StateVec::WriteShortMsg(u32 Offset, u16 Value)
        {
            auto StateBuffer = Factory->GetMsgBuffer();
            u16* ActPointer = (u16*)(StateBuffer + Offset);
            *ActPointer = Value;
        }

        u32 StateVec::ReadWordMsg(u32 Offset) const
        {
            auto StateBuffer = Factory->GetMsgBuffer();
            u32* ActPointer = (u32*)(StateBuffer + Offset);
            return *ActPointer;
        }

        void StateVec::WriteWordMsg(u32 Offset, u32 Value)
        {
            auto StateBuffer = Factory->GetMsgBuffer();
            u32* ActPointer = (u32*)(StateBuffer + Offset);
            *ActPointer = Value;
        }        

        u08& StateVec::operator [] (u32 Offset)
        {
            return StateBuffer[Offset];
        }

        const u08& StateVec::operator [] (u32 Offset) const
        {
            return StateBuffer[Offset];
        }

        u32 StateVec::GetSize() const
        {
            return Factory->GetSize();
        }

        u32 StateVec::GetMsgSize() const
        {
            return Factory->GetMsgSize();
        }

        bool StateVec::Equals(const StateVec& Other) const
        {
            return ((Factory == Other.Factory) &&
                    (memcmp(StateBuffer, Other.StateBuffer, GetSize()) == 0));
        }

        i32 StateVec::Compare(const StateVec& Other) const
        {
            return (memcmp(StateBuffer, Other.StateBuffer, GetSize()));
        }

        void StateVec::Set(const StateVec& Other) 
        {
            memcpy(StateBuffer, Other.StateBuffer, GetSize());
        }

        u32 StateVec::Hash() const
        {
            return SpookyHash::SpookyHash::Hash32(StateBuffer, GetSize(), 
                                                  0xBEADFEED);
        }

        StateVec* StateVec::Clone() const
        {
            return Factory->MakeState(this);
        }

        u08* StateVec::GetStateBuffer()
        {
            return StateBuffer;
        }

        const u08* StateVec::GetStateBuffer() const
        {
            return StateBuffer;
        }

        StateFactory* StateVec::GetFactory() const
        {
            return Factory;
        }

        u08* StateVec::GetMsgBuffer()
        {
            return Factory->GetMsgBuffer();
        }

        const u08* StateVec::GetMsgBuffer() const
        {
            return Factory->GetMsgBuffer();
        }

        void StateVec::ClearMsgBuffer()
        {
            Factory->ClearMsgBuffer();
        }

        StateFactory::StateFactory(u32 StateSize, u32 MsgSize)
            : StateSize(StateSize),
              StateVecPool(new boost::pool<>(sizeof(StateVec))),
              StateVecBufferPool(new boost::pool<>(StateSize)),
              NumActiveStates(0),
              MsgSize(MsgSize),
              MsgBuffer(new u08[MsgSize])
        {
            // Nothing here
        }

        StateFactory::~StateFactory()
        {
            delete StateVecPool;
            delete StateVecBufferPool;
            delete[] MsgBuffer;
        }

        void StateFactory::ClearMsgBuffer()
        {
            memset(MsgBuffer, 0, MsgSize);
        }
        
        u08* StateFactory::GetMsgBuffer()
        {
            return MsgBuffer;
        }

        StateVec* StateFactory::MakeState()
        {
            auto Retval = new (StateVecPool->malloc()) StateVec(this);
            NumActiveStates++;
            return Retval;
        }

        void StateFactory::TakeState(const StateVec* StatePtr)
        {
            StatePtr->~StateVec();
            StateVecPool->free((void*)StatePtr);
            NumActiveStates--;
        }

        u08* StateFactory::GetStateBuffer(bool Clear)
        {
            void* Retval = StateVecBufferPool->malloc();
            if (Clear) {
                memset(Retval, 0, StateSize);
            }
            return (u08*)Retval;
        }

        void StateFactory::ReleaseStateBuffer(u08* BufferPtr)
        {
            StateVecBufferPool->free(BufferPtr);
            return;
        }

        StateVec* StateFactory::MakeState(const StateVec* Other)
        {
            auto Retval = new (StateVecPool->malloc()) StateVec(Other);
            NumActiveStates++;
            return Retval;
        }

        u32 StateFactory::GetSize() const
        {
            return StateSize;
        }

        u32 StateFactory::GetMsgSize() const
        {
            return MsgSize;
        }

        u32 StateFactory::GetNumActiveStates() const
        {
            return NumActiveStates;
        }

    } /* end namespace MC */
} /* end namespace ESMC */

// 
// StateVec.cpp ends here
