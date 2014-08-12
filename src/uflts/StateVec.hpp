// State.hpp --- 
// 
// Filename: State.hpp
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

#if !defined ESMC_STATE_HPP_
#define ESMC_STATE_HPP_

#include "../common/FwdDecls.hpp"

namespace ESMC {
    namespace LTS {

        class State 
        {
        private:
            u08* StateBuffer;
            u64 Size;
            u64 HashCode;

        public:
            State(u64 Size);
            ~State();

            u16 ReadShort(u32 Offset) const;
            void WriteShort(u32 Offset, u16 Value);
            
            u32 ReadWord(u32 Offset) const;
            void WriteWord(u32 Offset, u32 Value);

            u08& operator [] (u32 Offset);
            const u08& operator [] (u32 Offset) const;

            void Freeze();
            u64 Hash() const;
        };

        class StateFactory
        {
        private:
            const StateInterpretation* Interpretation;
            const u32 StateSize;

        public:
            StateFactory(const StateInterpretation* Interpretation, u32 StateSize);
            State* Make() const;
        };

        class InterpretationBase
        {
        protected:
            u32 ObjectSize;
            u32 ObjectOffset;
            ExprTypeRef ObjectType;

        public:
            InterpretationBase(u32 ObjectSize, u32 ObjectOffset);
            virtual ~InterpretationBase();

            u32 GetObjectSize() const;
            u32 GetObjectOffset() const;
            const ExprTypeRef& GetObjectType() const;
            virtual string ObjectToString(const State* StatePtr) const = 0;
            virtual u32 ObjectToInt(const State* StatePtr) const = 0;
        };

        class ScalarInterpretation
        {
        public:
            
        };

        class StateInterpretation
        {
            
        };

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_STATE_HPP_ */

// 
// State.hpp ends here
