// LTSState.cpp ---
//
// Filename: LTSState.cpp
// Author: Abhishek Udupa
// Created: Fri Aug  8 13:47:45 2014 (-0400)
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

#include "LTSState.hpp"

namespace ESMC {
    namespace LTS {

        LTSState::LTSState()
        {
            // Nothing here
        }

        LTSState::LTSState(const string& StateName, bool Initial,
                           bool Final, bool Accepting, bool Error)
            : StateName(StateName), Accepting(Accepting),
              Final(Final), Error(Error), Initial(Initial)
        {
            // Nothing here
        }

        LTSState::LTSState(const LTSState& Other)
            : StateName(Other.StateName),
              Accepting(Other.Accepting),
              Final(Other.Final),
              Error(Other.Error),
              Initial(Other.Initial)
        {
            // Nothing here
        }

        LTSState::~LTSState()
        {
            // Nothing here
        }

        LTSState& LTSState::operator = (const LTSState& Other)
        {
            if (&Other == this) {
                return *this;
            }
            StateName = Other.StateName;
            Accepting = Other.Accepting;
            Final = Other.Final;
            Error = Other.Error;
            Initial = Other.Initial;
            return *this;
        }

        string LTSState::GetName() const
        {
            return StateName;
        }

        bool LTSState::IsAccepting() const
        {
            return Accepting;
        }

        bool LTSState::IsFinal() const
        {
            return Final;
        }

        bool LTSState::IsError() const
        {
            return Error;
        }

        bool LTSState::IsInitial() const
        {
            return Initial;
        }

        string LTSState::ToString() const
        {
            ostringstream sstr;
            sstr << StateName << " : "
                 << (Accepting ? "Accepting, " : "")
                 << (Final ? "Final, " : "")
                 << (Error ? "Error, " : "")
                 << (Initial ? "Initial" : "");
            return sstr.str();
        }

        LTSState LTSState::MakeEFSMState(const string& StateName,
                                         bool Initial,
                                         bool Final,
                                         bool Error)
        {
            LTSState Retval;
            Retval.StateName = StateName;
            Retval.Initial = Initial;
            Retval.Final = Final;
            Retval.Error = Error;
            Retval.Accepting = false;
            return Retval;
        }

        LTSState LTSState::MakeMonitorState(const string& StateName,
                                            bool Initial,
                                            bool Accepting)
        {
            LTSState Retval;
            Retval.StateName = StateName;
            Retval.Initial = Initial;
            Retval.Accepting = Accepting;
            Retval.Error = false;
            Retval.Final = false;
            return Retval;
        }


    } /* end namespace ESMC */
} /* end namespace ESMC */

//
// LTSState.cpp ends here
