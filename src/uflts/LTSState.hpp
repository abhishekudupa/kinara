// LTSState.hpp ---
//
// Filename: LTSState.hpp
// Author: Abhishek Udupa
// Created: Fri Aug  8 13:43:46 2014 (-0400)
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

#if !defined ESMC_LTS_STATE_HPP_
#define ESMC_LTS_STATE_HPP_

#include "../common/ESMCFwdDecls.hpp"

namespace ESMC {
namespace LTS {

class LTSState : public Stringifiable
{
    friend class AutomatonBase;
private:
    string StateName;
    bool Accepting;
    bool Final;
    bool Error;
    bool Initial;

private:
    LTSState(const string& StateName, bool Initial,
             bool Final, bool Accepting, bool Error);

public:
    LTSState(const LTSState& Other);
    LTSState();
    ~LTSState();

    LTSState& operator = (const LTSState& Other);

    string GetName() const;
    bool IsAccepting() const;
    bool IsFinal() const;
    bool IsError() const;
    bool IsInitial() const;
    virtual string ToString(u32 Verbosity = 0) const override;

    static LTSState MakeEFSMState(const string& StateName,
                                  bool Initial = false,
                                  bool Final = false,
                                  bool Error = false);

    static LTSState MakeMonitorState(const string& StateName,
                                     bool Initial = false,
                                     bool Accepting = false);
};

} /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LTS_STATE_HPP_ */

//
// LTSState.hpp ends here
