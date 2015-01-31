// ESMCLib.hpp ---
// Filename: ESMCLib.hpp
// Author: Abhishek Udupa
// Created: Mon Jan 19 23:54:40 2015 (-0500)
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

#if !defined ESMC_ESMC_LIB_HPP_
#define ESMC_ESMC_LIB_HPP_

#include <set>

#include "../common/ESMCFwdDecls.hpp"

namespace ESMC {

class ESMCLibOptionsT
{
public:
    string LogFileName;
    LogFileCompressionTechniqueT LogCompressionTechnique;
    set<string> LoggingOptions;

    ESMCLibOptionsT();
    virtual ~ESMCLibOptionsT();

    ESMCLibOptionsT(const ESMCLibOptionsT& Other);
    ESMCLibOptionsT& operator = (const ESMCLibOptionsT& Other);
};

class ESMCLib
{
private:
    static ESMCLibOptionsT& ESMCLibOptions();

    ESMCLib();
    ESMCLib(const ESMCLib& Other) = delete;
    ESMCLib(ESMCLib&& Other) = delete;

public:
    static void Initialize();
    static void Initialize(const ESMCLibOptionsT& LibOptions);
    static void Finalize();
};

__attribute__((constructor)) extern void ESMCLibInitialize_();
__attribute__((destructor)) extern void ESMCLibFinalize_();

} /* end namespace ESMC */

#endif /* ESMC_ESMC_LIB_HPP_ */

//
// ESMCLib.hpp ends here
