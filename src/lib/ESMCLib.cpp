// ESMCLib.cpp ---
// Filename: ESMCLib.cpp
// Author: Abhishek Udupa
// Created: Mon Jan 19 23:56:13 2015 (-0500)
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

#include <stdlib.h>
#include <string.h>

#include "../utils/LogManager.hpp"

#include "ESMCLib.hpp"

namespace ESMC {

ESMCLibOptionsT::ESMCLibOptionsT()
    : LogFileName(""),
      LogCompressionTechnique(LogFileCompressionTechniqueT::COMPRESS_NONE),
      LoggingOptions()
{
    // Nothing here
}

ESMCLibOptionsT::~ESMCLibOptionsT()
{
    // Nothing here
}

ESMCLibOptionsT::ESMCLibOptionsT(const ESMCLibOptionsT& Other)
    : LogFileName(Other.LogFileName),
      LogCompressionTechnique(Other.LogCompressionTechnique),
      LoggingOptions(Other.LoggingOptions)
{
    // Nothing here
}

ESMCLibOptionsT& ESMCLibOptionsT::operator = (const ESMCLibOptionsT& Other)
{
    if (&Other == this) {
        return *this;
    }
    LogFileName = Other.LogFileName;
    LogCompressionTechnique = Other.LogCompressionTechnique;
    LoggingOptions = Other.LoggingOptions;
    return *this;
}

ESMCLibOptionsT& ESMCLib::ESMCLibOptions()
{
    static ESMCLibOptionsT ESMCLibOptions_;
    return ESMCLibOptions_;
}

ESMCLib::ESMCLib()
{
    // Nothing here
}

void ESMCLib::Initialize()
{
    Logging::LogManager::Initialize();
}

void ESMCLib::Initialize(const ESMCLibOptionsT& LibOptions)
{
    ESMCLibOptions() = LibOptions;
    Logging::LogManager::Initialize(ESMCLibOptions().LogFileName,
                                    ESMCLibOptions().LogCompressionTechnique);
    Logging::LogManager::EnableLogOptions(ESMCLibOptions().LoggingOptions.begin(),
                                          ESMCLibOptions().LoggingOptions.end());
}

void ESMCLib::Finalize()
{
    Logging::LogManager::Finalize();
}

// The library initializer
__attribute__((constructor)) void ESMCLibInitialize_()
{
    ESMCLib::Initialize();
}

__attribute__((destructor)) void ESMCLibFinalize_()
{
    ESMCLib::Finalize();
}

} /* end namespace ESMC */

//
// ESMCLib.cpp ends here
