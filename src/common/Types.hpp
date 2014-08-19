// Types.hpp --- 
// 
// Filename: Types.hpp
// Author: Abhishek Udupa
// Created: Sun Jun 29 13:43:04 2014 (-0400)
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

// Common types used throughout the solver

#if !defined ESMC_TYPES_HPP_
#define ESMC_TYPES_HPP_

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <inttypes.h>
#include <exception>

#ifndef BOOST_SYSTEM_NO_DEPRECATED
#define BOOST_SYSTEM_NO_DEPRECATED 1
#endif

using namespace std;

namespace ESMC {
    typedef int8_t i08;
    
    typedef uint8_t u08;
    typedef int16_t i16;
    typedef uint16_t u16;
    typedef int32_t i32;
    typedef uint32_t u32;
    typedef int64_t i64;
    typedef uint64_t u64;

    class InternalError : public exception
    {
    private:
        string ErrorMsg;

    public:
        
        inline InternalError(const string& ErrorMsg) 
            : ErrorMsg((string)"InternalError: " + ErrorMsg) {}
        inline virtual ~InternalError() throw() {}
        inline virtual const char* what() const throw() override { return ErrorMsg.c_str(); }
    };

    class ESMCError : public exception
    {
    private:
        string ErrorMsg;
        
    public:
        inline ESMCError(const string& ErrorMsg)
            : ErrorMsg(ErrorMsg)
        {
            // Nothing here
        }

        inline virtual ~ESMCError() throw ()
        {
            // Nothing here
        }

        inline virtual const char* what() const throw() override { return ErrorMsg.c_str(); }
    };

    class UnimplementedException : public exception
    {
    private:
        string MethodName;
        string FileName;
        u32 LineNum;
        string ErrMsg;
    public:
        inline UnimplementedException(const string& MethodName,
                                      const string& FileName,
                                      u32 LineNum)
            : MethodName(MethodName), FileName(FileName),
              LineNum(LineNum)
        {
            ErrMsg = (string)"Unimplemented method: " + MethodName + 
                (string)", at " + FileName + (string)":" + 
                to_string(LineNum);
        }

        inline virtual ~UnimplementedException()
        {
            // Nothing here
        }

        inline virtual const char* what() const throw() override
        {
            return ErrMsg.c_str();
        }
    };

} /* end namespace ESMC */
    
#endif /* ESMC_TYPES_HPP_ */

// 
// Types.hpp ends here
