// ESMCESMCTypes.hpp ---
//
// Filename: ESMCESMCTypes.hpp
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

#if !defined ESMC_ESMC_ESMCTYPES_HPP_
#define ESMC_ESMC_ESMCTYPES_HPP_

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <inttypes.h>
#include <exception>
#include <functional>

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
#ifdef __APPLE__
typedef size_t u64;
#else
typedef uint64_t u64;
#endif

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

    inline const string& GetFileName() const
    {
        return FileName;
    }

    inline u32 GetLineNum() const
    {
        return LineNum;
    }

    inline const string& GetMethodName() const
    {
        return MethodName;
    }
};

// Base class for all stringifiable classes
class Stringifiable
{
public:
    inline Stringifiable() {}
    inline virtual ~Stringifiable() {}

    virtual string ToString(u32 Verbosity) const = 0;
    inline operator string () const
    {
        return ToString();
    }

    inline string ToString() const
    {
        return ToString(0);
    }
};

static inline ostream& operator << (ostream& Out, const Stringifiable& Obj)
{
    Out << Obj.ToString();
    return Out;
}

// Utility classes for hashing and comparing stringifiable objects

template <u32 VERBOSITY = 0>
class StringifiablePtrHasher
{
public:
    template <typename T>
    inline u64 operator () (const T& ObjPtr) const
    {
        std::hash<string> StringHasher;
        return StringHasher(ObjPtr->ToString(VERBOSITY));
    }
};

template <u32 VERBOSITY = 0>
class StringifiableObjHasher
{
public:
    template <typename U>
    inline u64 operator () (const U& Obj) const
    {
        std::hash<string> StringHasher;
        return StringHasher(Obj.ToString(VERBOSITY));
    }
};

template <u32 VERBOSITY = 0>
class StringifiablePtrCompare
{
public:
    template <typename T>
    inline i64 operator () (const T& ObjPtr1, const T& ObjPtr2) const
    {
        auto&& Str1 = ObjPtr1->ToString(VERBOSITY);
        auto&& Str2 = ObjPtr2->ToString(VERBOSITY);
        return (Str1 < Str2);
    }
};

template <u32 VERBOSITY = 0>
class StringifiableObjCompare
{
public:
    template <typename T>
    inline i64 operator () (const T& Obj1, const T& Obj2) const
    {
        auto&& Str1 = Obj1.ToString(VERBOSITY);
        auto&& Str2 = Obj2.ToString(VERBOSITY);
        return (Str1 < Str2);
    }
};

template <u32 VERBOSITY = 0>
class StringifiablePtrEquals
{
public:
    template <typename T>
    inline bool operator () (const T& ObjPtr1, const T& ObjPtr2) const
    {
        return (ObjPtr1->ToString(VERBOSITY) == ObjPtr2->ToString(VERBOSITY));
    }
};

template <u32 VERBOSITY = 0>
class StringifiableObjEquals
{
public:
    template <typename T>
    inline bool operator () (const T& Obj1, const T& Obj2) const
    {
        return (Obj1.ToString(VERBOSITY) == Obj2.ToString(VERBOSITY));
    }
};


} /* end namespace ESMC */

#endif /* ESMC_ESMC_ESMCTYPES_HPP_ */

//
// ESMCESMCTypes.hpp ends here
