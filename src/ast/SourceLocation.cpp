// SourceLocation.cpp --- 
// 
// Filename: SourceLocation.cpp
// Author: Abhishek Udupa
// Created: Sun Jun 29 20:27:30 2014 (-0400)
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

#include "SourceLocation.hpp"

namespace ESMC {

    const SourceLocation SourceLocation::NullLocation;

    SourceLocation::SourceLocation()
        : FileName(), LineNum(-1), ColNum(-1)
    {
        // Nothing here
    }

    SourceLocation::SourceLocation(const SourceLocation& Other)
        : FileName(Other.FileName), LineNum(Other.LineNum),
          ColNum(Other.ColNum)
    {
        // Nothing here
    }

    SourceLocation::SourceLocation(SourceLocation&& Other)
        : SourceLocation()
    {
        swap(FileName, Other.FileName);
        swap(LineNum, Other.LineNum);
        swap(ColNum, Other.ColNum);
    }

    SourceLocation::SourceLocation(string FileName, i32 LineNum, 
                                   i32 ColNum)
        : SourceLocation()
    {
        swap(this->FileName, FileName);
        this->LineNum = LineNum;
        this->ColNum = ColNum;
    }

    SourceLocation::~SourceLocation()
    {
        // Nothing here
    }

    SourceLocation& SourceLocation::operator = (SourceLocation Other)
    {
        swap(this->FileName, Other.FileName);
        this->LineNum = Other.LineNum;
        this->ColNum = Other.ColNum;
        return (*this);
    }

    bool SourceLocation::operator == (const SourceLocation& Other) const
    {
        return (FileName == Other.FileName &&
                LineNum == Other.LineNum &&
                ColNum == Other.ColNum);
    }

    bool SourceLocation::operator != (const SourceLocation& Other) const
    {
        return (!(*this == Other));
    }

    string SourceLocation::ToString() const
    {
        return ((string)"Line " + to_string(LineNum) + 
                ", Column " + to_string(ColNum) + 
                " in file \"" + FileName + "\"")
    }

    const string& SourceLocation::GetFileName() const
    {
        return FileName;
    }

    i32 SourceLocation::GetLineNum() const
    {
        return LineNum;
    }

    i32 SourceLocation::GetColNum() const
    {
        return ColNum;
    }

    ostream& operator << (ostream& Out, const SourceLocation& Loc)
    {
        Out << Loc.ToString();
        return Out;
    }

} /* end namespace */

// 
// SourceLocation.cpp ends here
