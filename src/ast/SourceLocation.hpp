// SourceLocation.hpp --- 
// 
// Filename: SourceLocation.hpp
// Author: Abhishek Udupa
// Created: Sun Jun 29 20:04:30 2014 (-0400)
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

#if !defined ESMC_SOURCE_LOCATION_HPP_
#define ESMC_SOURCE_LOCATION_HPP_

#include "../common/FwdDecls.hpp"

namespace ESMC {

    // This class is immutable 
    class SourceLocation
    {
    private:
        string FileName;
        i32 LineNum;
        i32 ColNum;

    public:
        static const SourceLocation NullLocation;
        SourceLocation();
        SourceLocation(const SourceLocation& Other);
        SourceLocation(SourceLocation&& Other);
        SourceLocation(const string FileName, i32 LineNum, i32 ColNum);
        ~SourceLocation();

        SourceLocation& operator = (SourceLocation Other);
       
        bool operator == (const SourceLocation& Other) const;
        bool operator != (const SourceLocation& Other) const;
        string ToString() const;

        // Accessors
        const string& GetFileName() const;
        i32 GetLineNum() const;
        i32 GetColNum() const;
    };

    extern ostream& operator << (ostream& Out, const SourceLocation& Loc);

} /* end namespace */

#endif /* ESMC_SOURCE_LOCATION_HPP_ */

// 
// SourceLocation.hpp ends here
