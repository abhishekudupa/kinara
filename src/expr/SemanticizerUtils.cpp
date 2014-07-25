// SemantizerUtils.cpp --- 
// 
// Filename: SemantizerUtils.cpp
// Author: Abhishek Udupa
// Created: Fri Jul 25 12:11:09 2014 (-0400)
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

#include "SemanticizerUtils.hpp"

namespace ESMC {
    namespace Exprs {
        namespace SemUtils {
            
            UFDescriptor::UFDescriptor()
                : RangeType(-1), Name(""), MangledName("")
            {
                // Nothing here
            }

            UFDescriptor::UFDescriptor(const UFDescriptor& Other) 
                : Identifier(Other.Identifier), 
                  DomainTypes(Other.DomainTypes), RangeType(Other.RangeType), 
                  Name(Other.Name), MangledName(Other.MangledName)
            {
                // Nothing here
            }

            UFDescriptor::UFDescriptor(i64 Identifier,
                                       const vector<i64>& DomainTypes,
                                       const i64 RangeType,
                                       const string& Name)
                : Identifier(Identifier),
                  DomainTypes(DomainTypes), RangeType(RangeType),
                  Name(Name), MangledName(MangleName(Name, DomainTypes))
            {
                // Nothing here
            }

            UFDescriptor::~UFDescriptor()
            {
                // Nothing here
            }

            UFDescriptor& UFDescriptor::operator = (const UFDescriptor& Other)
            {
                if (&Other == this) {
                    return *this;
                }
                Identifier = Other.Identifier;
                DomainTypes = Other.DomainTypes;
                RangeType = Other.RangeType;
                Name = Other.Name;
                MangledName = Other.MangledName;
                return *this;
            }

            const vector<i64>& UFDescriptor::GetDomainTypes() const
            {
                return DomainTypes;
            }

            i64 UFDescriptor::GetRangeType() const
            {
                return RangeType;
            }

            const string& UFDescriptor::GetName() const
            {
                return Name;
            }

            const string& UFDescriptor::GetMangledName() const
            {
                return MangledName;
            }

            i64 UFDescriptor::GetIdentifier() const
            {
                return Identifier;
            }            
            
        } /* end namespace SemUtils */
    } /* end namespace Exprs */
} /* end namespace ESMC */

// 
// SemantizerUtils.cpp ends here
