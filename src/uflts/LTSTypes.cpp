// LTSTypes.cpp --- 
// 
// Filename: LTSTypes.cpp
// Author: Abhishek Udupa
// Created: Thu Jul 24 15:10:36 2014 (-0400)
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

#include "LTSTypes.hpp"
#include "../utils/UIDGenerator.hpp"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/functional/hash.hpp>

namespace ESMC {
    namespace LTS {

        UIDGenerator LTSTypeBase::LTSTypeUIDGen(1);

        LTSTypeBase::LTSTypeBase()
            : TypeID(-1)
        {
            // Nothing here
        }

        LTSTypeBase::~LTSTypeBase()
        {
            // Nothing here
        }

        u64 LTSTypeBase::Hash() const
        {
            if (HashValid) {
                return HashCode;
            } else {
                ComputeHashValue();
                HashValid = true;
                return HashCode;
            }
        }

        bool LTSTypeBase::Equals(const LTSTypeBase& Other) const
        {
            return (this->Compare(Other) == 0);
        }

        bool LTSTypeBase::LT(const LTSTypeBase& Other) const
        {
            return (this->Compare(Other) < 0);
        }

        i64 LTSTypeBase::GetTypeID() const
        {
            return TypeID;
        }

        i64 LTSTypeBase::SetTypeID() const
        {
            TypeID = LTSTypeUIDGen.GetUID();
            return TypeID;
        }

        i64 LTSTypeBase::GetOrSetTypeID() const
        {
            if (TypeID == -1) {
                return SetTypeID();
            } else {
                return GetTypeID();
            }
        }

        LTSBoolType::LTSBoolType()
            : LTSTypeBase()
        {
            // Nothing here
        }

        LTSBoolType::~LTSBoolType()
        {
            // Nothing here
        }

        string LTSBoolType::ToString() const
        {
            return "(BoolType)";
        }

        i32 LTSBoolType::Compare(const LTSTypeBase& Other) const
        {
            if (Other.As<LTSBoolType>() == nullptr) {
                return -1;
            } else {
                return 0;
            }
        }

        void LTSBoolType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, "BoolType");
        }

        LTSIntType::LTSIntType()
            : LTSTypeBase()
        {
            // Nothing here
        }

        LTSIntType::~LTSIntType()
        {
            // Nothing here
        }
        
        void LTSIntType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, "IntType");
        }

        string LTSIntType::ToString() const
        {
            return "(IntType)";
        }

        i32 LTSIntType::Compare(const LTSTypeBase& Other) const
        {
            auto OtherAsPtr = &Other;
            
            if (OtherAsPtr->As<LTSBoolType>() != nullptr) {
                return 1;
            }
            
            auto OtherAsInt = OtherAsPtr->As<LTSIntType>();
            if (OtherAsInt == nullptr) {
                return -1;
            } else {
                auto OtherAsRange = OtherAsPtr->As<LTSRangeType>();
                if (OtherAsRange != nullptr) {
                    return -1;
                } else {
                    return 0;
                }
            }
        }

        // Inclusive range
        LTSRangeType::LTSRangeType(i64 RangeLow, i64 RangeHigh)
            : LTSIntType(), 
              RangeLow(RangeLow), RangeHigh(RangeHigh), 
              Size(RangeHigh - RangeLow + 1)
        {
            // Nothing here
        }

        LTSRangeType::~LTSRangeType()
        {
            // Nothing here
        }

        i64 LTSRangeType::GetLow() const
        {
            return RangeLow;
        }

        i64 LTSRangeType::GetHigh() const
        {
            return RangeHigh;
        }

        u64 LTSRangeType::GetSize() const
        {
            return Size;
        }

        void LTSRangeType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, RangeLow);
            boost::hash_combine(HashCode, RangeHigh);
        }

        string LTSRangeType::ToString() const
        {
            return ((string)"(Range [" + to_string(RangeLow) + 
                    "-" + to_string(RangeHigh) + "])");
        }

        i32 LTSRangeType::Compare(const LTSTypeBase& Other) const
        {
            auto OtherAsPtr = &Other;

            if (OtherAsPtr->As<LTSBoolType>() != nullptr) {
                return 1;
            }

            auto OtherAsRange = OtherAsPtr->As<LTSRangeType>();
            
            if (OtherAsRange == nullptr) {
                if (OtherAsPtr->As<LTSIntType>() != nullptr) {
                    return 1;
                } else {
                    return -1;
                }
            }
            
            if (OtherAsRange->Size > Size) {
                return -1;
            } else if (OtherAsRange->Size < Size) {
                return 1;
            } else {
                if (OtherAsRange->RangeLow < RangeLow) {
                    return 1;
                } else if (OtherAsRange->RangeLow > RangeLow) {
                    return -1;
                } else {
                    return 0;
                }
            }
        }

        LTSEnumType::LTSEnumType(const string& Name, 
                                 const set<string>& Members)
            : LTSTypeBase(), Name(Name), Members(Members)
        {
            // Nothing here
        }

        LTSEnumType::~LTSEnumType()
        {
            // Nothing here
        }

        const string& LTSEnumType::GetName() const
        {
            return Name;
        }

        const set<string>& LTSEnumType::GetMembers() const
        {
            return Members;
        }

        // We handle both qualified and unqualified enum names here
        bool LTSEnumType::IsMember(const string& MemberName) const
        {
            vector<string> SplitVec;
            boost::algorithm::split(SplitVec, MemberName, 
                                    boost::algorithm::is_any_of(":"),
                                    boost::algorithm::token_compress_on);
            string EnumName;
            if (SplitVec.size() == 2) {
                auto EnumName = boost::algorithm::trim_copy(SplitVec[0]);
                if (EnumName != Name) {
                    return false;
                } else {
                    auto MemberName = boost::algorithm::trim_copy(SplitVec[1]);
                    if (Members.find(MemberName) != Members.end()) {
                        return true;
                    } else {
                        return false;
                    }
                }
            } else if (SplitVec.size() == 1) {
                auto MemberName = boost::algorithm::trim_copy(SplitVec[0]);
                if (Members.find(MemberName) != Members.end()) {
                    return true;
                } else {
                    return false;
                }
            } else {
                return false;
            }
        }

        string LTSEnumType::ToString() const
        {
            return (string)"(Enum " + Name + ")";
        }

        void LTSEnumType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, Name);
            for (auto const& Mem : Members) {
                boost::hash_combine(HashCode, Mem);
            }
        }

        i32 LTSEnumType::Compare(const LTSTypeBase& Other) const
        {
            auto OtherPtr = &Other;
            if (OtherPtr->As<LTSBoolType>() != nullptr || 
                OtherPtr->As<LTSIntType>() != nullptr || 
                OtherPtr->As<LTSRangeType>() != nullptr) {
                return 1;
            }
            if (OtherPtr->As<LTSEnumType>() == nullptr) {
                return -1;
            }
            auto OtherAsEnum = OtherPtr->SAs<LTSEnumType>();

            if (OtherAsEnum->Name > Name) {
                return -1;
            } else if(OtherAsEnum->Name < Name) {
                return 1;
            } else {
                return 0;
            }
        }

        LTSSymmetricType::LTSSymmetricType(const string& Name, u32 Size)
            : LTSTypeBase(), Name(Name), Size(Size), Members(Size)
        {
            for(u32 i = 0; i < Size; ++i) {
                Members[i] = Name + to_string(i);
                MemberSet.insert(Name + to_string(i));
            }
        }

        LTSSymmetricType::~LTSSymmetricType()
        {
            // Nothing here
        }

        const string& LTSSymmetricType::GetName() const
        {
            return Name;
        }

        u32 LTSSymmetricType::GetSize() const
        {
            return Size;
        }

        const vector<string>& LTSSymmetricType::GetMembers() const
        {
            return Members;
        }

        const string& LTSSymmetricType::GetMember(u32 Index) const
        {
            assert (Index < Members.size());
            return Members[Index];
        }

        const bool LTSSymmetricType::IsMember(const string& Value) const
        {
            return (MemberSet.find(Value) != MemberSet.end());
        }

        string LTSSymmetricType::ToString() const
        {
            return (string)"(SymType " + Name + ")";
        }

        void LTSSymmetricType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, Name);
            boost::hash_combine(HashCode, Size);
        }

        i32 LTSSymmetricType::Compare(const LTSTypeBase& Other) const
        {
            auto OtherAsPtr = &Other;
            if (OtherAsPtr->As<LTSBoolType>() != nullptr ||
                OtherAsPtr->As<LTSIntType>() != nullptr ||
                OtherAsPtr->As<LTSRangeType>() != nullptr ||
                OtherAsPtr->As<LTSEnumType>() != nullptr) {
                return 1;
            }
            auto OtherAsSym = OtherAsPtr->As<LTSSymmetricType>();
            if (OtherAsPtr == nullptr) {
                return -1;
            }
            if (OtherAsSym->Name > Name) {
                return -1;
            } else if (OtherAsSym->Name < Name) {
                return 1;
            }
            return 0;
        }


        static inline string MangleName(const string& Name, 
                                        const vector<LTSTypeRef>& Args)
        {
            string Retval = Name;
            for (auto const& Arg : Args) {
                Retval += ((string)"@" + to_string(Arg->GetTypeID()));
            }
            return Retval;
        }

        LTSFuncType::LTSFuncType(const string& Name, 
                                 const vector<LTSTypeRef>& ArgTypes,
                                 const LTSTypeRef& FuncType)
            : LTSTypeBase(),
              Name(Name), MangledName(MangleName(Name, ArgTypes)), 
              ArgTypes(ArgTypes), FuncType(FuncType)
        {
            for(auto const& Arg : ArgTypes) {
                if (Arg->As<LTSFuncType>() != nullptr) {
                    throw ESMCError("Function types cannot have function types as params");
                }
            }
        }

        LTSFuncType::~LTSFuncType()
        {
            // Nothing here
        }

        const string& LTSFuncType::GetName() const
        {
            return Name;
        }

        const string& LTSFuncType::GetMangledName() const
        {
            return MangledName;
        }

        const vector<LTSTypeRef>& LTSFuncType::GetArgTypes() const
        {
            return ArgTypes;
        }

        const LTSTypeRef& LTSFuncType::GetFuncType() const
        {
            return FuncType;
        }

        void LTSFuncType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, Name);
            for (auto const& Arg : ArgTypes) {
                boost::hash_combine(HashCode, Arg->Hash());
            }
        }

        string LTSFuncType::ToString() const
        {
            string Retval = (string)"(Func " + Name + " : ";
            for (auto const& Arg : ArgTypes) {
                Retval += Arg->ToString() + " -> ";
            }
            Retval += (FuncType->ToString() + ")");
            return Retval;
        }

        i32 LTSFuncType::Compare(const LTSTypeBase& Other) const
        {
            auto OtherAsPtr = &Other;
            if (OtherAsPtr->As<LTSBoolType>() != nullptr ||
                OtherAsPtr->As<LTSIntType>() != nullptr ||
                OtherAsPtr->As<LTSRangeType> () != nullptr ||
                OtherAsPtr->As<LTSEnumType>() != nullptr ||
                OtherAsPtr->As<LTSSymmetricType>() != nullptr) {
                return 1;
            }
            
            auto OtherAsFunc = OtherAsPtr->As<LTSFuncType>();
            
            if (OtherAsFunc == nullptr) {
                return -1;
            }

            if (Name < OtherAsFunc->Name) {
                return -1;
            } else if (Name > OtherAsFunc->Name) {
                return 1;
            } else {
                if (ArgTypes.size() < OtherAsFunc->ArgTypes.size()) {
                    return -1;
                } else if (ArgTypes.size() > OtherAsFunc->ArgTypes.size()) {
                    return 1;
                } else {
                    for (u32 i = 0; i < ArgTypes.size(); ++i) {
                        auto Cmp = ArgTypes[i]->Compare(*OtherAsFunc->ArgTypes[i]);
                        if (Cmp != 0) {
                            return Cmp;
                        }
                    }
                    return 0;
                }
            }
        }

        LTSArrayType::LTSArrayType(const LTSTypeRef& IndexType,
                                   const LTSTypeRef& ValueType)
            : LTSTypeBase(), IndexType(IndexType), ValueType(ValueType)
        {
            if (IndexType->As<LTSFuncType>() != nullptr ||
                ValueType->As<LTSFuncType>() != nullptr) {
                throw ESMCError((string)"Array indices and values cannot be functions");
            }
        }

        LTSArrayType::~LTSArrayType()
        {
            // Nothing here
        }

        const LTSTypeRef& LTSArrayType::GetIndexType() const
        {
            return IndexType;
        }

        const LTSTypeRef& LTSArrayType::GetValueType() const
        {
            return ValueType;
        }

        string LTSArrayType::ToString() const
        {
            return ((string)"(Array : " + IndexType->ToString() + " -> " + 
                    ValueType->ToString() + ")");
        }

        i32 LTSArrayType::Compare(const LTSTypeBase& Other) const
        {
            auto OtherAsPtr = &Other;
            if (OtherAsPtr->As<LTSBoolType>() != nullptr ||
                OtherAsPtr->As<LTSIntType>() != nullptr ||
                OtherAsPtr->As<LTSRangeType>() != nullptr ||
                OtherAsPtr->As<LTSEnumType>() != nullptr ||
                OtherAsPtr->As<LTSSymmetricType>() != nullptr ||
                OtherAsPtr->As<LTSFuncType>() != nullptr) {
                return 1;
            }

            auto OtherAsArr = OtherAsPtr->As<LTSArrayType>();

            if (OtherAsArr == nullptr) {
                return -1;
            }

            auto Cmp = IndexType->Compare(*(OtherAsArr->IndexType));
            if (Cmp != 0) {
                return Cmp;
            }
            return ValueType->Compare(*(OtherAsArr->ValueType));
        }

        void LTSArrayType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, IndexType->Hash());
            boost::hash_combine(HashCode, ValueType->Hash());
        }

        LTSRecordType::LTSRecordType(const string& Name,
                                     const map<string, LTSTypeRef>& Members)
            : LTSTypeBase(), Name(Name), Members(Members)
        {
            for (auto const& NTPair : Members) {
                if (NTPair.second->As<LTSFuncType>() != nullptr) {
                    throw ESMCError("Record members cannot be functions types");
                }
            }
        }

        LTSRecordType::~LTSRecordType()
        {
            // Nothing here
        }

        const string& LTSRecordType::GetName() const
        {
            return Name;
        }

        const map<string, LTSTypeRef>& LTSRecordType::GetMembers() const
        {
            return Members;
        }

        const LTSTypeRef& LTSRecordType::GetTypeForMember(const string& MemberName) const
        {
            auto it = Members.find(MemberName);
            if (it == Members.end()) {
                return LTSTypeRef::NullPtr;
            }
            return it->second;
        }

        string LTSRecordType::ToString() const
        {
            string Retval;
            Retval += "(Rec :";
            for (auto const& NTPair : Members) {
                Retval += (" (" + NTPair.first + " : " + NTPair.second->ToString() + ")");
            }
            Retval += ")";
            return Retval;
        }

        i32 LTSRecordType::Compare(const LTSTypeBase& Other) const
        {
            auto OtherAsPtr = &Other;
            if (OtherAsPtr->As<LTSBoolType>() != nullptr ||
                OtherAsPtr->As<LTSIntType>() != nullptr ||
                OtherAsPtr->As<LTSRangeType>() != nullptr ||
                OtherAsPtr->As<LTSEnumType>() != nullptr ||
                OtherAsPtr->As<LTSSymmetricType>() != nullptr ||
                OtherAsPtr->As<LTSFuncType> () != nullptr ||
                OtherAsPtr->As<LTSArrayType> () != nullptr) {
                return 1;
            }

            auto OtherAsRec = OtherAsPtr->As<LTSRecordType>();
            if (OtherAsRec == nullptr) {
                return -1;
            }

            if (Name < OtherAsRec->Name) {
                return -1;
            } else if (Name > OtherAsRec->Name) {
                return 1;
            } else {
                return 0;
            }
        }

        void LTSRecordType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, Name);
            boost::hash_combine(HashCode, Members.size());
        }

        LTSParametricType::LTSParametricType(const LTSTypeRef& BaseType,
                                             const LTSTypeRef& ParameterType)
            : LTSTypeBase(), BaseType(BaseType), ParameterType(ParameterType)
        {
            if (ParameterType->As<LTSEnumType>() == nullptr &&
                ParameterType->As<LTSRangeType>() == nullptr &&
                ParameterType->As<LTSSymmetricType>() == nullptr) {
                throw ESMCError((string)"Parameteric types must have enum, range " + 
                                "or symmetric types as type parameters");
            }
        }

        LTSParametricType::~LTSParametricType()
        {
            // Nothing here
        }

        const LTSTypeRef& LTSParametricType::GetBaseType() const
        {
            return BaseType;
        }

        const LTSTypeRef& LTSParametricType::GetParameterType() const
        {
            return ParameterType;
        }

        void LTSParametricType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, BaseType->Hash());
            boost::hash_combine(HashCode, ParameterType->Hash());
        }

        string LTSParametricType::ToString() const
        {
            string Retval = "(ParamType : ";
            Retval += (ParameterType->ToString() + " -> ");
            Retval += (BaseType->ToString() + ")");
            return Retval;
        }

        i32 LTSParametricType::Compare(const LTSTypeBase& Other) const
        {
            auto OtherAsPtr = &Other;

            if (OtherAsPtr->As<LTSBoolType>() != nullptr ||
                OtherAsPtr->As<LTSIntType>() != nullptr ||
                OtherAsPtr->As<LTSRangeType>() != nullptr ||
                OtherAsPtr->As<LTSEnumType>() != nullptr ||
                OtherAsPtr->As<LTSSymmetricType>() != nullptr ||
                OtherAsPtr->As<LTSFuncType>() != nullptr ||
                OtherAsPtr->As<LTSArrayType>() != nullptr ||
                OtherAsPtr->As<LTSRecordType>() != nullptr) {
                return 1;
            }

            auto OtherAsPar = OtherAsPtr->As<LTSParametricType>();

            if (OtherAsPar == nullptr) {
                return -1;
            }

            auto Cmp = BaseType->Compare(*OtherAsPar->BaseType);
            if (Cmp != 0) {
                return Cmp;
            }

            return (ParameterType->Compare(*OtherAsPar->ParameterType));
        }

        LTSFieldAccessType::LTSFieldAccessType()
            : LTSTypeBase()
        {
            // Nothing here
        }

        LTSFieldAccessType::~LTSFieldAccessType()
        {
            // Nothing here
        }

        void LTSFieldAccessType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, "FieldAccessType");
        }

        i32 LTSFieldAccessType::Compare(const LTSTypeBase& Other) const
        {
            if (Other.As<LTSBoolType>() != nullptr ||
                Other.As<LTSIntType>() != nullptr ||
                Other.As<LTSRangeType>() != nullptr ||
                Other.As<LTSEnumType> () != nullptr ||
                Other.As<LTSSymmetricType>() != nullptr ||
                Other.As<LTSFuncType> () != nullptr || 
                Other.As<LTSArrayType>() != nullptr ||
                Other.As<LTSRecordType>() != nullptr ||
                Other.As<LTSParametricType>() != nullptr) {
                return 1;
            }

            if (Other.As<LTSFieldAccessType>() == nullptr) {
                return -1;
            } else {
                return 0;
            }
        }

        string LTSFieldAccessType::ToString() const
        {
            return "(FieldAccessType)";
        }

        LTSUndefType::LTSUndefType()
            : LTSTypeBase()
        {
            // NOthing here
        }

        LTSUndefType::~LTSUndefType()
        {
            // Nothing here
        }

        void LTSUndefType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, "UndefType");
        }

        string LTSUndefType::ToString() const
        {
            return "(UndefType)";
        }

        i32 LTSUndefType::Compare(const LTSTypeBase& Other) const
        {
            if (Other.As<LTSBoolType>() != nullptr ||
                Other.As<LTSIntType>() != nullptr ||
                Other.As<LTSRangeType>() != nullptr ||
                Other.As<LTSEnumType> () != nullptr ||
                Other.As<LTSSymmetricType>() != nullptr ||
                Other.As<LTSFuncType> () != nullptr || 
                Other.As<LTSArrayType>() != nullptr ||
                Other.As<LTSRecordType>() != nullptr ||
                Other.As<LTSParametricType>() != nullptr ||
                Other.As<LTSFieldAccessType>() != nullptr) {
                return 1;
            }

            if (Other.Is<LTSUndefType>()) {
                return 0;
            } else {
                return -1;
            }
        }

    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// LTSTypes.cpp ends here
