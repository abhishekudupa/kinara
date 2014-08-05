// ExprTypes.cpp --- 
// 
// Filename: ExprTypes.cpp
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

#include "ExprTypes.hpp"
#include "../utils/UIDGenerator.hpp"
#include "../utils/SizeUtils.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/functional/hash.hpp>

namespace ESMC {
    namespace Exprs {

        UIDGenerator ExprTypeBase::ExprTypeUIDGen(1);

        ExprTypeBase::ExprTypeBase()
            : TypeID(-1), HashValid(false), LastExtension(nullptr)
        {
            // Nothing here
        }

        ExprTypeBase::~ExprTypeBase()
        {
            // Nothing here
        }

        u64 ExprTypeBase::Hash() const
        {
            if (HashValid) {
                return HashCode;
            } else {
                ComputeHashValue();
                HashValid = true;
                return HashCode;
            }
        }

        bool ExprTypeBase::Equals(const ExprTypeBase& Other) const
        {
            return (this->Compare(Other) == 0);
        }

        bool ExprTypeBase::LT(const ExprTypeBase& Other) const
        {
            return (this->Compare(Other) < 0);
        }

        i64 ExprTypeBase::GetTypeID() const
        {
            return TypeID;
        }

        i64 ExprTypeBase::SetTypeID() const
        {
            TypeID = ExprTypeUIDGen.GetUID();
            return TypeID;
        }

        i64 ExprTypeBase::GetOrSetTypeID() const
        {
            if (TypeID == -1) {
                return SetTypeID();
            } else {
                return GetTypeID();
            }
        }

        ExprScalarType::ExprScalarType()
            : ExprTypeBase()
        {
            // Nothing here
        }

        ExprScalarType::~ExprScalarType()
        {
            // Nothing here
        }

        ExprBoolType::ExprBoolType()
            : ExprScalarType()
        {
            // Nothing here
        }

        ExprBoolType::~ExprBoolType()
        {
            // Nothing here
        }

        string ExprBoolType::ToString() const
        {
            return "(BoolType)";
        }

        i32 ExprBoolType::Compare(const ExprTypeBase& Other) const
        {
            if (Other.As<ExprBoolType>() == nullptr) {
                return -1;
            } else {
                return 0;
            }
        }

        vector<string> ExprBoolType::GetElements() const
        {
            vector<string> Retval = { "true", "false" };
            return Retval;
        }

        void ExprBoolType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, "BoolType");
        }

        u32 ExprBoolType::GetByteSize() const
        {
            return 1;
        }

        ExprIntType::ExprIntType()
            : ExprScalarType()
        {
            // Nothing here
        }

        ExprIntType::~ExprIntType()
        {
            // Nothing here
        }
        
        void ExprIntType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, "IntType");
        }

        string ExprIntType::ToString() const
        {
            return "(IntType)";
        }

        i32 ExprIntType::Compare(const ExprTypeBase& Other) const
        {
            auto OtherAsPtr = &Other;
            
            if (OtherAsPtr->As<ExprBoolType>() != nullptr) {
                return 1;
            }
            
            auto OtherAsInt = OtherAsPtr->As<ExprIntType>();
            if (OtherAsInt == nullptr) {
                return -1;
            } else {
                auto OtherAsRange = OtherAsPtr->As<ExprRangeType>();
                if (OtherAsRange != nullptr) {
                    return -1;
                } else {
                    return 0;
                }
            }
        }

        vector<string> ExprIntType::GetElements() const
        {
            throw ESMCError((string)"Cannot GetElements() on unbounded type IntType");
        }

        u32 ExprIntType::GetByteSize() const
        {
            throw InternalError((string)"ExprIntType::GetByteSize() should never have been " + 
                                "called.\nAt: " + __FILE__ + ":" + to_string(__LINE__));
        }

        // Inclusive range
        ExprRangeType::ExprRangeType(i64 RangeLow, i64 RangeHigh)
            : ExprIntType(), 
              RangeLow(RangeLow), RangeHigh(RangeHigh), 
              Size(RangeHigh - RangeLow + 1)
        {
            if (RangeLow > RangeHigh) {
                throw ESMCError((string)"Negative size specified for Range type");
            }
        }

        ExprRangeType::~ExprRangeType()
        {
            // Nothing here
        }

        i64 ExprRangeType::GetLow() const
        {
            return RangeLow;
        }

        i64 ExprRangeType::GetHigh() const
        {
            return RangeHigh;
        }

        u64 ExprRangeType::GetSize() const
        {
            return Size;
        }

        void ExprRangeType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, RangeLow);
            boost::hash_combine(HashCode, RangeHigh);
        }

        u32 ExprRangeType::GetByteSize() const 
        {
            return BytesForRange(RangeHigh - RangeLow + 1);
        }

        string ExprRangeType::ToString() const
        {
            return ((string)"(Range [" + to_string(RangeLow) + 
                    "-" + to_string(RangeHigh) + "])");
        }

        i32 ExprRangeType::Compare(const ExprTypeBase& Other) const
        {
            auto OtherAsPtr = &Other;

            if (OtherAsPtr->As<ExprBoolType>() != nullptr) {
                return 1;
            }

            auto OtherAsRange = OtherAsPtr->As<ExprRangeType>();
            
            if (OtherAsRange == nullptr) {
                if (OtherAsPtr->As<ExprIntType>() != nullptr) {
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

        vector<string> ExprRangeType::GetElements() const
        {
            vector<string> Retval;
            for(i64 i = RangeLow; i <= RangeHigh; ++i) {
                Retval.push_back(to_string(i));
            }
            return Retval;
        }

        ExprEnumType::ExprEnumType(const string& Name, 
                                 const set<string>& Members)
            : ExprScalarType(), Name(Name), Members(Members)
        {
            // Nothing here
        }

        ExprEnumType::~ExprEnumType()
        {
            // Nothing here
        }

        const string& ExprEnumType::GetName() const
        {
            return Name;
        }

        const set<string>& ExprEnumType::GetMembers() const
        {
            return Members;
        }

        // We handle both qualified and unqualified enum names here
        bool ExprEnumType::IsMember(const string& MemberName) const
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

        string ExprEnumType::ToString() const
        {
            return (string)"(Enum " + Name + ")";
        }

        void ExprEnumType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, Name);
            for (auto const& Mem : Members) {
                boost::hash_combine(HashCode, Mem);
            }
        }

        u32 ExprEnumType::GetByteSize() const
        {
            return BytesForRange(Members.size());
        }

        i32 ExprEnumType::Compare(const ExprTypeBase& Other) const
        {
            auto OtherPtr = &Other;
            if (OtherPtr->As<ExprBoolType>() != nullptr || 
                OtherPtr->As<ExprIntType>() != nullptr || 
                OtherPtr->As<ExprRangeType>() != nullptr) {
                return 1;
            }
            if (OtherPtr->As<ExprEnumType>() == nullptr) {
                return -1;
            }
            auto OtherAsEnum = OtherPtr->SAs<ExprEnumType>();

            if (OtherAsEnum->Name > Name) {
                return -1;
            } else if(OtherAsEnum->Name < Name) {
                return 1;
            } else {
                return 0;
            }
        }

        vector<string> ExprEnumType::GetElements() const
        {
            return (vector<string>(Members.begin(), Members.end()));
        }

        ExprSymmetricType::ExprSymmetricType(const string& Name, u32 Size)
            : ExprScalarType(), Name(Name), Size(Size), Members(Size)
        {
            for(u32 i = 0; i < Size; ++i) {
                Members[i] = Name + "::" + to_string(i);
                MemberSet.insert(Name + "::" + to_string(i));
            }
        }

        ExprSymmetricType::~ExprSymmetricType()
        {
            // Nothing here
        }

        const string& ExprSymmetricType::GetName() const
        {
            return Name;
        }

        u32 ExprSymmetricType::GetSize() const
        {
            return Size;
        }

        const vector<string>& ExprSymmetricType::GetMembers() const
        {
            return Members;
        }

        const string& ExprSymmetricType::GetMember(u32 Index) const
        {
            assert (Index < Members.size());
            return Members[Index];
        }

        const bool ExprSymmetricType::IsMember(const string& Value) const
        {
            return (MemberSet.find(Value) != MemberSet.end());
        }
        
        void ExprSymmetricType::SetIndex(u32 Index) const
        {
            this->Index = Index;
        }

        u32 ExprSymmetricType::GetIndex() const
        {
            return Index;
        }

        string ExprSymmetricType::ToString() const
        {
            return (string)"(SymType " + Name + ")";
        }

        void ExprSymmetricType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, Name);
            boost::hash_combine(HashCode, Size);
        }

        u32 ExprSymmetricType::GetByteSize() const
        {
            return BytesForRange(Members.size());
        }

        i32 ExprSymmetricType::Compare(const ExprTypeBase& Other) const
        {
            auto OtherAsPtr = &Other;
            if (OtherAsPtr->As<ExprBoolType>() != nullptr ||
                OtherAsPtr->As<ExprIntType>() != nullptr ||
                OtherAsPtr->As<ExprRangeType>() != nullptr ||
                OtherAsPtr->As<ExprEnumType>() != nullptr) {
                return 1;
            }
            auto OtherAsSym = OtherAsPtr->As<ExprSymmetricType>();
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

        vector<string> ExprSymmetricType::GetElements() const
        {
            return (vector<string>(MemberSet.begin(), MemberSet.end()));
        }

        static inline string MangleName(const string& Name, 
                                        const vector<ExprTypeRef>& Args)
        {
            string Retval = Name;
            for (auto const& Arg : Args) {
                Retval += ((string)"@" + to_string(Arg->GetTypeID()));
            }
            return Retval;
        }

        ExprFuncType::ExprFuncType(const string& Name, 
                                 const vector<ExprTypeRef>& ArgTypes,
                                 const ExprTypeRef& FuncType)
            : ExprTypeBase(),
              Name(Name), MangledName(MangleName(Name, ArgTypes)), 
              ArgTypes(ArgTypes), FuncType(FuncType)
        {
            for(auto const& Arg : ArgTypes) {
                if (Arg->As<ExprFuncType>() != nullptr) {
                    throw ESMCError("Function types cannot have function types as params");
                }
                // if (!Arg->Is<ExprScalarType>()) {
                //     throw ESMCError((string)"Only function types with scalar domain types " + 
                //                     "are currenly supported");
                // }
            }
        }

        ExprFuncType::~ExprFuncType()
        {
            // Nothing here
        }

        const string& ExprFuncType::GetName() const
        {
            return Name;
        }

        const string& ExprFuncType::GetMangledName() const
        {
            return MangledName;
        }

        const vector<ExprTypeRef>& ExprFuncType::GetArgTypes() const
        {
            return ArgTypes;
        }

        const ExprTypeRef& ExprFuncType::GetFuncType() const
        {
            return FuncType;
        }

        void ExprFuncType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, Name);
            for (auto const& Arg : ArgTypes) {
                boost::hash_combine(HashCode, Arg->Hash());
            }
        }

        string ExprFuncType::ToString() const
        {
            string Retval = (string)"(Func " + Name + " : ";
            for (auto const& Arg : ArgTypes) {
                Retval += Arg->ToString() + " -> ";
            }
            Retval += (FuncType->ToString() + ")");
            return Retval;
        }

        i32 ExprFuncType::Compare(const ExprTypeBase& Other) const
        {
            auto OtherAsPtr = &Other;
            if (OtherAsPtr->As<ExprBoolType>() != nullptr ||
                OtherAsPtr->As<ExprIntType>() != nullptr ||
                OtherAsPtr->As<ExprRangeType> () != nullptr ||
                OtherAsPtr->As<ExprEnumType>() != nullptr ||
                OtherAsPtr->As<ExprSymmetricType>() != nullptr) {
                return 1;
            }
            
            auto OtherAsFunc = OtherAsPtr->As<ExprFuncType>();
            
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

        vector<string> ExprFuncType::GetElements() const
        {
            throw ESMCError((string)"Cannot GetElements() of a function type");
        }

        u32 ExprFuncType::GetByteSize() const
        {
            // We return the byte size of the range multiplied
            // by the product of the domains
            u32 Retval = FuncType->GetByteSize();
            for (auto const& ArgType : ArgTypes) {
                Retval *= ArgType->GetElements().size();
            }
            return Retval;
        }

        ExprArrayType::ExprArrayType(const ExprTypeRef& IndexType,
                                   const ExprTypeRef& ValueType)
            : ExprTypeBase(), IndexType(IndexType), ValueType(ValueType)
        {
            if (IndexType->As<ExprFuncType>() != nullptr ||
                ValueType->As<ExprFuncType>() != nullptr || 
                IndexType->As<ExprScalarType>() != nullptr ||
                ValueType->As<ExprParametricType>() != nullptr || 
                ValueType->As<ExprFieldAccessType>() != nullptr) {
                throw ESMCError((string)"Array indices and values cannot be functions, " + 
                                "further, indices must be scalar and values cannot be " + 
                                "field access tyeps or parametric types");
            }
        }

        ExprArrayType::~ExprArrayType()
        {
            // Nothing here
        }

        const ExprTypeRef& ExprArrayType::GetIndexType() const
        {
            return IndexType;
        }

        const ExprTypeRef& ExprArrayType::GetValueType() const
        {
            return ValueType;
        }

        string ExprArrayType::ToString() const
        {
            return ((string)"(Array : " + IndexType->ToString() + " -> " + 
                    ValueType->ToString() + ")");
        }

        i32 ExprArrayType::Compare(const ExprTypeBase& Other) const
        {
            auto OtherAsPtr = &Other;
            if (OtherAsPtr->As<ExprBoolType>() != nullptr ||
                OtherAsPtr->As<ExprIntType>() != nullptr ||
                OtherAsPtr->As<ExprRangeType>() != nullptr ||
                OtherAsPtr->As<ExprEnumType>() != nullptr ||
                OtherAsPtr->As<ExprSymmetricType>() != nullptr ||
                OtherAsPtr->As<ExprFuncType>() != nullptr) {
                return 1;
            }

            auto OtherAsArr = OtherAsPtr->As<ExprArrayType>();

            if (OtherAsArr == nullptr) {
                return -1;
            }

            auto Cmp = IndexType->Compare(*(OtherAsArr->IndexType));
            if (Cmp != 0) {
                return Cmp;
            }
            return ValueType->Compare(*(OtherAsArr->ValueType));
        }

        vector<string> ExprArrayType::GetElements() const
        {
            throw ESMCError((string)"Cannot get elements of non-scalar type");
        }

        void ExprArrayType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, IndexType->Hash());
            boost::hash_combine(HashCode, ValueType->Hash());
        }

        u32 ExprArrayType::GetByteSize() const
        {
            u32 Retval = ValueType->GetByteSize();
            return (Retval * IndexType->GetElements().size());
        }

        ExprRecordType::ExprRecordType(const string& Name,
                                       const vector<pair<string, ExprTypeRef>>& Members)
            : ExprTypeBase(), Name(Name), MemberVec(Members)
        {
            for (auto const& NTPair : Members) {
                if (NTPair.second->As<ExprFuncType>() != nullptr ||
                    NTPair.second->As<ExprFieldAccessType>() != nullptr || 
                    NTPair.second->As<ExprParametricType>() != nullptr) {
                    throw ESMCError((string)"Record members cannot be functions types or " + 
                                    "field access types, or parametric types");
                }
            }
            for (auto const& Member : Members) {
                if (MemberMap.find(Member.first) != MemberMap.end()) {
                    throw ESMCError((string)"Member \"" + Member.first + 
                                    "\" redeclared in record type \"" + 
                                    Name + "\"");
                }
                MemberMap[Member.first] = Member.second;
            }
        }

        ExprRecordType::ExprRecordType()
            : ExprTypeBase()
        {
            // Nothing here
        }

        ExprRecordType::~ExprRecordType()
        {
            // Nothing here
        }

        const string& ExprRecordType::GetName() const
        {
            return Name;
        }

        const map<string, ExprTypeRef>& ExprRecordType::GetMemberMap() const
        {
            return MemberMap;
        }

        const vector<pair<string, ExprTypeRef>>& 
        ExprRecordType::GetMemberVec() const
        {
            return MemberVec;
        }

        const ExprTypeRef& ExprRecordType::GetTypeForMember(const string& MemberName) const
        {
            auto it = MemberMap.find(MemberName);
            if (it == MemberMap.end()) {
                return ExprTypeRef::NullPtr;
            }
            return it->second;
        }

        string ExprRecordType::ToString() const
        {
            string Retval;
            Retval += "(Rec " + Name + " :";
            for (auto const& NTPair : MemberVec) {
                Retval += (" (" + NTPair.first + " : " + NTPair.second->ToString() + ")");
            }
            Retval += ")";
            return Retval;
        }

        i32 ExprRecordType::Compare(const ExprTypeBase& Other) const
        {
            auto OtherAsPtr = &Other;
            if (OtherAsPtr->As<ExprBoolType>() != nullptr ||
                OtherAsPtr->As<ExprIntType>() != nullptr ||
                OtherAsPtr->As<ExprRangeType>() != nullptr ||
                OtherAsPtr->As<ExprEnumType>() != nullptr ||
                OtherAsPtr->As<ExprSymmetricType>() != nullptr ||
                OtherAsPtr->As<ExprFuncType> () != nullptr ||
                OtherAsPtr->As<ExprArrayType> () != nullptr) {
                return 1;
            }

            auto OtherAsRec = OtherAsPtr->As<ExprRecordType>();
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

        vector<string> ExprRecordType::GetElements() const
        {
            throw ESMCError((string)"Cannot get elements of non-scalar type");
        }

        void ExprRecordType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, Name);
            boost::hash_combine(HashCode, MemberVec.size());
        }

        u32 ExprRecordType::GetByteSize() const
        {
            u32 Offset = 0;
            u32 Retval = 0;
            for (auto const& Member : MemberVec) {
                auto CurSize = Member.second->GetByteSize();
                Offset = NextMultiple(Retval, CurSize);
                Retval += (Offset + CurSize);
            }
            return Retval;
        }

        ExprParametricType::ExprParametricType(const ExprTypeRef& BaseType,
                                              const ExprTypeRef& ParameterType)
            : ExprTypeBase(), BaseType(BaseType), ParameterType(ParameterType)
        {
            if (!BaseType->Is<ExprRecordType>() &&
                !BaseType->Is<ExprParametricType>()) {
                throw ESMCError((string)"Only record types (or other parametric types) " + 
                                "can currently be parametrized");
            }
            if (ParameterType->As<ExprEnumType>() == nullptr &&
                ParameterType->As<ExprRangeType>() == nullptr &&
                ParameterType->As<ExprSymmetricType>() == nullptr) {
                throw ESMCError((string)"Parameteric types must have enum, range " + 
                                "or symmetric types as type parameters");
            }
        }

        ExprParametricType::~ExprParametricType()
        {
            // Nothing here
        }

        const ExprTypeRef& ExprParametricType::GetBaseType() const
        {
            return BaseType;
        }

        const ExprTypeRef& ExprParametricType::GetParameterType() const
        {
            return ParameterType;
        }

        const string& ExprParametricType::GetName() const
        {
            ExprTypeRef CurPType = this;
            do {
                auto CurPTypeAsParam = CurPType->SAs<ExprParametricType>();
                CurPType = CurPTypeAsParam->GetBaseType();
            } while (!CurPType->Is<Exprs::ExprRecordType>());
            auto TypeAsRec = CurPType->SAs<Exprs::ExprRecordType>();
            return TypeAsRec->GetName();
        }

        ExprTypeRef ExprParametricType::GetTrueBaseType() const
        {
            ExprTypeRef CurPType = this;
            do {
                auto CurPTypeAsParam = CurPType->SAs<ExprParametricType>();
                CurPType = CurPTypeAsParam->GetBaseType();
            } while (!CurPType->Is<Exprs::ExprRecordType>());
            return CurPType;
        }

        void ExprParametricType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, BaseType->Hash());
            boost::hash_combine(HashCode, ParameterType->Hash());
        }

        string ExprParametricType::ToString() const
        {
            string Retval = "(ParamType : ";
            Retval += (ParameterType->ToString() + " -> ");
            Retval += (BaseType->ToString() + ")");
            return Retval;
        }

        i32 ExprParametricType::Compare(const ExprTypeBase& Other) const
        {
            auto OtherAsPtr = &Other;

            if (OtherAsPtr->As<ExprBoolType>() != nullptr ||
                OtherAsPtr->As<ExprIntType>() != nullptr ||
                OtherAsPtr->As<ExprRangeType>() != nullptr ||
                OtherAsPtr->As<ExprEnumType>() != nullptr ||
                OtherAsPtr->As<ExprSymmetricType>() != nullptr ||
                OtherAsPtr->As<ExprFuncType>() != nullptr ||
                OtherAsPtr->As<ExprArrayType>() != nullptr ||
                OtherAsPtr->As<ExprRecordType>() != nullptr) {
                return 1;
            }

            auto OtherAsPar = OtherAsPtr->As<ExprParametricType>();

            if (OtherAsPar == nullptr) {
                return -1;
            }

            auto Cmp = BaseType->Compare(*OtherAsPar->BaseType);
            if (Cmp != 0) {
                return Cmp;
            }

            return (ParameterType->Compare(*OtherAsPar->ParameterType));
        }

        vector<string> ExprParametricType::GetElements() const
        {
            throw ESMCError((string)"Cannot get elements of non-scalar type");
        }

        u32 ExprParametricType::GetByteSize() const
        {
            throw InternalError((string)"ExprParametricType::GetByteSize() should never " + 
                                "have been called.\nAt: " + __FILE__ + ":" + to_string(__LINE__));
        }

        ExprFieldAccessType::ExprFieldAccessType()
            : ExprTypeBase()
        {
            // Nothing here
        }

        ExprFieldAccessType::~ExprFieldAccessType()
        {
            // Nothing here
        }

        void ExprFieldAccessType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, "FieldAccessType");
        }

        i32 ExprFieldAccessType::Compare(const ExprTypeBase& Other) const
        {
            if (Other.As<ExprBoolType>() != nullptr ||
                Other.As<ExprIntType>() != nullptr ||
                Other.As<ExprRangeType>() != nullptr ||
                Other.As<ExprEnumType> () != nullptr ||
                Other.As<ExprSymmetricType>() != nullptr ||
                Other.As<ExprFuncType> () != nullptr || 
                Other.As<ExprArrayType>() != nullptr ||
                Other.As<ExprRecordType>() != nullptr ||
                Other.As<ExprParametricType>() != nullptr) {
                return 1;
            }

            if (Other.As<ExprFieldAccessType>() == nullptr) {
                return -1;
            } else {
                return 0;
            }
        }

        vector<string> ExprFieldAccessType::GetElements() const
        {
            throw ESMCError((string)"Cannot get elements of non-scalar type");
        }


        string ExprFieldAccessType::ToString() const
        {
            return "(FieldAccessType)";
        }

        u32 ExprFieldAccessType::GetByteSize() const
        {
            throw InternalError((string)"ExprFieldAccessType::GetByteSize() should never " + 
                                "have been called.\nAt: " + __FILE__ + ":" + to_string(__LINE__));
        }

        ExprMessageType::ExprMessageType(const string& Name,
                                         const set<ExprTypeRef>& MemberTypes,
                                         const ExprTypeRef& TypeIDFieldType)
            : ExprRecordType(), MemberTypes(MemberTypes),
              TypeIDFieldType(TypeIDFieldType)
        {
            this->Name = Name;
            // Sanity checks
            if (!TypeIDFieldType->Is<ExprRangeType>()) {
                throw ESMCError((string)"ExprMessageType must be instantiated with a " + 
                                "subrange type for TypeIDFieldType");
            }

            auto TypeIDTypeAsRange = TypeIDFieldType->As<ExprRangeType>();
            auto Low = TypeIDTypeAsRange->GetLow();
            if (Low != 0) {
                throw ESMCError((string)"Subrange type for ExprMessageType must have low at 0");
            }
            
            const u32 MaxNumTypes = TypeIDTypeAsRange->GetHigh();
            if (MaxNumTypes < MemberTypes.size()) {
                throw ESMCError((string)"Subrange type not sufficient to accommodate all " + 
                                "member types");
            }
            
            u32 NumTypesSeen = 0;
            // Find the maximum number of occurences of each type
            // in each member type
            map<ExprTypeRef, u32> MaxOccCount;
            
            for (auto const& MemberType : MemberTypes) {
                auto MemberTypeAsRec = MemberType->As<ExprRecordType>();
                if (MemberTypeAsRec == nullptr) {
                    throw ESMCError((string)"Only record types can be members of Message types");
                }
                map<ExprTypeRef, u32> CurOccCount;
                auto const& MemberVec = MemberTypeAsRec->GetMemberVec();
                
                for (auto const& Member : MemberVec) {
                    if (Member.first == TypeIDFieldName) {
                        throw ESMCError((string)"One of the record types in args to MessageType " + 
                                        "contains a reserved field \"" + TypeIDFieldName + "\"");
                    }
                    
                    if (CurOccCount.find(Member.second) == CurOccCount.end()) {
                        CurOccCount[Member.second] = 0;
                    }

                    CurOccCount[Member.second]++;
                }

                for (auto const TypeCount : CurOccCount) {
                    if (MaxOccCount.find(TypeCount.first) == MaxOccCount.end()) {
                        MaxOccCount[TypeCount.first] = 0;
                    }
                    if (MaxOccCount[TypeCount.first] < TypeCount.second) {
                        MaxOccCount[TypeCount.first] = TypeCount.second;
                    }
                }

                // Instantiate the TypeID fields as well
                auto CurID = NumTypesSeen++;
                MemberTypeToID[MemberType] = CurID;
                IDToMemberType[CurID] = MemberType;
            }

            this->MemberVec.push_back(make_pair(TypeIDFieldName, TypeIDFieldType));
            this->MemberMap[TypeIDFieldName] = TypeIDFieldType;

            // We now have a list of fields we need in the message type
            u32 FieldID = 0;
            map<pair<ExprTypeRef, u32>, string> TypeCountToField;
            for (auto const TypeCount : MaxOccCount) {
                for (u32 i = 0; i < TypeCount.second; ++i) {
                    string MemberName = ((string)"Field_" + to_string(FieldID++));
                    this->MemberVec.push_back(make_pair(MemberName, TypeCount.first));
                    this->MemberMap[MemberName] = TypeCount.first;
                    TypeCountToField[make_pair(TypeCount.first, i)] = MemberName;
                }
            }

            // Construct the mapping from record fields to my fields and vice-versa
            for (auto const& MemberType : MemberTypes) {
                auto MemID = MemberTypeToID[MemberType];
                map<ExprTypeRef, u32> OccCounts;
                auto const& MemVec = MemberType->SAs<ExprRecordType>()->GetMemberVec();
                for (auto const& Mem : MemVec) {
                    if (OccCounts.find(Mem.second) == OccCounts.end()) {
                        OccCounts[Mem.second] = 0;
                    }
                    auto TypeCount = make_pair(Mem.second, OccCounts[Mem.second]++);
                    MemberFieldToField[MemID][Mem.first] = TypeCountToField[TypeCount];
                    FieldToMemberField[MemID][TypeCountToField[TypeCount]] = Mem.first;
                }
            }
        }

        ExprMessageType::~ExprMessageType()
        {
            // Nothing here
        }

        const set<ExprTypeRef>& ExprMessageType::GetMemberTypes() const
        {
            return MemberTypes;
        }

        const map<ExprTypeRef, u32>& ExprMessageType::GetMemberTypeToID() const
        {
            return MemberTypeToID;
        }

        const map<u32, ExprTypeRef>& ExprMessageType::GetIDToMemberType() const
        {
            return IDToMemberType;
        }

        const map<u32, map<string, string>>& ExprMessageType::GetMemberFieldToField() const
        {
            return MemberFieldToField;
        }

        const map<u32, map<string, string>>& ExprMessageType::GetFieldToMemberField() const
        {
            return FieldToMemberField;
        }

        const string& ExprMessageType::GetTypeIDFieldName() const
        {
            return TypeIDFieldName;
        }

        const ExprTypeRef& ExprMessageType::GetTypeIDFieldType() const
        {
            return TypeIDFieldType;
        }

        const u32 ExprMessageType::GetTypeIDForMemberType(const ExprTypeRef& MemType) const
        {
            auto it = MemberTypeToID.find(MemType);
            if (it == MemberTypeToID.end()) {
                throw ESMCError((string)"Type \"" + MemType->ToString() + " \" does not " + 
                                "seem to be a member of message type");
            }
            return it->second;
        }

        const ExprTypeRef& ExprMessageType::GetMemberTypeForTypeID(u32 TypeID) const
        {
            auto it = IDToMemberType.find(TypeID);
            if (it == IDToMemberType.end()) {
                throw ESMCError((string)"TypeID " + to_string(TypeID) + " not a valid type id for " + 
                                "message type");
            }
            return it->second;
        }

        const string& ExprMessageType::MapFromMemberField(const ExprTypeRef& MemberType,
                                                          const string& FieldName) const
        {
            auto it = MemberTypeToID.find(MemberType);
            if (it == MemberTypeToID.end()) {
                throw ESMCError((string)"Type \"" + MemberType->ToString() + " \" does not " + 
                                "seem to be a member of message type");                
            }
            auto TypeID = it->second;

            auto const& FieldMap = MemberFieldToField.find(TypeID)->second;
            auto it2 = FieldMap.find(FieldName);
            if (it2 == FieldMap.end()) {
                throw ESMCError((string)"Field \"" + FieldName + "\" invalid for member type");
            }
            return it2->second;
        }

        const string& ExprMessageType::MapToMemberField(u32 TypeID,
                                                        const string& FieldName) const
        {
            auto it = FieldToMemberField.find(TypeID);
            if (it == FieldToMemberField.end()) {
                throw ESMCError((string)"TypeID " + to_string(TypeID) + " not a valid type id for " + 
                                "message type");
            }
            auto const& Map = it->second;
            auto it2 = Map.find(FieldName);
            if (it2 == Map.end()) {
                throw ESMCError((string)"Field Name \"" + FieldName + "\" does not " + 
                                "seem to be a member of message type");
            }
            return it2->second;
        }

        void ExprMessageType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, Name);
            for (auto const& Member : MemberTypes) {
                boost::hash_combine(HashCode, Member->Hash());
            }
        }

        string ExprMessageType::ToString() const
        {
            return Name;
        }

        i32 ExprMessageType::Compare(const ExprTypeBase& Other) const
        {
            if (Other.As<ExprBoolType>() != nullptr ||
                Other.As<ExprIntType>() != nullptr ||
                Other.As<ExprRangeType>() != nullptr ||
                Other.As<ExprEnumType> () != nullptr ||
                Other.As<ExprSymmetricType>() != nullptr ||
                Other.As<ExprFuncType> () != nullptr || 
                Other.As<ExprArrayType>() != nullptr ||
                Other.As<ExprParametricType>() != nullptr || 
                Other.As<ExprFieldAccessType>() != nullptr) {
                return 1;
            }
            if (Other.As<ExprRecordType>() != nullptr && 
                Other.As<ExprMessageType>() == nullptr) {
                return 1;
            }
            auto OtherAsMsg = Other.As<ExprMessageType>();
            if (OtherAsMsg == nullptr) {
                return -1;
            }

            if (Name < OtherAsMsg->Name) {
                return -1;
            } else if (Name > OtherAsMsg->Name) {
                return 1;
            } else {
                return 0;
            }
        }
        
    } /* end namespace Exprs */
} /* end namespace ESMC */

// 
// ExprTypes.cpp ends here
