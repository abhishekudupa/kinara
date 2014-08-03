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
                ValueType->As<ExprFuncType>() != nullptr) {
                throw ESMCError((string)"Array indices and values cannot be functions");
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
                if (NTPair.second->As<ExprFuncType>() != nullptr) {
                    throw ESMCError("Record members cannot be functions types");
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

        ExprUnionType::ExprUnionType(const string& Name,
                                     const set<ExprTypeRef>& MemberTypes)
            : ExprTypeBase(), Name(Name), MemberTypes(MemberTypes)
        {
            for (auto const& Type : MemberTypes) {
                if (!Type->Is<ExprRecordType>()) {
                    throw ESMCError((string)"Only record types can be members of " +
                                    "union types. The type " + Type->ToString() + 
                                    " is not a record type, while creating union " + 
                                    "type with name \"" + Name + "\"");
                }
            }

            // We make the union record type now
            // Count the number of instances of each type in each record type
            map<ExprTypeRef, u32> TypeOccurences;
            vector<ExprTypeRef> MemberTypeVec;
            map<pair<ExprTypeRef, u32>, string> TypeOccToFieldMap;

            for (auto const& Type : MemberTypes) {
                map<ExprTypeRef, u32> CurrentTypeOccurences;
                auto TypeAsRec = Type->template SAs<ExprRecordType>();
                auto const& Members = TypeAsRec->GetMemberVec();
                for (auto const& Member : Members) {
                    if (CurrentTypeOccurences.find(Member.second) == 
                        CurrentTypeOccurences.end()) {
                        MemberTypeVec.push_back(Member.second);
                        CurrentTypeOccurences[Member.second] = 1;
                    } else {
                        CurrentTypeOccurences[Member.second]++;
                    }
                }
                
                for (auto const& Occ : CurrentTypeOccurences) {
                    if (TypeOccurences.find(Occ.first) == TypeOccurences.end()) {
                        TypeOccurences[Occ.first] = Occ.second;
                    } else {
                        auto CurMaxOccur = TypeOccurences[Occ.first];
                        if (Occ.second > CurMaxOccur) {
                            TypeOccurences[Occ.first] = Occ.second;
                        }
                    }
                }
            }

            u32 FieldID = 0;
            // Make the field types
            for (auto const& Type : MemberTypeVec) {
                const u32 NumOccurences = TypeOccurences[Type];
                for (u32 i = 1; i <= NumOccurences; ++i) {
                    string FieldName = "Field_" + to_string(FieldID++);
                    FieldVec.push_back(make_pair(FieldName, Type));
                    FieldMap[FieldName] = Type;
                    TypeOccToFieldMap[make_pair(Type, i)] = FieldName;
                }
            }

            // Map the field names of the members to the field
            // names of the union
            
            for (auto const& Type : MemberTypes) {
                map<ExprTypeRef, u32> OccMap;
                MemberFieldToFieldMap[Type] = map<string, string>();

                auto TypeAsRec = Type->template SAs<ExprRecordType>();
                auto const& TypeMembers = TypeAsRec->GetMemberVec();

                for (auto const& Member : TypeMembers) {
                    if (OccMap.find(Member.second) == OccMap.end()) {
                        OccMap[Member.second] = 1;
                    } else {
                        OccMap[Member.second]++;
                    }
                    auto TypeOccKey = make_pair(Member.second, OccMap[Member.second]);
                    MemberFieldToFieldMap[Type][Member.first] = 
                        TypeOccToFieldMap[TypeOccKey];
                }
            }
        }

        ExprUnionType::~ExprUnionType()
        {
            // Nothing here
        }

        const string& ExprUnionType::GetName() const
        {
            return Name;
        }

        const set<ExprTypeRef>& ExprUnionType::GetMemberTypes() const
        {
            return MemberTypes;
        }

        bool ExprUnionType::IsMember(const ExprTypeRef& Type) const
        {
            return (MemberTypes.find(Type) != MemberTypes.end());
        }

        const ExprTypeRef& ExprUnionType::GetTypeForMemberField(const ExprTypeRef& MemberType, 
                                                                const string& MemberField) const
        {
            if (MemberTypes.find(MemberType) == MemberTypes.end()) {
                return ExprTypeRef::NullPtr;
            }
            
            auto it1 = MemberFieldToFieldMap.find(MemberType);
            auto const& Field2FieldMap = it1->second;
            auto it = Field2FieldMap.find(MemberField);
            if (it == Field2FieldMap.end()) {
                return ExprTypeRef::NullPtr;
            } else {
                auto it2 = FieldMap.find(it->second);
                return it2->second;
            }
        }

        string ExprUnionType::ToString() const
        {
            string Retval = "(UnionType : (";
            bool First = true;
            for (auto const& Type : MemberTypes) {
                if (!First) {
                    Retval += " ";
                }
                First = false;
                Retval += Type->SAs<ExprRecordType>()->GetName();
            }
            Retval += "))";
            return Retval;
        }

        i32 ExprUnionType::Compare(const ExprTypeBase& Other) const
        {
            if (Other.As<ExprBoolType>() != nullptr ||
                Other.As<ExprIntType>() != nullptr ||
                Other.As<ExprRangeType>() != nullptr ||
                Other.As<ExprEnumType> () != nullptr ||
                Other.As<ExprSymmetricType>() != nullptr ||
                Other.As<ExprFuncType> () != nullptr || 
                Other.As<ExprArrayType>() != nullptr ||
                Other.As<ExprRecordType>() != nullptr ||
                Other.As<ExprParametricType>() != nullptr || 
                Other.As<ExprFieldAccessType>() != nullptr) {
                return 1;
            }

            if (Other.As<ExprUnionType>() == nullptr) {
                return -1;
            } else {
                auto OtherAsUnion = Other.SAs<ExprUnionType>();
                if (OtherAsUnion->Name < Name) {
                    return 1;
                } else if (OtherAsUnion->Name > Name) {
                    return -1;
                } else {
                    return 0;
                }
            }
        }

        vector<string> ExprUnionType::GetElements() const
        {
            throw InternalError((string)"ExprUnionType::GetElements() should never " + 
                                "have been called.\nAt: " + __FILE__ + ":" + 
                                to_string(__LINE__));
        }

        u32 ExprUnionType::GetByteSize() const
        {
            u32 Retval = 0;
            for (auto const& Field : FieldVec) {
                Retval += Field.second->GetByteSize();
            }
            return Retval;
        }

        void ExprUnionType::ComputeHashValue() const
        {
            HashCode = 0;
            for (auto const& Type : MemberTypes) {
                boost::hash_combine(HashCode, Type->Hash());
            }
        }

        ExprUFAType::ExprUFAType(const ExprTypeRef& UnionMemberType)
            : ExprTypeBase(), UnionMemberType(UnionMemberType)
        {
            if (!UnionMemberType->Is<ExprRecordType>()) {
                throw ESMCError((string)"Union Field Access Types must be " + 
                                "constructed with a record type as member");
            }
        }

        ExprUFAType::~ExprUFAType()
        {
            // Nothing here
        }

        const ExprTypeRef& ExprUFAType::GetUnionMemberType() const
        {
            return UnionMemberType;
        }

        string ExprUFAType::ToString() const
        {
            return ((string)"(UnionFAType : " + 
                    UnionMemberType->SAs<ExprRecordType>()->ToString() + ")");
        }

        i32 ExprUFAType::Compare(const ExprTypeBase& Other) const
        {
            if (Other.As<ExprBoolType>() != nullptr ||
                Other.As<ExprIntType>() != nullptr ||
                Other.As<ExprRangeType>() != nullptr ||
                Other.As<ExprEnumType> () != nullptr ||
                Other.As<ExprSymmetricType>() != nullptr ||
                Other.As<ExprFuncType> () != nullptr || 
                Other.As<ExprArrayType>() != nullptr ||
                Other.As<ExprRecordType>() != nullptr ||
                Other.As<ExprParametricType>() != nullptr || 
                Other.As<ExprFieldAccessType>() != nullptr || 
                Other.As<ExprUnionType>() != nullptr) {
                return 1;
            }

            if (Other.As<ExprUFAType>() == nullptr) {
                return -1;
            } else {
                auto OtherAsUFA = Other.SAs<ExprUFAType>();
                return UnionMemberType->Compare(*(OtherAsUFA->UnionMemberType));
            }
        }

        vector<string> ExprUFAType::GetElements() const
        {
            throw InternalError((string)"ExprUFAType::GetElements() should never " + 
                                "have been called.\nAt: " + __FILE__ + ":" + 
                                to_string(__LINE__));
        }

        u32 ExprUFAType::GetByteSize() const
        {
            throw InternalError((string)"ExprUFAType::GetByteSize() should never " + 
                                "have been called.\nAt: " + __FILE__ + ":" + 
                                to_string(__LINE__));
        }

        void ExprUFAType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, "UFAType");
            boost::hash_combine(HashCode, UnionMemberType->Hash());
        }        
        
    } /* end namespace Exprs */
} /* end namespace ESMC */

// 
// ExprTypes.cpp ends here
