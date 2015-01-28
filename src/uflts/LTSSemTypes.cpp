// LTSSemTypes.cpp ---
//
// Filename: LTSSemTypes.cpp
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

#include "LTSSemTypes.hpp"
#include "../utils/UIDGenerator.hpp"
#include "../utils/SizeUtils.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>

namespace ESMC {
    namespace LTS {

        UIDGenerator TypeBase::TypeUIDGen(1);

        TypeExtensionBase::TypeExtensionBase()
        {
            // Nothing here
        }

        TypeExtensionBase::~TypeExtensionBase()
        {
            // Nothing here
        }

        TypeBase::TypeBase()
            : TypeID(-1), HashValid(false), LastExtension(nullptr)
        {
            // Nothing here
        }

        TypeBase::~TypeBase()
        {
            PurgeAllExtensions();
        }

        u64 TypeBase::Hash() const
        {
            if (HashValid) {
                return HashCode;
            } else {
                ComputeHashValue();
                HashValid = true;
                return HashCode;
            }
        }

        bool TypeBase::Equals(const TypeBase& Other) const
        {
            return (this->Compare(Other) == 0);
        }

        bool TypeBase::LT(const TypeBase& Other) const
        {
            return (this->Compare(Other) < 0);
        }

        i64 TypeBase::GetTypeID() const
        {
            return TypeID;
        }

        i64 TypeBase::SetTypeID() const
        {
            TypeID = TypeUIDGen.GetUID();
            return TypeID;
        }

        i64 TypeBase::GetOrSetTypeID() const
        {
            if (TypeID == -1) {
                return SetTypeID();
            } else {
                return GetTypeID();
            }
        }

        ScalarType::ScalarType()
            : TypeBase()
        {
            // Nothing here
        }

        ScalarType::~ScalarType()
        {
            // Nothing here
        }

        BooleanType::BooleanType()
            : ScalarType()
        {
            // Nothing here
        }

        BooleanType::~BooleanType()
        {
            // Nothing here
        }

        string BooleanType::ToString(u32 Verbosity) const
        {
            return "(BooleanType)";
        }

        i32 BooleanType::Compare(const TypeBase& Other) const
        {
            if (Other.As<BooleanType>() == nullptr) {
                return -1;
            } else {
                return 0;
            }
        }

        vector<string> BooleanType::GetElements() const
        {
            vector<string> Retval = { "true", "false" };
            return Retval;
        }

        vector<string> BooleanType::GetElementsNoUndef() const
        {
            vector<string> Retval = { "true", "false" };
            return Retval;
        }

        void BooleanType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, "BooleanType");
        }

        u32 BooleanType::GetByteSize() const
        {
            return 1;
        }

        u32 BooleanType::GetCardinality() const
        {
            return 2;
        }

        u32 BooleanType::GetCardinalityNoUndef() const
        {
            return 2;
        }

        i64 BooleanType::ConstToVal(const string& ConstVal) const
        {
            if (ConstVal == "true") {
                return 1;
            } else {
                return 0;
            }
        }

        string BooleanType::ValToConst(i64 Val) const
        {
            if (Val == 0) {
                return "false";
            } else {
                return "true";
            }
        }

        string BooleanType::GetClearValue() const
        {
            return "false";
        }

        IntegerType::IntegerType()
            : ScalarType()
        {
            // Nothing here
        }

        IntegerType::~IntegerType()
        {
            // Nothing here
        }

        void IntegerType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, "IntegerType");
        }

        string IntegerType::ToString(u32 Verbosity) const
        {
            return "(IntegerType)";
        }

        i32 IntegerType::Compare(const TypeBase& Other) const
        {
            auto OtherAsPtr = &Other;

            if (OtherAsPtr->As<BooleanType>() != nullptr) {
                return 1;
            }

            auto OtherAsInt = OtherAsPtr->As<IntegerType>();
            if (OtherAsInt == nullptr) {
                return -1;
            } else {
                auto OtherAsRange = OtherAsPtr->As<RangeType>();
                if (OtherAsRange != nullptr) {
                    return -1;
                } else {
                    return 0;
                }
            }
        }

        vector<string> IntegerType::GetElements() const
        {
            throw ESMCError((string)"Cannot GetElements() on unbounded type IntegerType");
        }

        vector<string> IntegerType::GetElementsNoUndef() const
        {
            return GetElements();
        }

        u32 IntegerType::GetByteSize() const
        {
            throw InternalError((string)"IntegerType::GetByteSize() should never have been " +
                                "called.\nAt: " + __FILE__ + ":" + to_string(__LINE__));
        }

        u32 IntegerType::GetCardinality() const
        {
            throw ESMCError((string)"Cannot get cardinality of unbounded type IntegerType");
        }

        u32 IntegerType::GetCardinalityNoUndef() const
        {
            throw ESMCError((string)"Cannot get cardinality of unbounded type IntegerType");
        }

        i64 IntegerType::ConstToVal(const string& ConstVal) const
        {
            return boost::lexical_cast<i64>(ConstVal);
        }

        string IntegerType::ValToConst(i64 Val) const
        {
            return to_string(Val);
        }

        string IntegerType::GetClearValue() const
        {
            return "0";
        }

        // Inclusive range
        RangeType::RangeType(i64 RangeLow, i64 RangeHigh)
            : IntegerType(),
              RangeLow(RangeLow), RangeHigh(RangeHigh),
              Size(RangeHigh - RangeLow + 1)
        {
            if (RangeLow > RangeHigh) {
                throw ESMCError((string)"Negative size specified for Range type");
            }
        }

        RangeType::~RangeType()
        {
            // Nothing here
        }

        i64 RangeType::GetLow() const
        {
            return RangeLow;
        }

        i64 RangeType::GetHigh() const
        {
            return RangeHigh;
        }

        u64 RangeType::GetSize() const
        {
            return Size;
        }

        void RangeType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, RangeLow);
            boost::hash_combine(HashCode, RangeHigh);
        }

        u32 RangeType::GetByteSize() const
        {
            return BytesForRange(RangeHigh - RangeLow + 1);
        }

        string RangeType::ToString(u32 Verbosity) const
        {
            return ((string)"(Range [" + to_string(RangeLow) +
                    "-" + to_string(RangeHigh) + "])");
        }

        i32 RangeType::Compare(const TypeBase& Other) const
        {
            auto OtherAsPtr = &Other;

            if (OtherAsPtr->As<BooleanType>() != nullptr) {
                return 1;
            }

            auto OtherAsRange = OtherAsPtr->As<RangeType>();

            if (OtherAsRange == nullptr) {
                if (OtherAsPtr->As<IntegerType>() != nullptr) {
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

        vector<string> RangeType::GetElements() const
        {
            vector<string> Retval;
            for(i64 i = RangeLow; i <= RangeHigh; ++i) {
                Retval.push_back(to_string(i));
            }
            return Retval;
        }

        vector<string> RangeType::GetElementsNoUndef() const
        {
            return GetElements();
        }

        u32 RangeType::GetCardinality() const
        {
            return (RangeHigh - RangeLow + 1);
        }

        u32 RangeType::GetCardinalityNoUndef() const
        {
            return (RangeHigh - RangeLow + 1);
        }

        i64 RangeType::ConstToVal(const string& ConstVal) const
        {
            return boost::lexical_cast<i64>(ConstVal);
        }

        string RangeType::ValToConst(i64 Val) const
        {
            return to_string(Val);
        }

        string RangeType::GetClearValue() const
        {
            return to_string(RangeLow);
        }

        EnumType::EnumType(const string& Name,
                           const set<string>& Members)
            : ScalarType(), Name(Name), Members(Members),
              MemberVec(Members.begin(), Members.end())
        {
            // Nothing here
        }

        EnumType::~EnumType()
        {
            // Nothing here
        }

        const string& EnumType::GetName() const
        {
            return Name;
        }

        const set<string>& EnumType::GetMembers() const
        {
            return Members;
        }

        // We handle both qualified and unqualified enum names here
        bool EnumType::IsMember(const string& MemberName) const
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

        u32 EnumType::GetMemberIdx(const string& MemberName) const
        {
            vector<string> SplitVec;
            boost::algorithm::split(SplitVec, MemberName,
                                    boost::algorithm::is_any_of(":"),
                                    boost::algorithm::token_compress_on);
            string EnumName;
            if (SplitVec.size() == 2) {
                auto EnumName = boost::algorithm::trim_copy(SplitVec[0]);
                if (EnumName != Name) {
                    throw ESMCError((string)"Invalid Enum Member Index requested \"" +
                                    MemberName + "\"");
                } else {
                    auto MemberName = boost::algorithm::trim_copy(SplitVec[1]);
                    for (u32 i = 0; i < MemberVec.size(); ++i) {
                        if (MemberName == MemberVec[i]) {
                            return i;
                        }
                    }
                    throw ESMCError((string)"Invalid Enum Member Index requested \"" +
                                    MemberName + "\"");
                }
            } else if (SplitVec.size() == 1) {
                auto MemberName = boost::algorithm::trim_copy(SplitVec[0]);
                for (u32 i = 0; i < MemberVec.size(); ++i) {
                    if (MemberName == MemberVec[i]) {
                        return i;
                    }
                }
                throw ESMCError((string)"Invalid Enum Member Index requested \"" +
                                MemberName + "\"");

            } else {
                throw ESMCError((string)"Invalid Enum Member Index requested \"" +
                                MemberName + "\"");
            }
        }

        string EnumType::ToString(u32 Verbosity) const
        {
            ostringstream sstr;
            if (Verbosity == 0) {
                sstr << "(Enum " << Name << ")";
            } else {
                sstr << "(Enum " << Name << " (";
                bool First = true;
                for (auto const& Member : MemberVec) {
                    if (!First) {
                        sstr << " ";
                    }
                    First = false;
                    sstr << Member;
                }
                sstr << "))";
            }
            return sstr.str();
        }

        void EnumType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, Name);
            for (auto const& Mem : Members) {
                boost::hash_combine(HashCode, Mem);
            }
        }

        u32 EnumType::GetByteSize() const
        {
            return BytesForRange(Members.size());
        }

        u32 EnumType::GetCardinality() const
        {
            return Members.size();
        }

        u32 EnumType::GetCardinalityNoUndef() const
        {
            return Members.size();
        }

        i32 EnumType::Compare(const TypeBase& Other) const
        {
            auto OtherPtr = &Other;
            if (OtherPtr->As<BooleanType>() != nullptr ||
                OtherPtr->As<IntegerType>() != nullptr ||
                OtherPtr->As<RangeType>() != nullptr) {
                return 1;
            }
            if (OtherPtr->As<EnumType>() == nullptr) {
                return -1;
            }
            auto OtherAsEnum = OtherPtr->SAs<EnumType>();

            if (OtherAsEnum->Name > Name) {
                return -1;
            } else if(OtherAsEnum->Name < Name) {
                return 1;
            } else {
                return 0;
            }
        }

        vector<string> EnumType::GetElements() const
        {
            return (vector<string>(Members.begin(), Members.end()));
        }

        vector<string> EnumType::GetElementsNoUndef() const
        {
            return GetElements();
        }

        i64 EnumType::ConstToVal(const string& ConstVal) const
        {
            return GetMemberIdx(ConstVal);
        }

        string EnumType::ValToConst(i64 Val) const
        {
            return MemberVec[Val];
        }

        string EnumType::GetClearValue() const
        {
            return MemberVec[0];
        }

        SymmetricType::SymmetricType(const string& Name, u32 Size)
            : ScalarType(), Name(Name), Size(Size), Members(Size + 1)
        {
            Members[0] = Name + "::undef";
            MemberSet.insert(Members[0]);
            for(u32 i = 1; i <= Size; ++i) {
                Members[i] = Name + "::" + to_string(i-1);
                MemberSet.insert(Members[i]);
            }

            MembersNoUndef = vector<string>(next(Members.begin()), Members.end());
        }

        SymmetricType::~SymmetricType()
        {
            // Nothing here
        }

        const string& SymmetricType::GetName() const
        {
            return Name;
        }

        u32 SymmetricType::GetSize() const
        {
            return Size;
        }

        const vector<string>& SymmetricType::GetMembers() const
        {
            return Members;
        }

        const string& SymmetricType::GetMember(u32 Index) const
        {
            assert (Index < Members.size());
            return Members[Index];
        }

        const bool SymmetricType::IsMember(const string& Value) const
        {
            return (MemberSet.find(Value) != MemberSet.end());
        }

        u32 SymmetricType::GetMemberIdx(const string& MemberName) const
        {
            for (u32 i = 0; i < Members.size(); ++i) {
                if (Members[i] == MemberName) {
                    return i;
                }
            }
            throw ESMCError((string)"Invalid symmetric type member index requested \"" +
                            MemberName + "\"");
        }

        void SymmetricType::SetIndex(u32 Index) const
        {
            this->Index = Index;
        }

        u32 SymmetricType::GetIndex() const
        {
            return Index;
        }

        string SymmetricType::ToString(u32 Verbosity) const
        {
            return (string)"(SymType " + Name + " " +
                to_string(Size) + ")";
        }

        void SymmetricType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, Name);
            boost::hash_combine(HashCode, Size);
        }

        u32 SymmetricType::GetByteSize() const
        {
            return BytesForRange(Members.size());
        }

        u32 SymmetricType::GetCardinality() const
        {
            return MemberSet.size();
        }

        u32 SymmetricType::GetCardinalityNoUndef() const
        {
            return MembersNoUndef.size();
        }

        i32 SymmetricType::Compare(const TypeBase& Other) const
        {
            auto OtherAsPtr = &Other;
            if (OtherAsPtr->As<BooleanType>() != nullptr ||
                OtherAsPtr->As<IntegerType>() != nullptr ||
                OtherAsPtr->As<RangeType>() != nullptr ||
                OtherAsPtr->As<EnumType>() != nullptr) {
                return 1;
            }
            auto OtherAsSym = OtherAsPtr->As<SymmetricType>();
            if (OtherAsSym == nullptr) {
                return -1;
            }
            if (OtherAsSym->Name > Name) {
                return -1;
            } else if (OtherAsSym->Name < Name) {
                return 1;
            }
            return 0;
        }

        vector<string> SymmetricType::GetElements() const
        {
            return Members;
        }

        vector<string> SymmetricType::GetElementsNoUndef() const
        {
            return MembersNoUndef;
        }

        i64 SymmetricType::ConstToVal(const string& ConstVal) const
        {
            return (GetMemberIdx(ConstVal));
        }

        string SymmetricType::ValToConst(i64 Val) const
        {
            return Members[Val];
        }

        string SymmetricType::GetClearValue() const
        {
            return (Name + "::undef");
        }

        static inline string MangleName(const string& Name,
                                        const vector<TypeRef>& Args)
        {
            string Retval = Name;
            for (auto const& Arg : Args) {
                Retval += ((string)"@" + to_string(Arg->GetTypeID()));
            }
            return Retval;
        }

        FuncType::FuncType(const string& Name,
                           const vector<TypeRef>& DomainTypes,
                           const TypeRef& RangeType)
            : TypeBase(),
              Name(Name), MangledName(MangleName(Name, DomainTypes)),
              ArgTypes(DomainTypes), EvalType(RangeType)
        {
            for(auto const& Arg : DomainTypes) {
                if (Arg->As<FuncType>() != nullptr) {
                    throw ESMCError("Function types cannot have function types as params");
                }
                if (!Arg->Is<ScalarType>()) {
                    throw ESMCError((string)"Only function types with scalar domain types " +
                                    "are currenly supported");
                }
            }
        }

        FuncType::~FuncType()
        {
            // Nothing here
        }

        const string& FuncType::GetName() const
        {
            return Name;
        }

        const string& FuncType::GetMangledName() const
        {
            return MangledName;
        }

        const vector<TypeRef>& FuncType::GetArgTypes() const
        {
            return ArgTypes;
        }

        const TypeRef& FuncType::GetEvalType() const
        {
            return EvalType;
        }

        void FuncType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, Name);
            for (auto const& Arg : ArgTypes) {
                boost::hash_combine(HashCode, Arg->Hash());
            }
        }

        string FuncType::ToString(u32 Verbosity) const
        {
            string Retval = (string)"(Func " + Name + " ";
            for (auto const& Arg : ArgTypes) {
                Retval += Arg->ToString(Verbosity) + " -> ";
            }
            Retval += (EvalType->ToString(Verbosity) + ")");
            return Retval;
        }

        i32 FuncType::Compare(const TypeBase& Other) const
        {
            auto OtherAsPtr = &Other;
            if (OtherAsPtr->As<BooleanType>() != nullptr ||
                OtherAsPtr->As<IntegerType>() != nullptr ||
                OtherAsPtr->As<RangeType> () != nullptr ||
                OtherAsPtr->As<EnumType>() != nullptr ||
                OtherAsPtr->As<SymmetricType>() != nullptr) {
                return 1;
            }

            auto OtherAsFunc = OtherAsPtr->As<FuncType>();

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

        vector<string> FuncType::GetElements() const
        {
            throw ESMCError((string)"Cannot GetElements() of a function type");
        }

        vector<string> FuncType::GetElementsNoUndef() const
        {
            throw ESMCError((string)"Cannot GetElements() of a function type");
        }

        u32 FuncType::GetByteSize() const
        {
            // We return the byte size of the range multiplied
            // by the product of the domains
            u32 Retval = EvalType->GetByteSize();
            for (auto const& ArgType : ArgTypes) {
                Retval *= ArgType->GetElements().size();
            }
            return Retval;
        }

        string FuncType::GetClearValue() const
        {
            throw ESMCError((string)"FuncType::GetClearValue() should never have " +
                            "been called");
        }

        u32 FuncType::GetCardinality() const
        {
            throw ESMCError((string)"Cannot GetCardinality() of a function type");
        }

        u32 FuncType::GetCardinalityNoUndef() const
        {
            throw ESMCError((string)"Cannot GetCardinality() of a function type");
        }

        ArrayType::ArrayType(const TypeRef& IndexType,
                             const TypeRef& ValueType)
            : TypeBase(), IndexType(IndexType), ValueType(ValueType)
        {
            if (IndexType->As<FuncType>() != nullptr ||
                ValueType->As<FuncType>() != nullptr ||
                IndexType->As<ScalarType>() == nullptr ||
                ValueType->As<ParametricType>() != nullptr ||
                ValueType->As<FieldAccessType>() != nullptr) {
                throw ESMCError((string)"Array indices and values cannot be functions, " +
                                "further, indices must be scalar and values cannot be " +
                                "field access types or parametric types");
            }
        }

        ArrayType::~ArrayType()
        {
            // Nothing here
        }

        const TypeRef& ArrayType::GetIndexType() const
        {
            return IndexType;
        }

        const TypeRef& ArrayType::GetValueType() const
        {
            return ValueType;
        }

        u32 ArrayType::GetOffset(u32 ElemIdx) const
        {
            auto ElemSize = ValueType->GetByteSize();
            ElemSize = Align(ElemSize, ElemSize);
            return (ElemSize * ElemIdx);
        }

        TypeRef ArrayType::GetBaseValueType() const
        {
            auto Retval = ValueType;
            while (Retval->Is<ArrayType>()) {
                Retval = Retval->SAs<ArrayType>()->GetValueType();
            }
            return Retval;
        }

        u32 ArrayType::GetLevelsOfIndex() const
        {
            u32 Retval = 1;
            auto ValType = ValueType;
            while (ValType->Is<ArrayType>()) {
                Retval++;
                ValType = ValType->SAs<ArrayType>()->GetValueType();
            }
            return Retval;
        }

        string ArrayType::ToString(u32 Verbosity) const
        {
            return ((string)"(Array " + IndexType->ToString(Verbosity) +
                    " -> " + ValueType->ToString(Verbosity) + ")");
        }

        i32 ArrayType::Compare(const TypeBase& Other) const
        {
            auto OtherAsPtr = &Other;
            if (OtherAsPtr->As<BooleanType>() != nullptr ||
                OtherAsPtr->As<IntegerType>() != nullptr ||
                OtherAsPtr->As<RangeType>() != nullptr ||
                OtherAsPtr->As<EnumType>() != nullptr ||
                OtherAsPtr->As<SymmetricType>() != nullptr ||
                OtherAsPtr->As<FuncType>() != nullptr) {
                return 1;
            }

            auto OtherAsArr = OtherAsPtr->As<ArrayType>();

            if (OtherAsArr == nullptr) {
                return -1;
            }

            auto Cmp = IndexType->Compare(*(OtherAsArr->IndexType));
            if (Cmp != 0) {
                return Cmp;
            }
            return ValueType->Compare(*(OtherAsArr->ValueType));
        }

        vector<string> ArrayType::GetElements() const
        {
            throw ESMCError((string)"Cannot get elements of non-scalar type");
        }

        vector<string> ArrayType::GetElementsNoUndef() const
        {
            throw ESMCError((string)"Cannot get elements of non-scalar type");
        }

        void ArrayType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, IndexType->Hash());
            boost::hash_combine(HashCode, ValueType->Hash());
        }

        u32 ArrayType::GetByteSize() const
        {
            u32 Retval = ValueType->GetByteSize();
            Retval = Align(Retval, Retval);
            return (Retval * IndexType->GetElementsNoUndef().size());
        }

        u32 ArrayType::GetCardinality() const
        {
            throw ESMCError((string)"Cannot get elements of non-scalar type");
        }

        u32 ArrayType::GetCardinalityNoUndef() const
        {
            throw ESMCError((string)"Cannot get elements of non-scalar type");
        }

        string ArrayType::GetClearValue() const
        {
            throw ESMCError((string)"ArrayType::GetClearValue() should never have " +
                            "been called");
        }

        RecordType::RecordType(const string& Name,
                               const vector<pair<string, TypeRef>>& Members)
            : TypeBase(), Name(Name), MemberVec(Members),
              ContainsUnboundedType(false), FieldOffsetsComputed(false)
        {
            for (auto const& NTPair : Members) {
                if (NTPair.second->As<FuncType>() != nullptr ||
                    NTPair.second->As<FieldAccessType>() != nullptr ||
                    NTPair.second->As<ParametricType>() != nullptr) {
                    throw ESMCError((string)"Record members cannot be functions types or " +
                                    "field access types, or parametric types");
                }
                if (NTPair.second->As<IntegerType>() != nullptr &&
                    NTPair.second->As<RangeType>() == nullptr) {
                    ContainsUnboundedType = true;
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

        void RecordType::ComputeFieldOffsets() const
        {
            for (auto const& NTPair : MemberVec) {
                if (NTPair.second->As<IntegerType>() != nullptr &&
                    NTPair.second->As<RangeType>() == nullptr) {
                    ContainsUnboundedType = true;
                }
            }
            if (!ContainsUnboundedType) {
                u32 CurOffset = 0;
                for (auto const& Member : MemberVec) {
                    auto CurSize = Member.second->GetByteSize();
                    CurOffset = Align(CurOffset, CurSize);
                    FieldOffsets[Member.first] = CurOffset;
                    CurOffset += CurSize;
                }
            }
            FieldOffsetsComputed = true;
        }

        RecordType::RecordType()
            : TypeBase(), ContainsUnboundedType(false),
              FieldOffsetsComputed(false)
        {
            // Nothing here
        }

        RecordType::~RecordType()
        {
            // Nothing here
        }

        const string& RecordType::GetName() const
        {
            return Name;
        }

        const map<string, TypeRef>& RecordType::GetMemberMap() const
        {
            return MemberMap;
        }

        const vector<pair<string, TypeRef>>&
            RecordType::GetMemberVec() const
        {
            return MemberVec;
        }

        const TypeRef& RecordType::GetTypeForMember(const string& MemberName) const
        {
            auto it = MemberMap.find(MemberName);
            if (it == MemberMap.end()) {
                return TypeRef::NullPtr;
            }
            return it->second;
        }

        u32 RecordType::GetFieldIdx(const string& FieldName) const
        {
            for (u32 i = 0; i < MemberVec.size(); ++i) {
                if (MemberVec[i].first == FieldName) {
                    return i;
                }
            }
            throw ESMCError((string)"Invalid field in request for field index \"" +
                            FieldName + "\"");
        }

        u32 RecordType::GetFieldOffset(const string& FieldName) const
        {
            if (ContainsUnboundedType) {
                throw ESMCError((string)"Record type \"" + Name + "\" contains " +
                                "one or more unbounded types. Field offsets " +
                                "can therefore not be computed");
            }
            if (!FieldOffsetsComputed) {
                ComputeFieldOffsets();
            }
            auto it = FieldOffsets.find(FieldName);
            if (it == FieldOffsets.end()) {
                throw ESMCError((string)"Invalid field in request for field offset \"" +
                                FieldName + "\"");
            }
            return it->second;
        }

        string RecordType::ToString(u32 Verbosity) const
        {
            string Retval;
            if (Verbosity == 0) {
                Retval += "(Rec " + Name + ")";
            } else {
                Retval += "(Rec " + Name + "(";
                bool First = true;
                for (auto const& NTPair : MemberVec) {
                    if (!First) {
                        Retval += " ";
                    }
                    First = false;
                    Retval += ("(" + NTPair.first + " : " +
                               NTPair.second->ToString(Verbosity) + ")");
                }
                Retval += "))";
            }
            return Retval;
        }

        i32 RecordType::Compare(const TypeBase& Other) const
        {
            auto OtherAsPtr = &Other;
            if (OtherAsPtr->As<BooleanType>() != nullptr ||
                OtherAsPtr->As<IntegerType>() != nullptr ||
                OtherAsPtr->As<RangeType>() != nullptr ||
                OtherAsPtr->As<EnumType>() != nullptr ||
                OtherAsPtr->As<SymmetricType>() != nullptr ||
                OtherAsPtr->As<FuncType> () != nullptr ||
                OtherAsPtr->As<ArrayType> () != nullptr) {
                return 1;
            }

            auto OtherAsRec = OtherAsPtr->As<RecordType>();
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

        vector<string> RecordType::GetElements() const
        {
            throw ESMCError((string)"Cannot get elements of non-scalar type");
        }

        vector<string> RecordType::GetElementsNoUndef() const
        {
            throw ESMCError((string)"Cannot get elements of non-scalar type");
        }

        u32 RecordType::GetCardinality() const
        {
            throw ESMCError((string)"Cannot get cardinality of non-scalar type");
        }

        u32 RecordType::GetCardinalityNoUndef() const
        {
            throw ESMCError((string)"Cannot get cardinality of non-scalar type");
        }

        void RecordType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, Name);
            boost::hash_combine(HashCode, MemberVec.size());
        }

        u32 RecordType::GetByteSize() const
        {
            u32 Retval = 0;
            for (auto const& Member : MemberVec) {
                auto CurSize = Member.second->GetByteSize();
                Retval = Align(Retval, CurSize);
                Retval += CurSize;
            }
            // Records are always padded up to 4 bytes
            return Align(Retval, 4);
        }

        string RecordType::GetClearValue() const
        {
            throw ESMCError((string)"RecordType::GetClearValue() should never " +
                            "have been called");
        }

        ParametricType::ParametricType(const TypeRef& BaseType,
                                       const vector<TypeRef>& ParameterTypes)
            : TypeBase(), BaseType(BaseType), ParameterTypes(ParameterTypes)
        {
            if (!BaseType->Is<RecordType>()) {
                throw ESMCError((string)"Only record types " +
                                "can currently be parametrized");
            }

            for (auto const& ParameterType : ParameterTypes) {
                if (ParameterType->As<EnumType>() == nullptr &&
                    ParameterType->As<RangeType>() == nullptr &&
                    ParameterType->As<SymmetricType>() == nullptr) {
                    throw ESMCError((string)"Parameteric types must have enum, range " +
                                    "or symmetric types as type parameters");
                }
            }
        }

        ParametricType::~ParametricType()
        {
            // Nothing here
        }

        const TypeRef& ParametricType::GetBaseType() const
        {
            return BaseType;
        }

        const vector<TypeRef>& ParametricType::GetParameterTypes() const
        {
            return ParameterTypes;
        }

        const string& ParametricType::GetName() const
        {
            return BaseType->SAs<RecordType>()->GetName();
        }

        void ParametricType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, BaseType->Hash());
            for (auto const& ParameterType : ParameterTypes) {
                boost::hash_combine(HashCode, ParameterType->Hash());
            }
        }

        string ParametricType::ToString(u32 Verbosity) const
        {
            string Retval = "(ParamType ";
            for (auto const& ParameterType : ParameterTypes) {
                Retval += (ParameterType->ToString(Verbosity) + " -> ");
            }
            Retval += (BaseType->ToString(Verbosity) + ")");
            return Retval;
        }

        i32 ParametricType::Compare(const TypeBase& Other) const
        {
            auto OtherAsPtr = &Other;

            if (OtherAsPtr->As<BooleanType>() != nullptr ||
                OtherAsPtr->As<IntegerType>() != nullptr ||
                OtherAsPtr->As<RangeType>() != nullptr ||
                OtherAsPtr->As<EnumType>() != nullptr ||
                OtherAsPtr->As<SymmetricType>() != nullptr ||
                OtherAsPtr->As<FuncType>() != nullptr ||
                OtherAsPtr->As<ArrayType>() != nullptr ||
                OtherAsPtr->As<RecordType>() != nullptr) {
                return 1;
            }

            auto OtherAsPar = OtherAsPtr->As<ParametricType>();

            if (OtherAsPar == nullptr) {
                return -1;
            }

            auto Cmp = BaseType->Compare(*OtherAsPar->BaseType);
            if (Cmp != 0) {
                return Cmp;
            }

            const u32 NumParams = ParameterTypes.size();
            const u32 OtherNumParams = OtherAsPar->ParameterTypes.size();
            if (NumParams < OtherNumParams) {
                return -1;
            } else if (NumParams > OtherNumParams) {
                return 1;
            } else {
                for (u32 i = 0; i < NumParams; ++i) {
                    auto Cmp = ParameterTypes[i]->Compare(*(OtherAsPar->ParameterTypes[i]));
                    if (Cmp != 0) {
                        return Cmp;
                    }
                }
                return 0;
            }
        }

        vector<string> ParametricType::GetElements() const
        {
            throw ESMCError((string)"Cannot get elements of non-scalar type");
        }

        vector<string> ParametricType::GetElementsNoUndef() const
        {
            throw ESMCError((string)"Cannot get elements of non-scalar type");
        }

        u32 ParametricType::GetCardinality() const
        {
            throw ESMCError((string)"Cannot get cardinality of non-scalar type");
        }

        u32 ParametricType::GetCardinalityNoUndef() const
        {
            throw ESMCError((string)"Cannot get cardinality of non-scalar type");
        }

        u32 ParametricType::GetByteSize() const
        {
            throw InternalError((string)"ParametricType::GetByteSize() should never " +
                                "have been called.\nAt: " + __FILE__ + ":" + to_string(__LINE__));
        }

        string ParametricType::GetClearValue() const
        {
            throw ESMCError((string)"ParametricType::GetClearValue() should never " +
                            "have been called");
        }

        FieldAccessType::FieldAccessType()
            : TypeBase()
        {
            // Nothing here
        }

        FieldAccessType::~FieldAccessType()
        {
            // Nothing here
        }

        void FieldAccessType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, "FieldAccessType");
        }

        i32 FieldAccessType::Compare(const TypeBase& Other) const
        {
            if (Other.As<BooleanType>() != nullptr ||
                Other.As<IntegerType>() != nullptr ||
                Other.As<RangeType>() != nullptr ||
                Other.As<EnumType> () != nullptr ||
                Other.As<SymmetricType>() != nullptr ||
                Other.As<FuncType> () != nullptr ||
                Other.As<ArrayType>() != nullptr ||
                Other.As<RecordType>() != nullptr ||
                Other.As<ParametricType>() != nullptr) {
                return 1;
            }

            if (Other.As<FieldAccessType>() == nullptr) {
                return -1;
            } else {
                return 0;
            }
        }

        vector<string> FieldAccessType::GetElements() const
        {
            throw ESMCError((string)"Cannot get elements of non-scalar type");
        }

        vector<string> FieldAccessType::GetElementsNoUndef() const
        {
            throw ESMCError((string)"Cannot get elements of non-scalar type");
        }

        u32 FieldAccessType::GetCardinality() const
        {
            throw ESMCError((string)"Cannot get cardinality of non-scalar type");
        }

        u32 FieldAccessType::GetCardinalityNoUndef() const
        {
            throw ESMCError((string)"Cannot get cardinality of non-scalar type");
        }

        string FieldAccessType::ToString(u32 Verbosity) const
        {
            return "(FieldAccessType)";
        }

        u32 FieldAccessType::GetByteSize() const
        {
            throw InternalError((string)"FieldAccessType::GetByteSize() should never " +
                                "have been called.\nAt: " + __FILE__ + ":" + to_string(__LINE__));
        }

        string FieldAccessType::GetClearValue() const
        {
            throw InternalError((string)"FieldAccessType::GetClearValue() " +
                                "should never have been called");
        }

        UnionType::UnionType(const string& Name,
                             const WellOrderedTypeSetT& MemberTypes,
                             const TypeRef& TypeIDFieldType)
            : RecordType(), MemberTypes(MemberTypes),
              TypeIDFieldType(TypeIDFieldType)
        {
            this->Name = Name;
            // Sanity checks
            if (!TypeIDFieldType->Is<RangeType>()) {
                throw ESMCError((string)"UnionType must be instantiated with a " +
                                "subrange type for TypeIDFieldType");
            }

            auto TypeIDTypeAsRange = TypeIDFieldType->As<RangeType>();
            auto Low = TypeIDTypeAsRange->GetLow();
            if (Low != 0) {
                throw ESMCError((string)"Subrange type for UnionType must have low at 0");
            }

            const u32 MaxNumTypes = TypeIDTypeAsRange->GetHigh();
            if (MaxNumTypes < MemberTypes.size()) {
                throw ESMCError((string)"Subrange type not sufficient to accommodate all " +
                                "member types");
            }

            u32 NumTypesSeen = 1;
            // Find the maximum number of occurences of each type
            // in each member type
            WellOrderedTypeMapT<u32> MaxOccCount;

            for (auto const& MemberType : MemberTypes) {
                auto MemberTypeAsRec = MemberType->As<RecordType>();
                if (MemberTypeAsRec == nullptr) {
                    throw ESMCError((string)"Only record types can be members of Message types");
                }
                WellOrderedTypeMapT<u32> CurOccCount;
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
            map<pair<TypeRef, u32>, string> TypeCountToField;
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
                map<TypeRef, u32> OccCounts;
                auto const& MemVec = MemberType->SAs<RecordType>()->GetMemberVec();
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

        UnionType::~UnionType()
        {
            // Nothing here
        }

        const WellOrderedTypeSetT& UnionType::GetMemberTypes() const
        {
            return MemberTypes;
        }

        const WellOrderedTypeMapT<u32>& UnionType::GetMemberTypeToID() const
        {
            return MemberTypeToID;
        }

        const map<u32, TypeRef>& UnionType::GetIDToMemberType() const
        {
            return IDToMemberType;
        }

        const map<u32, map<string, string>>& UnionType::GetMemberFieldToField() const
        {
            return MemberFieldToField;
        }

        const map<u32, map<string, string>>& UnionType::GetFieldToMemberField() const
        {
            return FieldToMemberField;
        }

        const string& UnionType::GetTypeIDFieldName() const
        {
            return TypeIDFieldName;
        }

        const TypeRef& UnionType::GetTypeIDFieldType() const
        {
            return TypeIDFieldType;
        }

        const u32 UnionType::GetTypeIDForMemberType(const TypeRef& MemType) const
        {
            auto TypeAsRec = MemType->As<RecordType>();
            if (TypeAsRec == nullptr) {
                TypeAsRec = MemType->As<ParametricType>()->GetBaseType()->As<RecordType>();
            }
            auto it = MemberTypeToID.find(TypeAsRec);
            if (it == MemberTypeToID.end()) {
                throw ESMCError((string)"Type \"" + MemType->ToString() + " \" does not " +
                                "seem to be a member of union type");
            }
            return it->second;
        }

        const TypeRef& UnionType::GetMemberTypeForTypeID(u32 TypeID) const
        {
            auto it = IDToMemberType.find(TypeID);
            if (it == IDToMemberType.end()) {
                throw ESMCError((string)"TypeID " + to_string(TypeID) +
                                " not a valid type id for message type");
            }
            return it->second;
        }

        const string& UnionType::MapFromMemberField(const TypeRef& MemberType,
                                                    const string& FieldName) const
        {

            auto TypeAsRec = MemberType->As<RecordType>();
            if (TypeAsRec == nullptr) {
                TypeAsRec =
                    MemberType->As<ParametricType>()->GetBaseType()->As<RecordType>();
            }

            auto it = MemberTypeToID.find(TypeAsRec);
            if (it == MemberTypeToID.end()) {
                throw ESMCError((string)"Type \"" + MemberType->ToString() + " \" does not " +
                                "seem to be a member of union type");
            }
            auto TypeID = it->second;

            auto const& FieldMap = MemberFieldToField.find(TypeID)->second;
            auto it2 = FieldMap.find(FieldName);
            if (it2 == FieldMap.end()) {
                throw ESMCError((string)"Field \"" + FieldName + "\" invalid for member type");
            }
            return it2->second;
        }

        const string& UnionType::MapToMemberField(u32 TypeID,
                                                  const string& FieldName) const
        {
            auto it = FieldToMemberField.find(TypeID);
            if (it == FieldToMemberField.end()) {
                throw ESMCError((string)"TypeID " + to_string(TypeID) + " not a valid " +
                                "type id for union type");
            }
            auto const& Map = it->second;
            auto it2 = Map.find(FieldName);
            if (it2 == Map.end()) {
                throw ESMCError((string)"Field Name \"" + FieldName + "\" does not " +
                                "seem to be a member of union type");
            }
            return it2->second;
        }

        void UnionType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, Name);
            for (auto const& Member : MemberTypes) {
                boost::hash_combine(HashCode, Member->Hash());
            }
        }

        i32 UnionType::Compare(const TypeBase& Other) const
        {
            if (Other.As<BooleanType>() != nullptr ||
                Other.As<IntegerType>() != nullptr ||
                Other.As<RangeType>() != nullptr ||
                Other.As<EnumType> () != nullptr ||
                Other.As<SymmetricType>() != nullptr ||
                Other.As<FuncType> () != nullptr ||
                Other.As<ArrayType>() != nullptr ||
                Other.As<ParametricType>() != nullptr ||
                Other.As<FieldAccessType>() != nullptr) {
                return 1;
            }
            if (Other.As<RecordType>() != nullptr &&
                Other.As<UnionType>() == nullptr) {
                return 1;
            }
            auto OtherAsMsg = Other.As<UnionType>();
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

        string UnionType::GetClearValue() const
        {
            throw ESMCError((string)"UnionType::GetClearValue() should never " +
                            "have been called");
        }

    } /* end namespace LTS */
} /* end namespace ESMC */

//
// LTSSemTypes.cpp ends here
