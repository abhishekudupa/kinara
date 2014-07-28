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
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/functional/hash.hpp>

namespace ESMC {
    namespace Exprs {

        UIDGenerator ExprTypeBase::ExprTypeUIDGen(1);

        ExprTypeBase::ExprTypeBase()
            : TypeID(-1)
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

        ExprBoolType::ExprBoolType()
            : ExprTypeBase()
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

        void ExprBoolType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, "BoolType");
        }

        ExprIntType::ExprIntType()
            : ExprTypeBase()
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

        // Inclusive range
        ExprRangeType::ExprRangeType(i64 RangeLow, i64 RangeHigh)
            : ExprIntType(), 
              RangeLow(RangeLow), RangeHigh(RangeHigh), 
              Size(RangeHigh - RangeLow + 1)
        {
            // Nothing here
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

        ExprEnumType::ExprEnumType(const string& Name, 
                                 const set<string>& Members)
            : ExprTypeBase(), Name(Name), Members(Members)
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

        ExprSymmetricType::ExprSymmetricType(const string& Name, u32 Size)
            : ExprTypeBase(), Name(Name), Size(Size), Members(Size)
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

        void ExprArrayType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, IndexType->Hash());
            boost::hash_combine(HashCode, ValueType->Hash());
        }

        ExprRecordType::ExprRecordType(const string& Name,
                                       const map<string, ExprTypeRef>& Members)
            : ExprTypeBase(), Name(Name), Members(Members)
        {
            for (auto const& NTPair : Members) {
                if (NTPair.second->As<ExprFuncType>() != nullptr) {
                    throw ESMCError("Record members cannot be functions types");
                }
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

        const map<string, ExprTypeRef>& ExprRecordType::GetMembers() const
        {
            return Members;
        }

        const ExprTypeRef& ExprRecordType::GetTypeForMember(const string& MemberName) const
        {
            auto it = Members.find(MemberName);
            if (it == Members.end()) {
                return ExprTypeRef::NullPtr;
            }
            return it->second;
        }

        string ExprRecordType::ToString() const
        {
            string Retval;
            Retval += "(Rec " + Name + " :";
            for (auto const& NTPair : Members) {
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

        void ExprRecordType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, Name);
            boost::hash_combine(HashCode, Members.size());
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

        string ExprFieldAccessType::ToString() const
        {
            return "(FieldAccessType)";
        }

        ExprUndefType::ExprUndefType()
            : ExprTypeBase()
        {
            // NOthing here
        }

        ExprUndefType::~ExprUndefType()
        {
            // Nothing here
        }

        void ExprUndefType::ComputeHashValue() const
        {
            HashCode = 0;
            boost::hash_combine(HashCode, "UndefType");
        }

        string ExprUndefType::ToString() const
        {
            return "(UndefType)";
        }

        i32 ExprUndefType::Compare(const ExprTypeBase& Other) const
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

            if (Other.Is<ExprUndefType>()) {
                return 0;
            } else {
                return -1;
            }
        }

    } /* end namespace Exprs */
} /* end namespace ESMC */

// 
// ExprTypes.cpp ends here
