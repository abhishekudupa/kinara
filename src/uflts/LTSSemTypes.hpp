// LTSSemTypes.hpp ---
//
// Filename: LTSSemTypes.hpp
// Author: Abhishek Udupa
// Created: Thu Jul 24 10:44:06 2014 (-0400)
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

#if !defined ESMC_LTS_SEM_TYPES_HPP_
#define ESMC_LTS_SEM_TYPES_HPP_

#include "../common/ESMCFwdDecls.hpp"
#include "../containers/RefCountable.hpp"
#include "../containers/SmartPtr.hpp"

#include <set>
#include <map>
#include <vector>
#include <list>

namespace ESMC {
    namespace LTS {

        class TypeExtensionBase
        {
        public:
            TypeExtensionBase();
            virtual ~TypeExtensionBase();

            template <typename T>
            inline T* As()
            {
                return dynamic_cast<T*>(this);
            }

            template <typename T>
            inline const T* As() const
            {
                return dynamic_cast<const T*>(this);
            }

            template <typename T>
            inline T* SAs()
            {
                return static_cast<T*>(this);
            }

            template <typename T>
            inline const T* SAs() const
            {
                return static_cast<const T*>(this);
            }

            template <typename T>
            inline bool Is() const
            {
                return (dynamic_cast<const T*>(this) != nullptr);
            }
        };


        class TypeBase : public RefCountable
        {
        private:
            mutable i64 TypeID;
            static UIDGenerator TypeUIDGen;
            mutable bool HashValid;
            mutable list<TypeExtensionBase*> Extensions;
            mutable TypeExtensionBase* LastExtension;

        protected:
            mutable u64 HashCode;

            virtual void ComputeHashValue() const = 0;

        public:
            TypeBase();
            virtual ~TypeBase();

            virtual string ToString() const = 0;
            virtual i32 Compare(const TypeBase& Other) const = 0;
            virtual vector<string> GetElements() const = 0;
            virtual vector<string> GetElementsNoUndef() const = 0;
            virtual u32 GetByteSize() const = 0;
            virtual u32 GetCardinality() const = 0;
            virtual u32 GetCardinalityNoUndef() const = 0;
            virtual string GetClearValue() const = 0;

            u64 Hash() const;
            bool Equals(const TypeBase& Other) const;
            bool LT(const TypeBase& Other) const;

            i64 GetTypeID() const;
            i64 SetTypeID() const;
            i64 GetOrSetTypeID() const;


            template <typename T>
            inline T* As()
            {
                return dynamic_cast<T*>(this);
            }

            template <typename T>
            inline const T* As() const
            {
                return dynamic_cast<const T*>(this);
            }

            template <typename T>
            inline T* SAs()
            {
                return static_cast<T*>(this);
            }

            template <typename T>
            inline const T* SAs() const
            {
                return static_cast<const T*>(this);
            }

            template <typename T>
            inline bool Is() const
            {
                return (dynamic_cast<const T*>(this) != nullptr);
            }

            inline void AddExtension(TypeExtensionBase* Ext) const
            {
                Extensions.push_front(Ext);
            }

            template <typename T>
            inline T* GetExtension() const
            {
                if (LastExtension->Is<T>()) {
                    return LastExtension->SAs<T>();
                }

                for (auto it = Extensions.begin(); it != Extensions.end(); ++it) {
                    auto Ext = *it;
                    if (Ext->Is<T>()) {
                        // Cache and move to head of list
                        LastExtension = Ext;
                        Extensions.erase(it);
                        Extensions.push_front(LastExtension);
                        return LastExtension->SAs<T>();
                    }
                }
                return nullptr;
            }

            template <typename T>
            vector<T*> GetAllExtensions() const
            {
                vector<T*> Retval;
                for (auto const& Ext : Extensions) {
                    if (Ext->Is<T>()) {
                        Retval.push_back(Ext->SAs<T>());
                    }
                }
                return Retval;
            }

            template <typename T>
            void PurgeExtensionsOfType() const
            {
                vector<list<TypeExtensionBase*>::iterator> ToDelete;

                for(auto it = Extensions.begin(); it != Extensions.end(); ++it) {
                    if ((*it)->Is<T>()) {
                        ToDelete.push_back(it);
                    }
                }

                for (auto const& it : ToDelete) {
                    Extensions.erase(it);
                    delete (*it);
                }
            }

            void PurgeAllExtensions() const
            {
                for (auto const& Ext : Extensions) {
                    delete Ext;
                }
                Extensions.clear();
            }
        };

        // An abstract base for all scalar types
        class ScalarType : public TypeBase
        {
        public:
            ScalarType();
            virtual ~ScalarType();

            virtual i64 ConstToVal(const string& ConstVal) const = 0;
            virtual string ValToConst(i64 Val) const = 0;
        };

        class BooleanType : public ScalarType
        {
        protected:
            virtual void ComputeHashValue() const override;

        public:
            BooleanType();
            virtual ~BooleanType();

            virtual string ToString() const override;
            virtual i32 Compare(const TypeBase& Other) const override;
            virtual vector<string> GetElements() const override;
            virtual vector<string> GetElementsNoUndef() const override;
            virtual u32 GetByteSize() const override;
            virtual u32 GetCardinality() const override;
            virtual u32 GetCardinalityNoUndef() const override;

            virtual i64 ConstToVal(const string& ConstVal) const override;
            virtual string ValToConst(i64 Val) const override;
            virtual string GetClearValue() const override;
        };


        // A generic int type, can be converted to
        // any kind of subrange type.
        class IntegerType : public ScalarType
        {
        protected:
            virtual void ComputeHashValue() const override;

        public:
            IntegerType();
            virtual ~IntegerType();

            virtual string ToString() const override;
            virtual i32 Compare(const TypeBase& Other) const override;
            virtual vector<string> GetElementsNoUndef() const override;
            virtual vector<string> GetElements() const override;
            virtual u32 GetByteSize() const override;
            virtual u32 GetCardinality() const override;
            virtual u32 GetCardinalityNoUndef() const override;

            virtual i64 ConstToVal(const string& ConstVal) const override;
            virtual string ValToConst(i64 Val) const override;
            virtual string GetClearValue() const override;
        };

        class RangeType : public IntegerType
        {
        private:
            i64 RangeLow;
            i64 RangeHigh;
            u64 Size;

        protected:
            virtual void ComputeHashValue() const;

        public:
            RangeType(i64 RangeLow, i64 RangeHigh);
            virtual ~RangeType();

            i64 GetLow() const;
            i64 GetHigh() const;
            u64 GetSize() const;

            virtual string ToString() const override;
            virtual i32 Compare(const TypeBase& Other) const override;
            virtual vector<string> GetElements() const override;
            virtual vector<string> GetElementsNoUndef() const override;
            virtual u32 GetByteSize() const override;
            virtual u32 GetCardinality() const override;
            virtual u32 GetCardinalityNoUndef() const override;

            virtual i64 ConstToVal(const string& ConstVal) const override;
            virtual string ValToConst(i64 Val) const override;
            virtual string GetClearValue() const override;
        };

        // Mainly for states and such
        class EnumType : public ScalarType
        {
        private:
            string Name;
            set<string> Members;
            vector<string> MemberVec;

        protected:
            virtual void ComputeHashValue() const;

        public:
            EnumType(const string& Name, const set<string>& Members);
            virtual ~EnumType();

            const string& GetName() const;
            const set<string>& GetMembers() const;

            bool IsMember(const string& MemberName) const;
            u32 GetMemberIdx(const string& MemberName) const;

            virtual string ToString() const override;
            virtual i32 Compare(const TypeBase& Other) const override;
            virtual vector<string> GetElements() const override;
            virtual vector<string> GetElementsNoUndef() const override;
            virtual u32 GetByteSize() const override;
            virtual u32 GetCardinality() const override;
            virtual u32 GetCardinalityNoUndef() const override;

            virtual i64 ConstToVal(const string& ConstVal) const override;
            virtual string ValToConst(i64 Val) const override;
            virtual string GetClearValue() const override;
        };

        class SymmetricType : public ScalarType
        {
        private:
            string Name;
            u32 Size;
            vector<string> Members;
            vector<string> MembersNoUndef;
            set<string> MemberSet;

            // An index into the permutation
            // set
            mutable u32 Index;

        protected:
            virtual void ComputeHashValue() const;

        public:
            SymmetricType(const string& Name, u32 Size);
            virtual ~SymmetricType();

            const string& GetName() const;
            u32 GetSize() const;

            const vector<string>& GetMembers() const;
            const string& GetMember(u32 Index) const;
            const bool IsMember(const string& Value) const;
            u32 GetMemberIdx(const string& Value) const;

            void SetIndex(u32 Index) const;
            u32 GetIndex() const;

            virtual string ToString() const override;
            virtual i32 Compare(const TypeBase& Other) const override;
            virtual vector<string> GetElements() const override;
            virtual vector<string> GetElementsNoUndef() const override;
            virtual u32 GetByteSize() const override;
            virtual u32 GetCardinality() const override;
            virtual u32 GetCardinalityNoUndef() const override;

            virtual i64 ConstToVal(const string& ConstVal) const override;
            virtual string ValToConst(i64 Val) const override;
            virtual string GetClearValue() const override;
        };

        class FuncType : public TypeBase
        {
        private:
            string Name;
            string MangledName;
            vector<TypeRef> ArgTypes;
            TypeRef EvalType;

        protected:
            virtual void ComputeHashValue() const;

        public:
            FuncType(const string& Name, const vector<TypeRef>& ArgTypes,
                     const TypeRef& RangeType);
            virtual ~FuncType();

            const string& GetName() const;
            const vector<TypeRef>& GetArgTypes() const;
            const TypeRef& GetEvalType() const;
            const string& GetMangledName() const;

            virtual string ToString() const override;
            virtual i32 Compare(const TypeBase& Other) const override;
            virtual vector<string> GetElements() const override;
            virtual vector<string> GetElementsNoUndef() const override;
            virtual u32 GetByteSize() const override;
            virtual u32 GetCardinality() const override;
            virtual u32 GetCardinalityNoUndef() const override;
            virtual string GetClearValue() const override;
        };

        class ArrayType : public TypeBase
        {
        private:
            TypeRef IndexType;
            TypeRef ValueType;

        protected:
            virtual void ComputeHashValue() const;

        public:
            ArrayType(const TypeRef& IndexType,
                      const TypeRef& ValueType);
            virtual ~ArrayType();

            const TypeRef& GetIndexType() const;
            const TypeRef& GetValueType() const;
            u32 GetOffset(u32 ElemIdx) const;
            TypeRef GetBaseValueType() const;
            u32 GetLevelsOfIndex() const;

            virtual string ToString() const override;
            virtual i32 Compare(const TypeBase& Other) const override;
            virtual vector<string> GetElements() const override;
            virtual vector<string> GetElementsNoUndef() const override;
            virtual u32 GetByteSize() const override;
            virtual u32 GetCardinality() const override;
            virtual u32 GetCardinalityNoUndef() const override;
            virtual string GetClearValue() const override;
        };

        class RecordType : public TypeBase
        {
        protected:
            string Name;
            map<string, TypeRef> MemberMap;
            vector<pair<string, TypeRef>> MemberVec;
            mutable map<string, u32> FieldOffsets;
            mutable bool ContainsUnboundedType;
            mutable bool FieldOffsetsComputed;

        protected:
            virtual void ComputeHashValue() const;
            // For use in subclasses, viz the ExprMessageType
            RecordType();

        private:
            void ComputeFieldOffsets() const;

        public:
            RecordType(const string& Name,
                       const vector<pair<string, TypeRef>>& RecordMembers);

            virtual ~RecordType();

            const string& GetName() const;
            const map<string, TypeRef>& GetMemberMap() const;
            const vector<pair<string, TypeRef>>& GetMemberVec() const;
            const TypeRef& GetTypeForMember(const string& MemberName) const;
            u32 GetFieldOffset(const string& FieldName) const;
            u32 GetFieldIdx(const string& FieldName) const;

            virtual string ToString() const;
            virtual i32 Compare(const TypeBase& Other) const override;
            virtual vector<string> GetElements() const override;
            virtual vector<string> GetElementsNoUndef() const override;
            virtual u32 GetByteSize() const override;
            virtual u32 GetCardinality() const override;
            virtual u32 GetCardinalityNoUndef() const override;
            virtual string GetClearValue() const override;
        };

        // A parametric record type RESOLVES to
        // the BASE TYPE, when instantiated with
        // VALUES of the appropriate types.
        class ParametricType : public TypeBase
        {
        private:
            TypeRef BaseType;
            vector<TypeRef> ParameterTypes;

        protected:
            virtual void ComputeHashValue() const override;

        public:
            ParametricType(const TypeRef& BaseType,
                               const vector<TypeRef>& ParameterTypes);
            virtual ~ParametricType();

            const TypeRef& GetBaseType() const;
            const vector<TypeRef>& GetParameterTypes() const;
            const string& GetName() const;

            virtual string ToString() const override;
            virtual i32 Compare(const TypeBase& Other) const override;
            virtual vector<string> GetElements() const override;
            virtual vector<string> GetElementsNoUndef() const override;
            virtual u32 GetByteSize() const override;
            virtual u32 GetCardinality() const override;
            virtual u32 GetCardinalityNoUndef() const override;
            virtual string GetClearValue() const override;
        };

        // A dummy type for field access terms variables
        class FieldAccessType : public TypeBase
        {
        protected:
            virtual void ComputeHashValue() const override;

        public:
            FieldAccessType();
            virtual ~FieldAccessType();

            virtual string ToString() const override;
            virtual i32 Compare(const TypeBase& Other) const override;
            virtual vector<string> GetElements() const override;
            virtual vector<string> GetElementsNoUndef() const override;
            virtual u32 GetByteSize() const override;
            virtual u32 GetCardinality() const override;
            virtual u32 GetCardinalityNoUndef() const override;
            virtual string GetClearValue() const override;
        };

        // A dedicated class for message types
        class UnionType : public RecordType
        {
        private:
            set<TypeRef> MemberTypes;
            map<TypeRef, u32> MemberTypeToID;
            map<u32, TypeRef> IDToMemberType;
            map<u32, map<string, string>> MemberFieldToField;
            map<u32, map<string, string>> FieldToMemberField;

            const string TypeIDFieldName = "__mtype__";
            TypeRef TypeIDFieldType;

        protected:
            virtual void ComputeHashValue() const override;

        public:
            UnionType(const string& Name,
                      const set<TypeRef>& MemberTypes,
                      const TypeRef& TypeIDFieldType);
            virtual ~UnionType();

            const set<TypeRef>& GetMemberTypes() const;
            const map<TypeRef, u32>& GetMemberTypeToID() const;
            const map<u32, TypeRef>& GetIDToMemberType() const;
            const map<u32, map<string, string>>& GetMemberFieldToField() const;
            const map<u32, map<string, string>>& GetFieldToMemberField() const;
            const string& GetTypeIDFieldName() const;
            const TypeRef& GetTypeIDFieldType() const;
            const u32 GetTypeIDForMemberType(const TypeRef& MemType) const;
            const TypeRef& GetMemberTypeForTypeID(u32 TypeID) const;
            const string& MapFromMemberField(const TypeRef& MemberType,
                                             const string& FieldName) const;
            const string& MapToMemberField(u32 TypeID,
                                           const string& FieldName) const;

            virtual i32 Compare(const TypeBase& Other) const override;
            string GetClearValue() const override;
        };

        class TypePtrHasher
        {
        public:
            // inline u64 operator () (const TypeBase* Type) const
            // {
            //     return Type->Hash();
            // }

            inline u64 operator () (const TypeRef& Type) const
            {
                return Type->Hash();
            }
        };

        class TypePtrEquals
        {
        public:
            // inline bool operator () (const TypeBase* Type1,
            //                          const TypeBase* Type2) const
            // {
            //     return Type1->Equals(*Type2);
            // }

            inline bool operator () (const TypeRef& Type1,
                                     const TypeRef& Type2) const
            {
                return Type1->Equals(*Type2);
            }
        };

        class TypePtrCompare
        {
        public:
            // inline bool operator () (const TypeBase* Type1,
            //                          const TypeBase* Type2) const
            // {
            //     return Type1->LT(*Type2);
            // }

            inline bool operator () (const TypeRef& Type1,
                                     const TypeRef& Type2) const
            {
                return Type1->LT(*Type2);
            }
        };

        // A helper routine to check if types are assignment compatible
        static inline bool CheckAsgnCompat(const TypeRef& LHS,
                                           const TypeRef& RHS)
        {
            if (LHS->Equals(*RHS)) {
                return true;
            }

            if (LHS->Is<IntegerType>() &&
                RHS->Is<RangeType>()) {
                return true;
            }

            if (RHS->Is<IntegerType>() &&
                LHS->Is<RangeType>()) {
                return true;
            }
            return false;
        }

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LTS_SEM_TYPES_HPP_ */

//
// LTSSemTypes.hpp ends here
