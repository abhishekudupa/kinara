// ExprTypes.hpp --- 
// 
// Filename: ExprTypes.hpp
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

#if !defined ESMC_EXPR_TYPES_HPP_
#define ESMC_EXPR_TYPES_HPP_

#include "../common/FwdDecls.hpp"
#include "../containers/RefCountable.hpp"
#include "../containers/SmartPtr.hpp"
#include <set>
#include <map>
#include <vector>
#include <list>

namespace ESMC {
    namespace Exprs {

        class ExprTypeExtensionBase
        {
        public:
            ExprTypeExtensionBase();
            virtual ~ExprTypeExtensionBase();

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

        class ExprTypeBase : public RefCountable
        {
        private:
            mutable i64 TypeID;
            static UIDGenerator ExprTypeUIDGen;
            mutable bool HashValid;
            mutable list<ExprTypeExtensionBase*> Extensions;
            mutable ExprTypeExtensionBase* LastExtension;

        protected:
            mutable u64 HashCode;

            virtual void ComputeHashValue() const = 0;

        public:
            ExprTypeBase();
            virtual ~ExprTypeBase();

            virtual string ToString() const = 0;
            virtual i32 Compare(const ExprTypeBase& Other) const = 0;
            virtual vector<string> GetElements() const = 0;
            virtual u32 GetByteSize() const = 0;
            
            u64 Hash() const;
            bool Equals(const ExprTypeBase& Other) const;
            bool LT(const ExprTypeBase& Other) const;

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
                vector<list<ExprTypeExtensionBase*>::iterator> ToDelete;

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

            void PurgeAllExtension() const
            {
                for (auto const& Ext : Extensions) {
                    delete Ext;
                }
                Extensions.clear();
            }
        };

        // An abstract base for all scalar types
        class ExprScalarType : public ExprTypeBase
        {
        public:
            ExprScalarType();
            virtual ~ExprScalarType();
        };

        class ExprBoolType : public ExprScalarType
        {
        protected:
            virtual void ComputeHashValue() const override;

        public:
            ExprBoolType();
            virtual ~ExprBoolType();

            virtual string ToString() const override;
            virtual i32 Compare(const ExprTypeBase& Other) const override;
            virtual vector<string> GetElements() const override;
            virtual u32 GetByteSize() const override;
        };


        // A generic int type, can be converted to 
        // any kind of subrange type. 
        class ExprIntType : public ExprScalarType
        {
        protected:
            virtual void ComputeHashValue() const override;
            
        public:
            ExprIntType();
            virtual ~ExprIntType();
            
            virtual string ToString() const override;
            virtual i32 Compare(const ExprTypeBase& Other) const override;
            virtual vector<string> GetElements() const override;
            virtual u32 GetByteSize() const override;
        };

        class ExprRangeType : public ExprIntType
        {
        private:
            i64 RangeLow;
            i64 RangeHigh;
            u64 Size;

        protected:
            virtual void ComputeHashValue() const;

        public:
            ExprRangeType(i64 RangeLow, i64 RangeHigh);
            virtual ~ExprRangeType();
            
            i64 GetLow() const;
            i64 GetHigh() const;
            u64 GetSize() const;

            virtual string ToString() const override;
            virtual i32 Compare(const ExprTypeBase& Other) const override;
            virtual vector<string> GetElements() const override;
            virtual u32 GetByteSize() const override;
        };

        // Mainly for states and such
        class ExprEnumType : public ExprScalarType
        {
        private:
            string Name;
            set<string> Members;

        protected:
            virtual void ComputeHashValue() const;

        public:
            ExprEnumType(const string& Name, const set<string>& Members);
            virtual ~ExprEnumType();
            
            const string& GetName() const;
            const set<string>& GetMembers() const;
            
            bool IsMember(const string& MemberName) const;
            virtual string ToString() const override;
            virtual i32 Compare(const ExprTypeBase& Other) const override;
            virtual vector<string> GetElements() const override;
            virtual u32 GetByteSize() const override;
        };

        class ExprSymmetricType : public ExprScalarType
        {
        private:
            string Name;
            u32 Size;
            vector<string> Members;
            set<string> MemberSet;
            
            // An index into the permutation
            // set
            mutable u32 Index;

        protected:
            virtual void ComputeHashValue() const;

        public:
            ExprSymmetricType(const string& Name, u32 Size);
            virtual ~ExprSymmetricType();

            const string& GetName() const;
            u32 GetSize() const;
            
            const vector<string>& GetMembers() const;
            const string& GetMember(u32 Index) const;
            const bool IsMember(const string& Value) const;

            void SetIndex(u32 Index) const;
            u32 GetIndex() const;

            virtual string ToString() const override;
            virtual i32 Compare(const ExprTypeBase& Other) const override;
            virtual vector<string> GetElements() const override;
            virtual u32 GetByteSize() const override;
        };

        class ExprFuncType : public ExprTypeBase
        {
        private:
            string Name;
            string MangledName;
            vector<ExprTypeRef> ArgTypes;
            ExprTypeRef FuncType;

        protected:
            virtual void ComputeHashValue() const;

        public:
            ExprFuncType(const string& Name, const vector<ExprTypeRef>& ArgTypes,
                        const ExprTypeRef& FuncType);
            virtual ~ExprFuncType();

            const string& GetName() const;
            const vector<ExprTypeRef>& GetArgTypes() const;
            const ExprTypeRef& GetFuncType() const;
            const string& GetMangledName() const;

            virtual string ToString() const override;
            virtual i32 Compare(const ExprTypeBase& Other) const override;
            virtual vector<string> GetElements() const override;
            virtual u32 GetByteSize() const override;
        };

        class ExprArrayType : public ExprTypeBase
        {
        private:
            ExprTypeRef IndexType;
            ExprTypeRef ValueType;

        protected:
            virtual void ComputeHashValue() const;

        public:
            ExprArrayType(const ExprTypeRef& IndexType,
                         const ExprTypeRef& ValueType);
            virtual ~ExprArrayType();

            const ExprTypeRef& GetIndexType() const;
            const ExprTypeRef& GetValueType() const;
            
            virtual string ToString() const override;
            virtual i32 Compare(const ExprTypeBase& Other) const override;
            virtual vector<string> GetElements() const override;
            virtual u32 GetByteSize() const override;
        };

        class ExprRecordType : public ExprTypeBase
        {
        protected:
            string Name;
            map<string, ExprTypeRef> MemberMap;
            vector<pair<string, ExprTypeRef>> MemberVec;

        protected:
            virtual void ComputeHashValue() const;
            // For use in subclasses, viz the ExprMessageType
            ExprRecordType();

        public:
            ExprRecordType(const string& Name,
                           const vector<pair<string, ExprTypeRef>>& RecordMembers);

            virtual ~ExprRecordType();

            const string& GetName() const;
            const map<string, ExprTypeRef>& GetMemberMap() const;
            const vector<pair<string, ExprTypeRef>>& GetMemberVec() const;
            const ExprTypeRef& GetTypeForMember(const string& MemberName) const;

            virtual string ToString() const;
            virtual i32 Compare(const ExprTypeBase& Other) const override;
            virtual vector<string> GetElements() const override;
            virtual u32 GetByteSize() const override;
        };

        // A parametric type RESOLVES to
        // the BASE TYPE, when instantiated with 
        // VALUES of the appropriate types.
        class ExprParametricType : public ExprTypeBase
        {
        private:
            ExprTypeRef BaseType;
            ExprTypeRef ParameterType;

        protected:
            virtual void ComputeHashValue() const override;
            
        public:
            ExprParametricType(const ExprTypeRef& BaseType,
                               const ExprTypeRef& ParameterTypes);
            virtual ~ExprParametricType();

            const ExprTypeRef& GetBaseType() const;
            ExprTypeRef GetTrueBaseType() const;
            const ExprTypeRef& GetParameterType() const;
            const string& GetName() const;

            virtual string ToString() const override;
            virtual i32 Compare(const ExprTypeBase& Other) const override;
            virtual vector<string> GetElements() const override;
            virtual u32 GetByteSize() const override;
        };

        // A dummy type for field access terms variables
        class ExprFieldAccessType : public ExprTypeBase
        {
        protected:
            virtual void ComputeHashValue() const override;

        public:
            ExprFieldAccessType();
            virtual ~ExprFieldAccessType();

            virtual string ToString() const override;
            virtual i32 Compare(const ExprTypeBase& Other) const override;
            virtual vector<string> GetElements() const override;
            virtual u32 GetByteSize() const override;
        };

        // A dedicated class for message types
        class ExprMessageType : public ExprRecordType
        {
        private:
            set<ExprTypeRef> MemberTypes;
            map<ExprTypeRef, u32> MemberTypeToID;
            map<u32, ExprTypeRef> IDToMemberType;
            map<u32, map<string, string>> MemberFieldToField;
            map<u32, map<string, string>> FieldToMemberField;

            const string TypeIDFieldName = "__mtype__";
            ExprTypeRef TypeIDFieldType;

        protected:
            virtual void ComputeHashValue() const override;

        public:
            ExprMessageType(const string& Name,
                            const set<ExprTypeRef>& MemberTypes,
                            const ExprTypeRef& TypeIDFieldType);
            virtual ~ExprMessageType();

            const set<ExprTypeRef>& GetMemberTypes() const;
            const map<ExprTypeRef, u32>& GetMemberTypeToID() const;
            const map<u32, ExprTypeRef>& GetIDToMemberType() const;
            const map<u32, map<string, string>>& GetMemberFieldToField() const;
            const map<u32, map<string, string>>& GetFieldToMemberField() const;
            const string& GetTypeIDFieldName() const;
            const ExprTypeRef& GetTypeIDFieldType() const;
            const u32 GetTypeIDForMemberType(const ExprTypeRef& MemType) const;
            const ExprTypeRef& GetMemberTypeForTypeID(u32 TypeID) const;
            const string& MapFromMemberField(const ExprTypeRef& MemberType, 
                                             const string& FieldName) const;
            const string& MapToMemberField(u32 TypeID,
                                           const string& FieldName) const;

            virtual string ToString() const override;
            virtual i32 Compare(const ExprTypeBase& Other) const override;
        };

        class ExprTypePtrHasher
        {
        public:
            // inline u64 operator () (const ExprTypeBase* Type) const
            // {
            //     return Type->Hash();
            // }
            
            inline u64 operator () (const ExprTypeRef& Type) const
            {
                return Type->Hash();
            }
        };

        class ExprTypePtrEquals
        {
        public:
            // inline bool operator () (const ExprTypeBase* Type1,
            //                          const ExprTypeBase* Type2) const
            // {
            //     return Type1->Equals(*Type2);
            // }

            inline bool operator () (const ExprTypeRef& Type1,
                                     const ExprTypeRef& Type2) const
            {
                return Type1->Equals(*Type2);
            }
        };

        class ExprTypePtrCompare
        {
        public:
            // inline bool operator () (const ExprTypeBase* Type1,
            //                          const ExprTypeBase* Type2) const
            // {
            //     return Type1->LT(*Type2);
            // }

            inline bool operator () (const ExprTypeRef& Type1,
                                     const ExprTypeRef& Type2) const
            {
                return Type1->LT(*Type2);
            }
        };

        // A helper routine to check if types are assignment compatible
        static inline bool CheckAsgnCompat(const ExprTypeRef& LHS,
                                           const ExprTypeRef& RHS)
        {
            if (LHS->Equals(*RHS)) {
                return true;
            }

            if (LHS->Is<ExprIntType>() &&
                RHS->Is<ExprRangeType>()) {
                return true;
            }
            
            if (RHS->Is<ExprIntType>() &&
                LHS->Is<ExprRangeType>()) {
                return true;
            }
            return false;
        }

    } /* end namespace Exprs */
} /* end namespace ESMC */

namespace std {

    template<>
    struct hash<ESMC::Exprs::ExprTypeRef>
    {
        inline size_t operator () (const ESMC::Exprs::ExprTypeRef& Type) const 
        { 
            return Type->Hash(); 
        }
    };

} /* end namespace std */

#endif /* ESMC_EXPR_TYPES_HPP_ */

// 
// ExprTypes.hpp ends here