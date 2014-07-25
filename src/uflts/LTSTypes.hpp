// LTSTypes.hpp --- 
// 
// Filename: LTSTypes.hpp
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

#if !defined ESMC_LTS_TYPES_HPP_
#define ESMC_LTS_TYPES_HPP_

#include "../common/FwdDecls.hpp"
#include "../containers/RefCountable.hpp"
#include "../containers/SmartPtr.hpp"
#include <set>
#include <map>
#include <vector>

namespace ESMC {
    namespace LTS {

        class LTSTypeBase : public RefCountable
        {
        private:
            mutable i64 TypeID;
            static UIDGenerator LTSTypeUIDGen;
            mutable bool HashValid;

        protected:
            mutable u64 HashCode;

            virtual void ComputeHashValue() const = 0;

        public:
            LTSTypeBase();
            virtual ~LTSTypeBase();

            virtual string ToString() const = 0;
            virtual i32 Compare(const LTSTypeBase& Other) const = 0;
            
            
            u64 Hash() const;
            bool Equals(const LTSTypeBase& Other) const;
            bool LT(const LTSTypeBase& Other) const;

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
        };

        class LTSBoolType : public LTSTypeBase
        {
        protected:
            virtual void ComputeHashValue() const override;

        public:
            LTSBoolType();
            virtual ~LTSBoolType();

            virtual string ToString() const override;
            virtual i32 Compare(const LTSTypeBase& Other) const override;
        };


        // A generic int type, can be converted to 
        // any kind of subrange type. 
        class LTSIntType : public LTSTypeBase
        {
        protected:
            virtual void ComputeHashValue() const override;
            
        public:
            LTSIntType();
            virtual ~LTSIntType();
            
            virtual string ToString() const override;
            virtual i32 Compare(const LTSTypeBase& Other) const override;
        };

        class LTSRangeType : public LTSIntType
        {
        private:
            i64 RangeLow;
            i64 RangeHigh;
            u64 Size;

        protected:
            virtual void ComputeHashValue() const;

        public:
            LTSRangeType(i64 RangeLow, i64 RangeHigh);
            virtual ~LTSRangeType();
            
            i64 GetLow() const;
            i64 GetHigh() const;
            u64 GetSize() const;

            virtual string ToString() const override;
            virtual i32 Compare(const LTSTypeBase& Other) const override;
        };

        // Mainly for states and such
        class LTSEnumType : public LTSTypeBase
        {
        private:
            string Name;
            set<string> Members;

        protected:
            virtual void ComputeHashValue() const;

        public:
            LTSEnumType(const string& Name, const set<string>& Members);
            virtual ~LTSEnumType();
            
            const string& GetName() const;
            const set<string>& GetMembers() const;
            
            bool IsMember(const string& MemberName) const;
            virtual string ToString() const override;
            virtual i32 Compare(const LTSTypeBase& Other) const override;
        };

        class LTSSymmetricType : public LTSTypeBase 
        {
        private:
            string Name;
            u32 Size;
            vector<string> Members;
            set<string> MemberSet;

        protected:
            virtual void ComputeHashValue() const;

        public:
            LTSSymmetricType(const string& Name, u32 Size);
            virtual ~LTSSymmetricType();

            const string& GetName() const;
            u32 GetSize() const;
            
            const vector<string>& GetMembers() const;
            const string& GetMember(u32 Index) const;
            const bool IsMember(const string& Value) const;

            virtual string ToString() const override;
            virtual i32 Compare(const LTSTypeBase& Other) const override;
        };

        class LTSFuncType : public LTSTypeBase
        {
        private:
            string Name;
            string MangledName;
            vector<LTSTypeRef> ArgTypes;
            LTSTypeRef FuncType;

        protected:
            virtual void ComputeHashValue() const;

        public:
            LTSFuncType(const string& Name, const vector<LTSTypeRef>& ArgTypes,
                        const LTSTypeRef& FuncType);
            virtual ~LTSFuncType();

            const string& GetName() const;
            const vector<LTSTypeRef>& GetArgTypes() const;
            const LTSTypeRef& GetFuncType() const;
            const string& GetMangledName() const;

            virtual string ToString() const override;
            virtual i32 Compare(const LTSTypeBase& Other) const override;
        };

        class LTSArrayType : public LTSTypeBase
        {
        private:
            LTSTypeRef IndexType;
            LTSTypeRef ValueType;

        protected:
            virtual void ComputeHashValue() const;

        public:
            LTSArrayType(const LTSTypeRef& IndexType,
                         const LTSTypeRef& ValueType);
            virtual ~LTSArrayType();

            const LTSTypeRef& GetIndexType() const;
            const LTSTypeRef& GetValueType() const;
            
            virtual string ToString() const override;
            virtual i32 Compare(const LTSTypeBase& Other) const override;
        };

        class LTSRecordType : public LTSTypeBase
        {
        private:
            string Name;
            map<string, LTSTypeRef> Members;

        protected:
            virtual void ComputeHashValue() const;

        public:
            LTSRecordType(const string& Name,
                          const map<string, LTSTypeRef>& RecordMembers);
            virtual ~LTSRecordType();

            const string& GetName() const;
            const map<string, LTSTypeRef>& GetMembers() const;
            const LTSTypeRef& GetTypeForMember(const string& MemberName) const;

            virtual string ToString() const;
            virtual i32 Compare(const LTSTypeBase& Other) const override;
        };

        // A VALUE of a parametric type RESOLVES to a VALUE
        // of the BASE TYPE, when instantiated with 
        // VALUES of the appropriate types.
        class LTSParametricType : public LTSTypeBase
        {
        private:
            LTSTypeRef BaseType;
            vector<LTSTypeRef> ParameterTypes;

        protected:
            virtual void ComputeHashValue() const override;
            
        public:
            LTSParametricType(const LTSTypeRef& BaseType,
                              const vector<LTSTypeRef>& ParameterTypes);
            virtual ~LTSParametricType();

            const LTSTypeRef& GetBaseType() const;
            const vector<LTSTypeRef>& GetParameterTypes() const;

            virtual string ToString() const override;
            virtual i32 Compare(const LTSTypeBase& Other) const override;
        };


        class LTSTypePtrHasher
        {
            inline u64 operator () (const LTSTypeBase* Type) const
            {
                return Type->Hash();
            }
            
            inline u64 operator () (const LTSTypeRef& Type) const
            {
                return Type->Hash();
            }
        };

        class LTSTypePtrEquals
        {
            inline bool operator () (const LTSTypeBase* Type1,
                                     const LTSTypeBase* Type2) const
            {
                return Type1->Equals(*Type2);
            }

            inline bool operator () (const LTSTypeRef& Type1,
                                     const LTSTypeRef& Type2) const
            {
                return Type1->Equals(*Type2);
            }
        };

        class LTSTypePtrCompare
        {
            inline bool operator () (const LTSTypeBase* Type1,
                                     const LTSTypeBase* Type2) const
            {
                return Type1->LT(*Type2);
            }

            inline bool operator () (const LTSTypeRef& Type1,
                                     const LTSTypeRef& Type2) const
            {
                return Type1->LT(*Type2);
            }
        };

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LTS_TYPES_HPP_ */

// 
// LTSTypes.hpp ends here
