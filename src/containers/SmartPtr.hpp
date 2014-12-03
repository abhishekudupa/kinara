// SmartPtr.hpp ---
//
// Filename: SmartPtr.hpp
// Author: Abhishek Udupa
// Created: Sun Jun 29 14:10:50 2014 (-0400)
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

#if !defined ESMC_SMART_PTR_HPP_
#define ESMC_SMART_PTR_HPP_

#include "../common/FwdDecls.hpp"

namespace ESMC {

    // Forward declarations
    template <typename T> class SmartPtr;
    // const smart ptr
    template <typename T> class CSmartPtr;

    // Typedef for a normal ptr
    template <typename T>
    using RawPtr = T*;

    template <typename T>
    using RawCPtr = const T*;

    // Typedef for no reference at all
    template <typename T>
    using NoPtr = T;

    template <typename T>
    class SmartPtr
    {
        friend class CSmartPtr<T>;
    private:
        T* Ptr_;

        template <typename U>
        inline i64 Compare_(const U& Other) const;
        inline i64 Compare_(const T* OtherPtr) const;

    public:
        static const SmartPtr NullPtr;

        inline SmartPtr();
        inline SmartPtr(const SmartPtr& Other);
        // move constructor
        inline SmartPtr(SmartPtr&& Other);
        inline SmartPtr(T* OtherPtr);
        inline ~SmartPtr();

        inline T* GetPtr_() const;

        // casting, use at your own risk!
        inline operator T* () const;

        inline SmartPtr& operator = (SmartPtr Other);
        inline SmartPtr& operator = (T* OtherPtr);

        inline T* operator -> () const;
        inline T& operator * () const;

        template <typename U>
        inline bool operator == (const U& Other) const;
        template <typename U>
        inline bool operator != (const U& Other) const;
        template<typename U>
        inline bool operator < (const U& Other) const;
        template <typename U>
        inline bool operator <= (const U& Other) const;
        template <typename U>
        inline bool operator > (const U& Other) const;
        template <typename U>
        inline bool operator >= (const U& Other) const;

        inline bool operator == (const T* OtherPtr) const;
        inline bool operator != (const T* OtherPtr) const;
        inline bool operator < (const T* OtherPtr) const;
        inline bool operator <= (const T* OtherPtr) const;
        inline bool operator > (const T* OtherPtr) const;
        inline bool operator >= (const T* OtherPtr) const;

        inline bool operator ! () const;
        inline bool IsNull_() const;
    };

    template<typename T>
    class CSmartPtr
    {
    private:
        const T* Ptr_;

        template<typename U>
        inline i64 Compare_(const U& Other) const;
        inline i64 Compare_(const T* OtherPtr) const;

    public:
        static const CSmartPtr NullPtr;

        inline CSmartPtr();
        inline CSmartPtr(const CSmartPtr& Other);
        inline CSmartPtr(CSmartPtr&& Other);
        inline CSmartPtr(const SmartPtr<T>& Other);
        inline CSmartPtr(SmartPtr<T>&& Other);
        inline CSmartPtr(const T* OtherPtr);
        inline ~CSmartPtr();

        CSmartPtr& operator = (CSmartPtr Other);
        CSmartPtr& operator = (SmartPtr<T> Other);
        CSmartPtr& operator = (const T* OtherPtr);

        inline const T* GetPtr_() const;

        // casting, use at your own risk!
        inline operator const T* () const;

        inline const T* operator -> () const;
        inline const T& operator * () const;

        template <typename U>
        inline bool operator == (const U& Other) const;
        template <typename U>
        inline bool operator != (const U& Other) const;
        template <typename U>
        inline bool operator < (const U& Other) const;
        template <typename U>
        inline bool operator <= (const U& Other) const;
        template <typename U>
        inline bool operator > (const U& Other) const;
        template <typename U>
        inline bool operator >= (const U& Other) const;

        inline bool operator == (const T* OtherPtr) const;
        inline bool operator != (const T* OtherPtr) const;
        inline bool operator < (const T* OtherPtr) const;
        inline bool operator <= (const T* OtherPtr) const;
        inline bool operator > (const T* OtherPtr) const;
        inline bool operator >= (const T* OtherPtr) const;

        inline bool operator ! () const;
        inline bool IsNull_() const;
    };

    // Implementation of SmartPtr
    template <typename T>
    const SmartPtr<T> SmartPtr<T>::NullPtr;

    template <typename T>
    template <typename U>
    inline i64 SmartPtr<T>::Compare_(const U &Other) const
    {
        return (i64)((char*)Ptr_ - (char*)Other.Ptr_);
    }

    template <typename T>
    inline i64 SmartPtr<T>::Compare_(const T *OtherPtr) const
    {
        return (i64)((char*)Ptr_ - (char*)OtherPtr);
    }

    template <typename T>
    inline SmartPtr<T>::SmartPtr()
        : Ptr_(nullptr)
    {
        // Nothing here
    }

    template <typename T>
    inline SmartPtr<T>::SmartPtr(const SmartPtr<T>& Other)
        : Ptr_(nullptr)
    {
        Ptr_ = Other.Ptr_;
        if (Ptr_ != nullptr) {
            Ptr_->IncRef_();
        }
    }

    template <typename T>
    inline SmartPtr<T>::SmartPtr(SmartPtr<T>&& Other)
        : SmartPtr<T>()
    {
        swap(Ptr_, Other.Ptr_);
    }

    template <typename T>
    inline SmartPtr<T>::SmartPtr(T* OtherPtr)
        : Ptr_(OtherPtr)
    {
        if (Ptr_ != nullptr) {
            Ptr_->IncRef_();
        }
    }

    template <typename T>
    inline SmartPtr<T>::~SmartPtr()
    {
        if (Ptr_ != nullptr) {
            Ptr_->DecRef_();
        }
        Ptr_ = nullptr;
    }

    template <typename T>
    inline T* SmartPtr<T>::GetPtr_() const
    {
        return Ptr_;
    }

    template <typename T>
    inline SmartPtr<T>::operator T* () const
    {
        return (GetPtr_());
    }

    template <typename T>
    inline SmartPtr<T>& SmartPtr<T>::operator = (SmartPtr<T> Other)
    {
        swap(Ptr_, Other.Ptr_);
        return (*this);
    }

    template <typename T>
    inline SmartPtr<T>& SmartPtr<T>::operator = (T* OtherPtr)
    {
        SmartPtr<T> Dummy(OtherPtr);
        swap(Ptr_, Dummy.Ptr_);
        return (*this);
    }

    template <typename T>
    inline T* SmartPtr<T>::operator -> () const
    {
        return (GetPtr_());
    }

    template <typename T>
    inline T& SmartPtr<T>::operator * () const
    {
        return (*Ptr_);
    }

    template <typename T>
    template <typename U>
    inline bool SmartPtr<T>::operator == (const U& Other) const
    {
        return (Compare_(Other) == 0);
    }

    template <typename T>
    template <typename U>
    inline bool SmartPtr<T>::operator != (const U& Other) const
    {
        return (Compare_(Other) != 0);
    }

    template <typename T>
    template <typename U>
    inline bool SmartPtr<T>::operator < (const U& Other) const
    {
        return (Compare_(Other) < 0);
    }

    template <typename T>
    template <typename U>
    inline bool SmartPtr<T>::operator <= (const U& Other) const
    {
        return (Compare_(Other) <= 0);
    }

    template <typename T>
    template <typename U>
    inline bool SmartPtr<T>::operator > (const U& Other) const
    {
        return (Compare_(Other) > 0);
    }

    template <typename T>
    template <typename U>
    inline bool SmartPtr<T>::operator >= (const U& Other) const
    {
        return (Compare_(Other) >= 0);
    }

    template <typename T>
    inline bool SmartPtr<T>::operator == (const T* OtherPtr) const
    {
        return (Compare_(OtherPtr) == 0);
    }

    template <typename T>
    inline bool SmartPtr<T>::operator != (const T* OtherPtr) const
    {
        return (Compare_(OtherPtr) != 0);
    }

    template <typename T>
    inline bool SmartPtr<T>::operator < (const T* OtherPtr) const
    {
        return (Compare_(OtherPtr) < 0);
    }

    template <typename T>
    inline bool SmartPtr<T>::operator <= (const T* OtherPtr) const
    {
        return (Compare_(OtherPtr) <= 0);
    }

    template <typename T>
    inline bool SmartPtr<T>::operator > (const T* OtherPtr) const
    {
        return (Compare_(OtherPtr) > 0);
    }

    template <typename T>
    inline bool SmartPtr<T>::operator >= (const T* OtherPtr) const
    {
        return (Compare_(OtherPtr) >= 0);
    }

    template <typename T>
    inline bool SmartPtr<T>::IsNull_() const
    {
        return (Ptr_ == nullptr);
    }

    template <typename T>
    inline bool SmartPtr<T>::operator ! () const
    {
        return (IsNull_());
    }

    // implementation of CSmartPtr

    template<typename T>
    const CSmartPtr<T> CSmartPtr<T>::NullPtr;

    template <typename T>
    template <typename U>
    inline i64 CSmartPtr<T>::Compare_(const U &Other) const
    {
        return (i64)((char*)Ptr_ - (char*)Other.Ptr_);
    }

    template <typename T>
    inline i64 CSmartPtr<T>::Compare_(const T *OtherPtr) const
    {
        return (i64)((char*)Ptr_ - (char*)OtherPtr);
    }

    template <typename T>
    inline CSmartPtr<T>::CSmartPtr()
        : Ptr_(nullptr)
    {
        // Nothing here
    }

    template <typename T>
    inline CSmartPtr<T>::CSmartPtr(const CSmartPtr<T>& Other)
        : Ptr_(nullptr)
    {
        Ptr_ = Other.Ptr_;
        if (Ptr_ != nullptr) {
            Ptr_->IncRef_();
        }
    }

    template <typename T>
    inline CSmartPtr<T>::CSmartPtr(CSmartPtr<T>&& Other)
        : CSmartPtr<T>()
    {
        swap(Ptr_, Other.Ptr_);
    }

    template <typename T>
    inline CSmartPtr<T>::CSmartPtr(const SmartPtr<T>& Other)
        : Ptr_(nullptr)
    {
        Ptr_ = Other.Ptr_;
        if (Ptr_ != nullptr) {
            Ptr_->IncRef_();
        }
    }

    template <typename T>
    inline CSmartPtr<T>::CSmartPtr(SmartPtr<T>&& Other)
        : CSmartPtr()
    {
        swap(const_cast<T*>(Ptr_), Other.Ptr_);
    }

    template <typename T>
    inline CSmartPtr<T>::CSmartPtr(const T* OtherPtr)
        : Ptr_(nullptr)
    {
        Ptr_ = OtherPtr;
        if (Ptr_ != nullptr) {
            Ptr_->IncRef_();
        }
    }

    template <typename T>
    inline CSmartPtr<T>::~CSmartPtr()
    {
        if (Ptr_ != nullptr) {
            Ptr_->DecRef_();
        }
        Ptr_ = nullptr;
    }

    template <typename T>
    inline CSmartPtr<T>& CSmartPtr<T>::operator = (CSmartPtr<T> Other)
    {
        swap(Ptr_, Other.Ptr_);
        return (*this);
    }

    template <typename T>
    inline CSmartPtr<T>& CSmartPtr<T>::operator = (SmartPtr<T> Other)
    {
        T* TempPtr = const_cast<T*>(Ptr_);
        swap(TempPtr, Other.Ptr_);
        Ptr_ = TempPtr;
        return (*this);
    }

    template <typename T>
    inline CSmartPtr<T>& CSmartPtr<T>::operator = (const T* OtherPtr)
    {
        CSmartPtr<T> Dummy(OtherPtr);
        swap(Ptr_, Dummy.Ptr_);
        return (*this);
    }

    template <typename T>
    inline const T* CSmartPtr<T>::GetPtr_() const
    {
        return Ptr_;
    }

    template <typename T>
    inline CSmartPtr<T>::operator const T* () const
    {
        return (GetPtr_());
    }

    template <typename T>
    inline const T* CSmartPtr<T>::operator -> () const
    {
        return (GetPtr_());
    }

    template <typename T>
    inline const T& CSmartPtr<T>::operator * () const
    {
        return (*Ptr_);
    }

    template <typename T>
    template <typename U>
    inline bool CSmartPtr<T>::operator == (const U& Other) const
    {
        return (Compare_(Other) == 0);
    }

    template <typename T>
    template <typename U>
    inline bool CSmartPtr<T>::operator != (const U& Other) const
    {
        return (Compare_(Other) != 0);
    }

    template <typename T>
    template <typename U>
    inline bool CSmartPtr<T>::operator < (const U& Other) const
    {
        return (Compare_(Other) < 0);
    }

    template <typename T>
    template <typename U>
    inline bool CSmartPtr<T>::operator <= (const U& Other) const
    {
        return (Compare_(Other) <= 0);
    }

    template <typename T>
    template <typename U>
    inline bool CSmartPtr<T>::operator > (const U& Other) const
    {
        return (Compare_(Other) > 0);
    }

    template <typename T>
    template <typename U>
    inline bool CSmartPtr<T>::operator >= (const U& Other) const
    {
        return (Compare_(Other) >= 0);
    }

    template <typename T>
    inline bool CSmartPtr<T>::operator == (const T* OtherPtr) const
    {
        return (Compare_(OtherPtr) == 0);
    }

    template <typename T>
    inline bool CSmartPtr<T>::operator != (const T* OtherPtr) const
    {
        return (Compare_(OtherPtr) != 0);
    }

    template <typename T>
    inline bool CSmartPtr<T>::operator < (const T* OtherPtr) const
    {
        return (Compare_(OtherPtr) < 0);
    }

    template <typename T>
    inline bool CSmartPtr<T>::operator <= (const T* OtherPtr) const
    {
        return (Compare_(OtherPtr) <= 0);
    }

    template <typename T>
    inline bool CSmartPtr<T>::operator > (const T* OtherPtr) const
    {
        return (Compare_(OtherPtr) > 0);
    }

    template <typename T>
    inline bool CSmartPtr<T>::operator >= (const T* OtherPtr) const
    {
        return (Compare_(OtherPtr) >= 0);
    }

    template <typename T>
    inline bool CSmartPtr<T>::IsNull_() const
    {
        return (Ptr_ == nullptr);
    }

    template <typename T>
    inline bool CSmartPtr<T>::operator ! () const
    {
        return (IsNull_());
    }

    template <typename T>
    static inline SmartPtr<T> ConstCast(const CSmartPtr<T>& CPtr)
    {
        return SmartPtr<T>(const_cast<T*>(CPtr->GetPtr_()));
    }

    template <typename T>
    static inline ostream& operator << (ostream& Out, const SmartPtr<T>& Ptr)
    {
        Out << Ptr->GetPtr_();
        return Out;
    }

    template <typename T>
    static inline ostream& operator << (ostream& Out, const CSmartPtr<T>& Ptr)
    {
        Out << Ptr->GetPtr_();
        return Out;
    }

} /* end namespace */

#endif /* ESMC_SMART_PTR_HPP_ */

//
// SmartPtr.hpp ends here
