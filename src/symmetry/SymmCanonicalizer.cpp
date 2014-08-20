// SymmCanonicalizer.cpp --- 
// 
// Filename: SymmCanonicalizer.cpp
// Author: Abhishek Udupa
// Created: Sun Aug 17 18:53:45 2014 (-0400)
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

#include <string.h>

#include "../expr/ExprTypes.hpp"
#include "../uflts/LabelledTS.hpp"
#include "../uflts/LTSExtensions.hpp"
#include "../utils/SizeUtils.hpp"
#include "../mc/StateVec.hpp"
#include "../mc/Compiler.hpp"

#include "SymmCanonicalizer.hpp"

namespace ESMC {
    namespace Symm {

        using ESMC::LTS::ExpT;
        
        using ESMC::MC::StateVec;
        using ESMC::Exprs::ExprArrayType;
        using ESMC::Exprs::ExprRecordType;
        using ESMC::Exprs::ExprSymmetricType;
        using ESMC::LTS::LTSExtensionT;
        using ESMC::LTS::LTSTypeExtensionT;

        // Use explicit permutations upto this size
        const u32 MaxExplicitSize = 1024;

        PermuterBase::PermuterBase(u32 Offset, u32 TypeOffset, u32 PermSize)
            : Offset(Offset), TypeOffset(TypeOffset), PermSize(PermSize)
        {
            // Nothing here
        }

        PermuterBase::~PermuterBase()
        {
            // Nothing here
        }

        u32 PermuterBase::GetOffset() const
        {
            return Offset;
        }
        
        u32 PermuterBase::GetTypeOffset() const
        {
            return TypeOffset;
        }

        u32 PermuterBase::GetPermSize() const
        {
            return PermSize;
        }

        PermuterBase* PermuterBase::MakePermuter(u32 Offset, const ExprTypeRef &Type,
                                                 const LabelledTS* TheLTS)
        {
            auto TypeAsArr = Type->As<ExprArrayType>();
            if (TypeAsArr != nullptr) {
                return new ArrayPermuter(Offset, Type, TheLTS);
            }
            auto TypeAsRec = Type->As<ExprRecordType>();
            if (TypeAsRec != nullptr) {
                return new RecordPermuter(Offset, Type, TheLTS);
            }
            auto TypeAsSym = Type->As<ExprSymmetricType>();
            if (TypeAsSym != nullptr) {
                return new SymmTypePermuter(Offset, Type, TheLTS);
            }
            // This does not need any permutation
            return new NoOpPermuter();
        }

        ArrayPermuter::ArrayPermuter(u32 Offset, const ExprTypeRef& ArrayType, 
                                     const LabelledTS* TheLTS)
            : PermuterBase(Offset, 0, 0)
        {
            auto TypeAsArr = ArrayType->As<ExprArrayType>();
            auto const& IndexType = TypeAsArr->GetIndexType();
            auto const& ValueType = TypeAsArr->GetValueType();
            NumElems = IndexType->GetCardinality();

            if (!IndexType->Is<ExprSymmetricType>()) {
                PermSize = 0;
                TypeOffset = 0;
                ElemSize = 0;
            } else {
                auto TypeAsSym = IndexType->As<ExprSymmetricType>();
                PermSize = TypeAsSym->GetCardinality();
                auto TypeExt = TypeAsSym->GetExtension<LTSTypeExtensionT>();
                TypeOffset = TypeExt->TypeOffset;
                ElemSize = ValueType->GetByteSize();
                ElemSize = Align(ElemSize, ElemSize);
                
                for (u32 i = 0; i < PermSize; ++i) {
                    u32 CurOffset = Offset + (i * ElemSize);
                    auto SubPermuter = 
                        PermuterBase::MakePermuter(CurOffset, ValueType, TheLTS);
                    ElemPermuters.push_back(SubPermuter);
                }
            }
        }

        ArrayPermuter::~ArrayPermuter()
        {
            for (auto const& ElemPermuter : ElemPermuters) {
                delete ElemPermuter;
            }
        }

        u32 ArrayPermuter::GetElemSize() const
        {
            return ElemSize;
        }

        u32 ArrayPermuter::GetNumElems() const
        {
            return NumElems;
        }

        const vector<PermuterBase*>& ArrayPermuter::GetElemPermuters() const
        {
            return ElemPermuters;
        }

        void ArrayPermuter::Permute(const StateVec* InStateVector, 
                                    StateVec* OutStateVector,
                                    const PermutationSet::iterator& CurPerm)
        {
            auto const& Permutation = CurPerm.GetPerm();
            // Apply the permutation on the elements first
            for (auto ElemPermuter : ElemPermuters) {
                ElemPermuter->Permute(InStateVector, OutStateVector, CurPerm);
            }

            // Now apply my own permutation
            // But instead of using the input state vector
            // use a clone of the output state vector computed 
            // so far
            auto NewOutStateVector = OutStateVector->Clone();

            for (u32 i = TypeOffset; i < TypeOffset + PermSize; ++i) {
                const u08* SrcBasePtr = NewOutStateVector->GetStateBuffer();
                u08* DstBasePtr = OutStateVector->GetStateBuffer();

                u32 CurGetPos = Permutation[i];
                const u08* SrcPtr = (SrcBasePtr + Offset + (ElemSize * CurGetPos));
                u08* DstPtr = (DstBasePtr + Offset + (ElemSize * i));
                memcpy(DstPtr, SrcPtr, ElemSize);
            }
            
            NewOutStateVector->GetFactory()->TakeState(NewOutStateVector);
        }

        RecordPermuter::RecordPermuter(u32 Offset, const ExprTypeRef& RecordType,
                                       const LabelledTS* TheLTS)
            : PermuterBase(Offset, 0, 0)
        {
            auto UMType = TheLTS->GetUnifiedMType()->As<Exprs::ExprUnionType>();
            bool IsUMType = (RecordType == TheLTS->GetUnifiedMType());
            auto TypeAsRec = RecordType->As<Exprs::ExprRecordType>();
            auto const& MemberVec = TypeAsRec->GetMemberVec();

            for (auto const& Member : MemberVec) {
                auto const& FieldType = Member.second;
                auto const& FieldName = Member.first;
                auto FieldOffset = Offset + TypeAsRec->GetFieldOffset(Member.first);

                if (IsUMType && FieldName == UMType->GetTypeIDFieldName()) {
                    ElemPermuters.push_back(new MTypePermuter(FieldOffset, FieldType, TheLTS));
                } else {
                    ElemPermuters.push_back(PermuterBase::MakePermuter(FieldOffset, FieldType,
                                                                       TheLTS));
                }
            }

            TypeOffset = 0;
            PermSize = 0;
        }

        RecordPermuter::~RecordPermuter()
        {
            for (auto ElemPermuter : ElemPermuters) {
                delete ElemPermuter;
            }
        }

        void RecordPermuter::Permute(const StateVec* InStateVector, 
                                     StateVec* OutStateVector, 
                                     const PermutationSet::iterator& CurPerm)
        {
            // Just apply the permutation to all the members
            for (auto ElemPermuter : ElemPermuters) {
                ElemPermuter->Permute(InStateVector, OutStateVector, CurPerm);
            }
        }

        const vector<PermuterBase*>& RecordPermuter::GetElemPermuters() const
        {
            return ElemPermuters;
        }

        MTypePermuter::MTypePermuter(u32 Offset, const ExprTypeRef& Type,
                                     const LabelledTS* TheLTS)
            : PermuterBase(Offset, 0, 0),
              MsgCanonMap(TheLTS->GetMsgCanonMap()), 
              TypeSize(Type->GetByteSize())
        {
            // Nothing here
        }

        MTypePermuter::~MTypePermuter()
        {
            // Nothing here
        }

        void MTypePermuter::Permute(const StateVec* InStateVector,
                                    StateVec* OutStateVector,
                                    const PermutationSet::iterator& CurPerm) 
        {
            auto PermIdx = CurPerm.GetIndex();
            u32 ActVal;
            if (TypeSize == 1) {
                ActVal = InStateVector->ReadByte(Offset);
            } else if (TypeSize == 2) {
                ActVal = InStateVector->ReadShort(Offset);
            } else {
                ActVal = InStateVector->ReadWord(Offset);
            }
            u32 PermVal = MsgCanonMap[ActVal][PermIdx];
            if (TypeSize == 1) {
                OutStateVector->WriteByte(Offset, (u08)PermVal);
            } else if (TypeSize == 2) {
                OutStateVector->WriteWord(Offset, (u16)PermVal);
            } else {
                OutStateVector->WriteWord(Offset, PermVal);
            }
        }

        SymmTypePermuter::SymmTypePermuter(u32 Offset, const ExprTypeRef& SymmType,
                                           const LabelledTS* TheLTS)
            : PermuterBase(Offset, 0, 0)
        {
            auto TypeExt = SymmType->GetExtension<LTSTypeExtensionT>();
            TypeOffset = TypeExt->TypeOffset;
            PermSize = SymmType->As<ExprSymmetricType>()->GetCardinality();
        }

        SymmTypePermuter::~SymmTypePermuter()
        {
            // Nothing here
        }

        void SymmTypePermuter::Permute(const StateVec* InStateVector, 
                                     StateVec* OutStateVector, 
                                     const PermutationSet::iterator& CurPerm)
        {
            auto const& Permutation = CurPerm.GetPerm();
            u32 ActVal;
            if (PermSize <= 256) {
                ActVal = InStateVector->ReadByte(Offset);
            } else if (PermSize <= 65536) {
                ActVal = InStateVector->ReadShort(Offset);
            } else {
                ActVal = InStateVector->ReadWord(Offset);
            }
            u32 PermVal = Permutation[TypeOffset + ActVal];
            if (PermSize <= 256) {
                OutStateVector->WriteByte(Offset, (u08)PermVal);
            } else if (PermSize <= 65536) {
                OutStateVector->WriteWord(Offset, (u16)PermVal);
            } else {
                OutStateVector->WriteWord(Offset, PermVal);
            }
        }

        NoOpPermuter::NoOpPermuter()
            : PermuterBase(0, 0, 0)
        {
            // Nothing here
        }

        NoOpPermuter::~NoOpPermuter()
        {
            // Nothing here
        }

        void NoOpPermuter::Permute(const StateVec* InStateVector, 
                                   StateVec* OutStateVector, 
                                   const PermutationSet::iterator& CurPerm)
        {
            // Nothing here
        }

        ChanBufferSorter::ChanBufferSorter(u32 Offset, const ExprTypeRef& ChanBufferType,
                                           const ExpT& CountExp)
            : Offset(Offset), CountExp(CountExp)
        {
            auto TypeAsArray = ChanBufferType->As<ExprArrayType>();
            if (TypeAsArray == nullptr) {
                throw InternalError((string)"Expected Channel buffer type to be an " + 
                                    "array.\nAt: " + __FILE__ + ":" + to_string(__LINE__));
            }
            auto IndexType = TypeAsArray->GetIndexType();
            auto ValueType = TypeAsArray->GetValueType();
            ElemSize = ValueType->GetByteSize();
            ElemSize = Align(ElemSize, ElemSize);
        }

        ChanBufferSorter::~ChanBufferSorter()
        {
            // Nothing here
        }

        void ChanBufferSorter::Sort(StateVec* OutStateVector)
        {
            // O(n^2) sorting, I know, but we don't expect these 
            // channels to have more than 10 or so elements
            u08* BasePtr = OutStateVector->GetStateBuffer();
            auto WorkingVec = OutStateVector->Clone();
            u08* WorkBasePtr = WorkingVec->GetStateBuffer();

            auto NumElems = CountExp->ExtensionData.Interp->EvaluateScalar(OutStateVector);

            for (u32 i = 0; i < NumElems - 1; ++i) {
                u32 MinIndex = i;
                u32 MinOffset = Offset + (ElemSize * MinIndex);

                for (u32 j = i + 1; j < NumElems; ++j) {
                    u32 JOffset = Offset + (ElemSize * j);
                    auto CmpRes = memcmp(BasePtr + MinOffset, BasePtr + JOffset, ElemSize);
                    if (CmpRes < 0) {
                        MinIndex = j;
                        MinOffset = JOffset;
                    }
                }
                if (MinIndex != i) {
                    u32 IOffset = Offset + (ElemSize * i);
                    memcpy(WorkBasePtr + IOffset, BasePtr + IOffset, ElemSize);
                    memcpy(BasePtr + IOffset, BasePtr + MinOffset, ElemSize);
                    memcpy(BasePtr + MinOffset, WorkBasePtr + IOffset, ElemSize);
                }
            }

            WorkingVec->GetFactory()->TakeState(WorkingVec);
            return;
        }

        static i64 Factorial(u32 Size)
        {
            i64 Retval = 1;
            for (u32 i = 1; i <= Size; ++i) {
                Retval *= i;
            }
            return Retval;
        }

        Canonicalizer::Canonicalizer(const LabelledTS* TheLTS)
        {
            auto const& StateVectorVars = TheLTS->GetStateVectorVars();

            u32 Offset = 0;
            for (auto const& StateVectorVar : StateVectorVars) {
                auto CurPermuter = PermuterBase::MakePermuter(Offset, 
                                                              StateVectorVar->GetType(),
                                                              TheLTS);
                Permuters.push_back(CurPermuter);
                Offset += StateVectorVar->GetType()->GetByteSize();
            }

            // Assumption: All expressions have been 
            // compiled and therefore have offsets attached
            auto const& ChansToSort = TheLTS->GetChanBuffersToSort();
            for (auto const& ChanVarCount : ChansToSort) {
                auto const& ChanVar = ChanVarCount.first;
                auto const& CountExp = ChanVarCount.second;
                u32 Offset = ChanVar->ExtensionData.Offset;
                Sorters.push_back(new ChanBufferSorter(Offset, ChanVar->GetType(),
                                                       CountExp));
            }

            // Create the permutation set
            auto const& SymmTypes = TheLTS->GetUsedSymmTypes();
            // We can be assured that the symmtypes are in the order
            // of their computed type offsets
            vector<u32> TypeSizes(SymmTypes.size());
            i64 PermutationSize = 1;
            for (auto const& SymmType : SymmTypes) {
                u32 CurTypeSize = SymmType->As<ExprSymmetricType>()->GetCardinality();
                TypeSizes[SymmType->GetExtension<LTSTypeExtensionT>()->TypeID] = CurTypeSize;

                PermutationSize *= Factorial(CurTypeSize);
            }
            PermSet = PermutationSet(TypeSizes, (PermutationSize > MaxExplicitSize));
        }

        Canonicalizer::~Canonicalizer()
        {
            for (auto Permuter : Permuters) {
                delete Permuter;
            }
            for (auto Sorter : Sorters) {
                delete Sorter;
            }
        }

        StateVec* Canonicalizer::Canonicalize(const StateVec* InputVector,
                                              u32& PermID) const
        {
            auto Factory = InputVector->GetFactory();
            StateVec* BestStateVec = InputVector->Clone();
            auto WorkingStateVec = InputVector->Clone();

            for (auto it = PermSet.Begin(); it != PermSet.End(); ++it) {
                WorkingStateVec->Set(*InputVector);

                for (auto Permuter : Permuters) {
                    Permuter->Permute(InputVector, WorkingStateVec, it);
                }
                for (auto Sorter : Sorters) {
                    Sorter->Sort(WorkingStateVec);
                }
                if (BestStateVec->Compare(*WorkingStateVec) < 0) {
                    BestStateVec->Set(*WorkingStateVec);
                    PermID = it.GetIndex();
                }
            }

            Factory->TakeState(WorkingStateVec);
            Factory->TakeState(InputVector);

            return BestStateVec;
        }

        StateVec* Canonicalizer::ApplyPermutation(const StateVec* InputVector, u32 PermID) const
        {
            auto Retval = InputVector->Clone();
            auto const& Perm = PermSet.GetIterator(PermID);
            for (auto Permuter : Permuters) {
                Permuter->Permute(InputVector, Retval, Perm);
            }
            return Retval;
        }

        StateVec* Canonicalizer::ApplyInvPermutation(const StateVec* InputVector, u32 PermID) const
        {
            auto Retval = InputVector->Clone();
            auto const& Perm = PermSet.GetIteratorForInv(PermID);
            for (auto Permuter : Permuters) {
                Permuter->Permute(InputVector, Retval, Perm);
            }
            return Retval;
        }

    } /* end namespace Symm */
} /* end namespace ESMC */

// 
// SymmCanonicalizer.cpp ends here
