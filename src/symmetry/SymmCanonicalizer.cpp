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
#include "../uflts/LTSChannelEFSM.hpp"
#include "../uflts/LTSExtensions.hpp"
#include "../utils/SizeUtils.hpp"
#include "../utils/CombUtils.hpp"
#include "../mc/StateVec.hpp"
#include "../mc/StateVecPrinter.hpp"
#include "../mc/Compiler.hpp"
#include "../mc/AQStructure.hpp"
#include "../mc/IndexSet.hpp"
#include "../uflts/LTSAssign.hpp"

#include "SymmCanonicalizer.hpp"

namespace ESMC {
    namespace Symm {

        using ESMC::LTS::ExpT;
        using ESMC::MC::StateVecPrinter;

        using ESMC::MC::StateVec;
        using ESMC::Exprs::ExprArrayType;
        using ESMC::Exprs::ExprRecordType;
        using ESMC::Exprs::ExprSymmetricType;
        using ESMC::LTS::LTSExtensionT;
        using ESMC::LTS::LTSTypeExtensionT;
        using ESMC::MC::ProductState;
        using ESMC::MC::ProcessIndexSet;

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
            NumElems = IndexType->GetCardinalityNoUndef();
            ElemSize = ValueType->GetByteSize();
            ElemSize = Align(ElemSize, ElemSize);

            if (!IndexType->Is<ExprSymmetricType>()) {
                PermSize = 0;
                TypeOffset = 0;
                IsSymmArray = false;
            } else {
                IsSymmArray = true;
                PermSize = IndexType->GetCardinalityNoUndef();
                auto TypeExt = IndexType->GetExtension<LTSTypeExtensionT>();
                TypeOffset = TypeExt->TypeOffset;
            }

            for (u32 i = 0; i < NumElems; ++i) {
                u32 CurOffset = Offset + (i * ElemSize);
                auto SubPermuter =
                    PermuterBase::MakePermuter(CurOffset, ValueType, TheLTS);
                ElemPermuters.push_back(SubPermuter);
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

            if (IsSymmArray) {
                // Now apply my own permutation
                // But instead of using the input state vector
                // use a clone of the output state vector computed
                // so far, this way, we retain the permutations
                // performed by the sub permuters

                // Rule: Given a permutation <p1, p2, ... pn>, we
                // interpret this is as a map:
                // Arr[0] |-> p1
                // Arr[1] |-> p2
                // ...
                // Arr[n] |-> pn
                auto NewOutStateVector = OutStateVector->Clone();

                for (u32 i = 0; i < PermSize; ++i) {
                    u32 j = i + TypeOffset;
                    const u08* SrcBasePtr = NewOutStateVector->GetStateBuffer();
                    u08* DstBasePtr = OutStateVector->GetStateBuffer();

                    u32 CurPutPos = Permutation[j];
                    const u08* SrcPtr = (SrcBasePtr + Offset + (ElemSize * i));
                    u08* DstPtr = (DstBasePtr + Offset + (ElemSize * CurPutPos));
                    memcpy(DstPtr, SrcPtr, ElemSize);
                }
                NewOutStateVector->Recycle();
            }
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
                OutStateVector->WriteShort(Offset, (u16)PermVal);
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
            PermSize = SymmType->As<ExprSymmetricType>()->GetCardinalityNoUndef();
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
            if (PermSize <= 255) {
                ActVal = InStateVector->ReadByte(Offset);
            } else if (PermSize <= 65535) {
                ActVal = InStateVector->ReadShort(Offset);
            } else {
                ActVal = InStateVector->ReadWord(Offset);
            }
            if (ActVal == 0) {
                // Undefined, leave it alone
                return;
            } else {
                ActVal -= 1;
            }

            // Add back the offset
            u32 PermVal = Permutation[TypeOffset + ActVal] + 1;
            if (PermSize <= 255) {
                OutStateVector->WriteByte(Offset, (u08)PermVal);
            } else if (PermSize <= 65535) {
                OutStateVector->WriteShort(Offset, (u16)PermVal);
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
                                           u32 Capacity, u32 PermVecOffset,
                                           ChannelEFSM* ChanEFSM, u32 InstanceID)
            : Offset(Offset), Capacity(Capacity), PermVecOffset(PermVecOffset),
              ChanEFSM(ChanEFSM), InstanceID(InstanceID)
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

            for (u32 i = 0; i < Capacity; ++i) {
                LastPermutation.push_back(i);
            }
            IdentityPermutation = LastPermutation;
            ScratchPermutation = LastPermutation;
        }

        ChanBufferSorter::~ChanBufferSorter()
        {
            // Nothing here
        }

        void ChanBufferSorter::Sort(StateVec* OutStateVector,
                                    bool RememberPerm)
        {
            if (RememberPerm) {
                LastPermutation = IdentityPermutation;
            }

            // O(n^2) sorting, I know, but we don't expect these
            // channels to have more than 10 or so elements
            u08* BasePtr = OutStateVector->GetStateBuffer();
            auto WorkingVec = OutStateVector->Clone();
            u08* WorkBasePtr = WorkingVec->GetStateBuffer();

            auto NumElems = Capacity;

            for (u32 i = 0; i < NumElems - 1; ++i) {
                u32 MinIndex = i;
                u32 MinOffset = Offset + (ElemSize * MinIndex);

                for (u32 j = i + 1; j < NumElems; ++j) {
                    u32 JOffset = Offset + (ElemSize * j);
                    auto CmpRes = memcmp(BasePtr + MinOffset, BasePtr + JOffset, ElemSize);
                    if (CmpRes > 0) {
                        MinIndex = j;
                        MinOffset = JOffset;
                    }
                }
                if (MinIndex != i) {
                    u32 IOffset = Offset + (ElemSize * i);
                    memcpy(WorkBasePtr + IOffset, BasePtr + IOffset, ElemSize);
                    memcpy(BasePtr + IOffset, BasePtr + MinOffset, ElemSize);
                    memcpy(BasePtr + MinOffset, WorkBasePtr + IOffset, ElemSize);

                    if (RememberPerm) {
                        swap(LastPermutation[i], LastPermutation[MinIndex]);
                    }
                }
            }

            // Fixup the permutation to mean what we want it to mean
            if (RememberPerm) {
                swap(ScratchPermutation, LastPermutation);
                for (u32 i = 0; i < Capacity; ++i) {
                    LastPermutation[ScratchPermutation[i]] = i;
                }
            }

            WorkingVec->Recycle();
            return;
        }

        const vector<u08>& ChanBufferSorter::GetLastPermutation() const
        {
            return LastPermutation;
        }

        u32 ChanBufferSorter::GetCapacity() const
        {
            return Capacity;
        }

        u32 ChanBufferSorter::GetPermVecOffset() const
        {
            return PermVecOffset;
        }

        void ChanBufferSorter::ApplyPermutation(StateVec* OutStateVector,
                                                const vector<u08>& PermVec)
        {
            u08* BasePtr = OutStateVector->GetStateBuffer();
            auto WorkingVec = OutStateVector->Clone();
            u08* WorkBasePtr = WorkingVec->GetStateBuffer();

            for (u32 i = 0; i < Capacity; ++i) {
                u32 j = i + PermVecOffset;
                u32 CurPutPos = PermVec[j];
                const u08* SrcPtr = (WorkBasePtr + Offset + (ElemSize * i));
                u08* DstPtr = (BasePtr + Offset + (ElemSize * CurPutPos));
                memcpy(DstPtr, SrcPtr, ElemSize);
            }

            WorkingVec->Recycle();
            return;
        }

        vector<LTSAssignRef>
        ChanBufferSorter::GetUpdatesForPermutation(const vector<u08>& PermVec) const
        {
            vector<u08> SubPermVec(Capacity);
            for (u32 i = 0; i < Capacity; ++i) {
                SubPermVec[i] = PermVec[PermVecOffset + i];
            }
            return ChanEFSM->GetUpdatesForPermutation(SubPermVec, InstanceID);
        }

        Canonicalizer::Canonicalizer(const LabelledTS* TheLTS,
                                     StateVecPrinter* Printer)
            : Printer(Printer)
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
            vector<u32> ChanTypeSizes;
            i64 PermutationSize = 1;
            for (auto const& ChanBufferExp : ChansToSort) {
                auto Capacity = get<0>(ChanBufferExp)->GetCapacity();
                ChanTypeSizes.push_back(Capacity);
                PermutationSize *= Factorial(Capacity);
            }
            SortPermSet = new PermutationSet(ChanTypeSizes, (PermutationSize > MaxExplicitSize));

            u32 RunningOffset = 0;
            for (auto const& ChanBufferExp : ChansToSort) {
                auto const& ChanVar = get<1>(ChanBufferExp);
                auto const& CountExp = get<0>(ChanBufferExp)->GetCapacity();
                u32 Offset = ChanVar->ExtensionData.Offset;
                Sorters.push_back(new ChanBufferSorter(Offset, ChanVar->GetType(),
                                                       CountExp, RunningOffset,
                                                       get<0>(ChanBufferExp),
                                                       get<2>(ChanBufferExp)));
                RunningOffset += CountExp;
            }
            LastSortPermutation = vector<u08>(RunningOffset, 0);

            // Create the permutation set
            auto const& SymmTypes = TheLTS->GetUsedSymmTypes();
            // We can be assured that the symmtypes are in the order
            // of their computed type offsets
            vector<u32> TypeSizes(SymmTypes.size());
            PermutationSize = 1;
            for (auto const& SymmType : SymmTypes) {
                u32 CurTypeSize = SymmType->As<ExprSymmetricType>()->GetCardinalityNoUndef();
                TypeSizes[SymmType->GetExtension<LTSTypeExtensionT>()->TypeID] = CurTypeSize;

                PermutationSize *= Factorial(CurTypeSize);
            }
            PermSet = new PermutationSet(TypeSizes, (PermutationSize > MaxExplicitSize));
        }

        Canonicalizer::~Canonicalizer()
        {
            for (auto Permuter : Permuters) {
                delete Permuter;
            }
            for (auto Sorter : Sorters) {
                delete Sorter;
            }
            delete PermSet;
            delete SortPermSet;
        }

        StateVec* Canonicalizer::Canonicalize(const StateVec* InputVector,
                                              u32& PermID) const
        {
            StateVec* BestStateVec = InputVector->Clone();
            auto WorkingStateVec = InputVector->Clone();
            PermID = 0;

            for (auto it = PermSet->Begin(); it != PermSet->End(); ++it) {
                WorkingStateVec->Set(*InputVector);

                // cout << "Canonicalizer: Working state before any permutations are applied "
                //      << endl;
                // cout << "-----------------------------------------" << endl;
                // Printer->PrintState(WorkingStateVec, cout);
                // cout << "-----------------------------------------" << endl;

                for (auto Permuter : Permuters) {
                    Permuter->Permute(InputVector, WorkingStateVec, it);
                    // cout << "Canonicalizer: After applying permuter "
                    //      << "with offset " << Permuter->GetOffset()
                    //      << " and permutation " + PermToString(it.GetPerm())
                    //      << endl;
                    // cout << "-----------------------------------------" << endl;
                    // Printer->PrintState(WorkingStateVec, cout);
                    // cout << "-----------------------------------------" << endl;
                }

                for (auto Sorter : Sorters) {
                    Sorter->Sort(WorkingStateVec, false);
                }

                if (BestStateVec->Compare(*WorkingStateVec) > 0) {
                    BestStateVec->Set(*WorkingStateVec);
                    PermID = it.GetIndex();
                }

                // cout << "Canonicalizer: Current Best State:" << endl;
                // cout << "-----------------------------------------" << endl;
                // Printer->PrintState(BestStateVec, cout);
                // cout << "-----------------------------------------" << endl;
            }

            // if (InputVector->Equals(*BestStateVec)) {
            //     cout << "Canonicalizer: Original and Canonical states are the same!" << endl;
            // } else {
            //     cout << "Canonicalizer: Original State:" << endl;
            //     cout << "-----------------------------------------" << endl;
            //     Printer->PrintState(InputVector, cout);
            //     cout << "-----------------------------------------" << endl;
            //     cout << "Canonicalizer: Canonicalized State:" << endl;
            //     cout << "-----------------------------------------" << endl;
            //     Printer->PrintState(BestStateVec, cout);
            //     cout << "-----------------------------------------" << endl << endl;
            // }


            WorkingStateVec->Recycle();
            InputVector->Recycle();

            return BestStateVec;
        }

        StateVec* Canonicalizer::ApplyPermutation(const StateVec* InputVector, u32 PermID) const
        {
            auto Retval = InputVector->Clone();
            auto Perm = PermSet->GetIterator(PermID);
            for (auto Permuter : Permuters) {
                Permuter->Permute(InputVector, Retval, Perm);
            }
            return Retval;
        }

        ProductState* Canonicalizer::ApplyPermutation(const ProductState *InputPS,
                                                      u32 PermID,
                                                      const ProcessIndexSet* ProcIdxSet) const
        {
            auto InputSV = InputPS->GetSVPtr();
            auto InputTracked = InputPS->GetIndexID();

            auto PermSV = ApplyPermutation(InputSV, PermID);
            auto ThePermIt = PermSet->GetIterator(PermID);
            auto const& ThePerm = ThePermIt.GetPerm();
            auto PermTracked = ProcIdxSet->Permute(InputTracked, ThePerm);
            return new ProductState(PermSV, InputPS->GetMonitorState(),
                                    PermTracked, 0);
        }

        void Canonicalizer::ApplySort(StateVec* InputVector, u32 SortPermID) const
        {
            auto const& Perm = SortPermSet->GetIterator(SortPermID).GetPerm();
            for (auto const& Sorter : Sorters) {
                Sorter->ApplyPermutation(InputVector, Perm);
            }
        }

        void Canonicalizer::ApplySort(ProductState* InputPS, u32 SortPermID) const
        {
            ApplySort(const_cast<StateVec*>(InputPS->GetSVPtr()), SortPermID);
        }

        void Canonicalizer::Sort(StateVec* InputVector) const
        {
            for (auto const& Sorter : Sorters) {
                Sorter->Sort(InputVector, false);
            }
        }

        void Canonicalizer::Sort(ProductState* InputPS) const
        {
            Sort(const_cast<StateVec*>(InputPS->GetSVPtr()));
        }

        StateVec* Canonicalizer::SortChans(const StateVec* InputVector,
                                           bool RememberPerm, u32& PermID) const
        {
            auto Retval = InputVector->Clone();
            u32 PermOffset = 0;
            for (auto Sorter : Sorters) {
                Sorter->Sort(Retval, RememberPerm);
                if (RememberPerm) {
                    const u32 CurPermSize = Sorter->GetCapacity();
                    auto const& CurPerm = Sorter->GetLastPermutation();
                    for (u32 i = 0; i < CurPermSize; ++i) {
                        LastSortPermutation[PermOffset + i] = CurPerm[i];
                    }
                    PermOffset += Sorter->GetCapacity();
                }
            }

            if (RememberPerm) {
                PermID = SortPermSet->GetIndexForPerm(LastSortPermutation);
            }
            return Retval;
        }

        PermutationSet* Canonicalizer::GetPermSet() const
        {
            return PermSet;
        }

        PermutationSet* Canonicalizer::GetSortPermSet() const
        {
            return SortPermSet;
        }

        // Check if SV1 and SV2 are equivalent modulo some permutation
        bool Canonicalizer::StatesEquivalent(const StateVec* SV1, const StateVec* SV2) const
        {
            u32 DummyPerm = 0;
            auto CanonSV1 = Canonicalize(SV1, DummyPerm);
            auto CanonSV2 = Canonicalize(SV2, DummyPerm);
            auto Retval = CanonSV1->Equals(*CanonSV2);
            CanonSV1->Recycle();
            CanonSV2->Recycle();
            return Retval;
        }

        StateVecPrinter* Canonicalizer::GetPrinter() const
        {
            return Printer;
        }

        vector<LTSAssignRef> Canonicalizer::GetChanUpdatesForPermutation(u32 PermIdx) const
        {
            vector<LTSAssignRef> Retval;
            auto const& Permutation = SortPermSet->GetIterator(PermIdx).GetPerm();
            for (auto const& Sorter : Sorters) {
                auto&& CurUpdates = Sorter->GetUpdatesForPermutation(Permutation);
                Retval.insert(Retval.end(), CurUpdates.begin(), CurUpdates.end());
            }

            return Retval;
        }

    } /* end namespace Symm */
} /* end namespace ESMC */

//
// SymmCanonicalizer.cpp ends here
