// LabelledTS.cpp ---
//
// Filename: LabelledTS.cpp
// Author: Abhishek Udupa
// Created: Mon Aug 11 11:11:33 2014 (-0400)
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

#include <boost/algorithm/string/predicate.hpp>

#include "../utils/CombUtils.hpp"
#include "../symmetry/Permutations.hpp"
#include "../mc/Compiler.hpp"

#include "LabelledTS.hpp"
#include "LTSUtils.hpp"
#include "LTSEFSM.hpp"
#include "LTSChannelEFSM.hpp"
#include "LTSTransitions.hpp"
#include "LTSExtensions.hpp"
#include "LTSFairnessSet.hpp"

namespace ESMC {
    namespace LTS {

        const string LabelledTS::ProductMsgName = "__trans_msg__";

        using ESMC::Symm::PermutationSet;

        LabelledTS::LabelledTS()
            : Mgr(new MgrT()),
              Frozen(false), MsgsFrozen(false), AutomataFrozen(false),
              InvariantExp(Mgr->MakeTrue()), FinalCondExp(Mgr->MakeTrue())
        {
            // Nothing here
        }

        LabelledTS::~LabelledTS()
        {
            delete Mgr;
            for (auto const& NameEFSM : AllEFSMs) {
                delete NameEFSM.second;
            }
        }

        u32 LabelledTS::GetAutomataClassUID()
        {
            auto Retval = AutomatonClassIDGen.GetUID();
            return Retval;
        }

        const string& LabelledTS::GetProductMsgName() const
        {
            return ProductMsgName;
        }

        const vector<vector<u32>>& LabelledTS::GetMsgCanonMap() const
        {
            return MsgCanonMap;
        }

        const vector<string>& LabelledTS::GetMsgTypeMap() const
        {
            return MsgTypeMap;
        }

        // helpers
        void LabelledTS::AssertFrozen() const
        {
            if (!Frozen) {
                throw ESMCError((string)"Operation cannot be performed before freezing " +
                                "the Labelled Transition System");
            }
        }

        void LabelledTS::AssertNotFrozen() const
        {
            if (Frozen) {
                throw ESMCError((string)"Operation cannot be performed after freezing " +
                                "the Labelled Transition System");
            }

        }

        void LabelledTS::AssertMsgsFrozen() const
        {
            if (!MsgsFrozen) {
                throw ESMCError((string)"Operation cannot be performed before freezing " +
                                "messages of the Labelled Transition System");
            }
        }

        void LabelledTS::AssertMsgsNotFrozen() const
        {
            if (MsgsFrozen) {
                throw ESMCError((string)"Operation cannot be performed after freezing " +
                                "messages of the Labelled Transition System");
            }
        }

        void LabelledTS::AssertAutomataFrozen() const
        {
            if (!AutomataFrozen) {
                throw ESMCError((string)"Operation cannot be performed before freezing " +
                                "the automata of the Labelled Transition System");
            }
        }

        void LabelledTS::AssertAutomataNotFrozen() const
        {
            if (AutomataFrozen) {
                throw ESMCError((string)"Operation cannot be performed after freezing " +
                                "the automata of the Labelled Transition System");
            }
        }

        void LabelledTS::CheckConsistency() const
        {
            set<ExprTypeRef> Inputs;
            set<ExprTypeRef> Outputs;
            for (auto const& NameEFSM : AllEFSMs) {
                auto EFSM = NameEFSM.second;
                auto const& ParamInsts = EFSM->GetParamInsts();
                for (auto const& ParamInst : ParamInsts) {
                    auto const& InpSet = EFSM->Inputs[ParamInst];
                    auto const& OutSet = EFSM->Outputs[ParamInst];

                    for (auto const& Output : OutSet) {
                        if (Outputs.find(Output) != Outputs.end()) {
                            throw ESMCError((string)"Multiple EFSMs have message type \"" +
                                            Output->SAs<ExprRecordType>()->GetName() +
                                            "\" defined as an output");
                        }
                        Outputs.insert(Output);
                    }

                    Inputs.insert(InpSet.begin(), InpSet.end());
                }
            }

            if ((!includes(Inputs.begin(), Inputs.end(),
                           Outputs.begin(), Outputs.end())) ||
                (!includes(Outputs.begin(), Outputs.end(),
                           Inputs.begin(), Inputs.end()))) {

                ostringstream sstr;
                sstr << "The LTS is not closed." << endl;
                vector<ExprTypeRef> InputsMinusOutputs;
                vector<ExprTypeRef> OutputsMinusInputs;
                set_difference(Inputs.begin(), Inputs.end(),
                               Outputs.begin(), Outputs.end(),
                               back_inserter(InputsMinusOutputs));
                set_difference(Outputs.begin(), Outputs.end(),
                               Inputs.begin(), Inputs.end(),
                               back_inserter(OutputsMinusInputs));
                if (InputsMinusOutputs.size() > 0) {
                    sstr << "The following types are the output of no EFSM:" << endl;
                    for (auto const& Type : InputsMinusOutputs) {
                        sstr << Type->SAs<ExprRecordType>()->GetName() << endl;
                    }
                }

                if (OutputsMinusInputs.size() > 0) {
                    sstr << "The following types are the input of no EFSM:" << endl;
                    for (auto const& Type : OutputsMinusInputs) {
                        sstr << Type->SAs<ExprRecordType>()->GetName() << endl;
                    }
                }

                throw ESMCError(sstr.str());
            }

            return;
        }

        GCmdRef
        LabelledTS::MakeGuardedCommand(const vector<LTSTransRef>& ProductTrans) const
        {
            vector<ExpT> GuardComps;
            vector<LTSAssignRef> UpdateComps;
            ExprTypeRef MsgType = ExprTypeRef::NullPtr;

            // All the fields of the message are initially set to be
            // undefined
            auto MsgLHS = Mgr->MakeVar(ProductMsgName, UnifiedMsgType);
            auto MsgRHS = Mgr->MakeVal("clear", UnifiedMsgType);
            LTSAssignRef MsgClear = new LTSAssignSimple(MsgLHS, MsgRHS);
            UpdateComps = MsgClear->ExpandNonScalarUpdates();

            for (auto const& Trans : ProductTrans) {
                if (!Trans->Is<LTSTransitionIOBase>()) {
                    throw InternalError((string)"Expected transition to be an IO " +
                                        "transition.\nAt: " + __FILE__ + ":" +
                                        to_string(__LINE__));
                }
                auto TransAsIO = Trans->SAs<LTSTransitionIOBase>();
                auto const& MsgName = TransAsIO->GetMessageName();
                if (MsgType == ExprTypeRef::NullPtr) {
                    MsgType = TransAsIO->GetMessageType();
                } else {
                    if (MsgType != TransAsIO->GetMessageType()) {
                        throw InternalError((string)"Error in cross product construction. " +
                                            "Expected all product transitions to be on the " +
                                            "same message type.\nAt: " + __FILE__ + ":" +
                                            to_string(__LINE__));
                    }
                }

                MgrT::SubstMapT SubstMap;
                SubstMap[Mgr->MakeVar(MsgName, UnifiedMsgType)] =
                    Mgr->MakeVar(ProductMsgName, UnifiedMsgType);

                GuardComps.push_back(Mgr->Substitute(SubstMap, TransAsIO->GetGuard()));
                auto const& OldUpdates = TransAsIO->GetUpdates();

                for (auto const& Update : OldUpdates) {
                    auto SubstLHS = Mgr->Substitute(SubstMap, Update->GetLHS());
                    auto SubstRHS = Mgr->Substitute(SubstMap, Update->GetRHS());
                    UpdateComps.push_back(new LTSAssignSimple(SubstLHS, SubstRHS));
                }
            }

            // Get the fairness sets
            auto OutputTrans = ProductTrans[0]->As<LTSTransitionOutput>();
            auto OutputEFSM = OutputTrans->GetAutomaton()->As<EFSMBase>();
            auto const& ParamInst = OutputTrans->GetParamInst();
            auto const& StrFairnesses = OutputTrans->GetCompOfFairnessSets();

            set<LTSFairObjRef> ActualFairnesses;
            for (auto const& StrFairness : StrFairnesses) {
                auto const& ActFairness =
                    OutputEFSM->Fairnesses->GetFairnessObj(StrFairness, ParamInst);
                ActualFairnesses.insert(ActFairness);
            }
            auto UMTypeAsUnion = UnifiedMsgType->As<ExprUnionType>();
            auto MsgTypeID = UMTypeAsUnion->GetTypeIDForMemberType(MsgType);
            return (new LTSGuardedCommand(Mgr, GuardComps, UpdateComps, MsgType,
                                          MsgTypeID, ActualFairnesses, ProductTrans));
        }

        void LabelledTS::Freeze()
        {
            AssertAutomataFrozen();
            if (Frozen) {
                return;
            }
            Frozen = true;
            u32 GCmdCounter = 0;

            // We now compute the product
            // which will be represented as a
            // list of guarded commands

            // First check consistency of EFSMs
            CheckConsistency();

            // for each message type, get the transitions
            // on the message type from each efsm

            for (auto const& NameType : MsgTypes) {
                vector<vector<LTSTransRef>> TransForCP;

                auto const& MType = NameType.second;

                // Get the output transitions first
                for (auto const& NameEFSM : AllEFSMs) {
                    auto EFSM = NameEFSM.second;

                    auto&& OutputTrans = EFSM->GetOutputTransitionsOnMsg(MType);
                    if (OutputTrans.size() > 0) {
                        TransForCP.push_back(OutputTrans);
                        break;
                    }
                }

                const bool HasOutput = (TransForCP.size() > 0);

                // Now get the input transitions on all EFSMs
                for (auto const& NameEFSM : AllEFSMs) {
                    auto EFSM = NameEFSM.second;
                    auto&& InputTrans = EFSM->GetInputTransitionsOnMsg(MType);
                    if (InputTrans.size() > 0) {
                        TransForCP.insert(TransForCP.end(), InputTrans.begin(),
                                          InputTrans.end());
                    }
                }

                if (TransForCP.size() > 0 && !HasOutput) {
                    throw ESMCError((string)"Message type \"" +
                                    MType->SAs<ExprRecordType>()->GetName() +
                                    "\" is the output of no EFSM but is used as an " +
                                    "input to one or more EFSMs");
                }

                auto&& CPTrans = CrossProduct<LTSTransRef>(TransForCP.begin(), TransForCP.end());
                for (auto const& CPElem : CPTrans) {
                    GuardedCommands.push_back(MakeGuardedCommand(CPElem));
                    GuardedCommands.back()->SetCmdID(GCmdCounter++);
                }
            }

            // Push the internal transitions as well
            for (auto const& NameEFSM : AllEFSMs) {
                auto EFSM = NameEFSM.second;
                auto&& CurTrans = EFSM->GetInternalTransitions();
                for (auto const& Trans : CurTrans) {
                    auto TransAsInt = Trans->As<LTSTransitionInternal>();
                    auto const& StrFairnesses = TransAsInt->GetCompOfFairnessSets();
                    auto const& ParamInst = Trans->GetParamInst();

                    set<LTSFairObjRef> ActualFairSet;
                    for (auto const& StrFairness : StrFairnesses) {
                        auto const& ActFairness =
                            EFSM->Fairnesses->GetFairnessObj(StrFairness, ParamInst);
                        ActualFairSet.insert(ActFairness);
                    }

                    auto CurGCmd = new LTSGuardedCommand(Mgr,
                                                         { Trans->GetGuard() },
                                                         Trans->GetUpdates(),
                                                         ExprTypeRef::NullPtr,
                                                         -1, ActualFairSet,
                                                         { Trans });
                    GuardedCommands.push_back(CurGCmd);
                    GuardedCommands.back()->SetCmdID(GCmdCounter++);
                }
            }

            // Compile the guarded commands
            auto Compiler = new MC::LTSCompiler();
            GuardedCommands = Compiler->CompileCommands(GuardedCommands, this);
            delete Compiler;

            // Build the invariant and final condition
            for (auto const& NameEFSM : AllEFSMs) {
                auto EFSM = NameEFSM.second;
                InvariantExp = Mgr->MakeExpr(LTSOps::OpAND, InvariantExp,
                                             Mgr->MakeExpr(LTSOps::OpNOT,
                                                           EFSM->ErrorCondition));
                FinalCondExp = Mgr->MakeExpr(LTSOps::OpAND, FinalCondExp,
                                             EFSM->FinalCondition);
            }

            InvariantExp = Mgr->Simplify(InvariantExp);
            FinalCondExp = Mgr->Simplify(FinalCondExp);

            // unify the constraints by op for each incomplete efsm
            for (auto const& NameEFSM : AllEFSMs) {
                auto EFSM = NameEFSM.second;
                if (!EFSM->Is<IncompleteEFSM>()) {
                    continue;
                }

                auto const& CurConstraints =
                    EFSM->SAs<IncompleteEFSM>()->GetConstraintsByGuardOp();
                for (auto const& OpConstraints : CurConstraints) {
                    ConstraintsByOp[OpConstraints.first] = OpConstraints.second;
                }

                auto const& CurUpdateOpToLValue =
                    EFSM->SAs<IncompleteEFSM>()->GetUpdateOpToUpdateLValue();
                for (auto const& OpLValPair : CurUpdateOpToLValue) {
                    UpdateOpToUpdateLValue[OpLValPair.first] = OpLValPair.second;
                }
            }
        }

        const ExpT& LabelledTS::GetInvariant() const
        {
            return InvariantExp;
        }

        const ExpT& LabelledTS::GetFinalCond() const
        {
            return FinalCondExp;
        }

        const set<ExprTypeRef>& LabelledTS::GetUsedSymmTypes() const
        {
            return UsedSymmTypes;
        }

        u32 LabelledTS::GetStateVectorSize() const
        {
            return StateVectorSize;
        }

        MgrT::SubstMapT LabelledTS::ApplyPerm(const vector<vector<ExpT>>& ParamElems,
                                              const vector<u08>& Perm)
        {
            MgrT::SubstMapT Retval;
            for (auto const& ParamElemVec : ParamElems) {
                auto const& ParamType = ParamElemVec[0]->GetType();
                auto Extension = ParamType->GetExtension<LTSTypeExtensionT>();
                const i32 Offset = Extension->TypeOffset;
                for (u32 i = 0; i < ParamElemVec.size(); ++i) {
                    Retval[ParamElemVec[i]] = ParamElemVec[Perm[Offset + i]];
                }
            }
            return Retval;
        }

        void LabelledTS::MakeMsgCanonMap()
        {
            auto UMType = UnifiedMsgType->As<ExprUnionType>();
            const u32 NumMsgTypes = UMType->GetMemberTypes().size();
            MsgCanonMap = vector<vector<u32>>(NumMsgTypes + 1);

            vector<u32> TypeSizes;
            for (auto const& Type : UsedSymmTypes) {
                TypeSizes.push_back(Type->GetCardinalityNoUndef());
            }

            PermutationSet PermSet(TypeSizes, true);
            const u32 NumPerms = PermSet.GetSize();
            for (u32 i = 0; i < NumMsgTypes + 1; ++i) {
                MsgCanonMap[i] = vector<u32>(NumPerms);
            }
            for (u32 i = 0; i < NumPerms; ++i) {
                MsgCanonMap[0][i] = 0;
            }

            vector<vector<ExpT>> ParamElems;
            for (auto const& Type : UsedSymmTypes) {
                ParamElems.push_back(vector<ExpT>());
                auto&& Elems = Type->GetElementsNoUndef();
                for (auto const Elem : Elems) {
                    ParamElems.back().push_back(Mgr->MakeVal(Elem, Type));
                }
            }

            for (auto const& NameType : MsgTypes) {
                auto const& CurMType = NameType.second;
                const u32 CurMTypeID = UMType->GetTypeIDForMemberType(CurMType);
                auto it1 = PInstToParams.find(CurMType);
                if (it1 == PInstToParams.end()) {
                    for (u32 j = 0; j < NumPerms; ++j) {
                        MsgCanonMap[CurMTypeID][j] = CurMTypeID;
                    }
                } else {
                    auto const& InstParams = it1->second;
                    auto const& ParamType = PInstToParamType[CurMType];
                    for (auto it2 = PermSet.Begin(); it2 != PermSet.End(); ++it2) {
                        const u32 CurPermIndex = it2.GetIndex();
                        auto const& CurPerm = it2.GetPerm();

                        auto SubstMap = ApplyPerm(ParamElems, CurPerm);

                        vector<ExpT> PermParams;
                        for (auto const& InstParam : InstParams) {
                            if (SubstMap.find(InstParam) == SubstMap.end()) {
                                // Not a part of the symmetry
                                // set up an identity mapping
                                PermParams.push_back(InstParam);
                            } else {
                                PermParams.push_back(SubstMap[InstParam]);
                            }
                        }

                        auto PermType = Mgr->InstantiateType(ParamType, PermParams);
                        u32 PermTypeID = UMType->GetTypeIDForMemberType(PermType);
                        MsgCanonMap[CurMTypeID][CurPermIndex] = PermTypeID;

                        // cout << CurMType->ToString() << " -> " << PermType->ToString()
                        //      << " on permutation " << Symm::PermToString(CurPerm) << endl;
                    }
                }
            }

            // We also populate the msg type map
            MsgTypeMap = vector<string>(NumMsgTypes + 1);
            MsgTypeMap[0] = "undefined_mtype";
            for (auto const& NameType : MsgTypes) {
                auto const& Type = NameType.second;
                auto const& Name = NameType.first;

                const u32 TypeID = UMType->GetTypeIDForMemberType(Type);
                MsgTypeMap[TypeID] = Name;
            }
        }

        void LabelledTS::FreezeAutomata()
        {
            AssertMsgsFrozen();
            if (AutomataFrozen) {
                return;
            }

            AutomataFrozen = true;

            StateVectorSize = 0;
            // Create the state variables
            for (auto& NameEFSM : AllEFSMs) {
                auto EFSM = NameEFSM.second;
                EFSM->Freeze();
                auto const& Name = EFSM->Name;
                auto const& StateVarType = EFSM->StateVarType;

                // Make the state variables
                auto CurStateVar = Mgr->MakeVar(Name, StateVarType);
                // Not strictly our job, but it's easiest to
                // add the extension data right here
                CurStateVar->ExtensionData.Offset = StateVectorSize;
                cout << "Var \"" << Name << "\" assigned to offset " << StateVectorSize
                     << " - " << StateVectorSize + StateVarType->GetByteSize() << endl;
                StateVectorVars.push_back(CurStateVar);
                StateVectorSize += StateVarType->GetByteSize();
                ValidAutomata[EFSM->Name] =
                    set<vector<ExpT>>(EFSM->ParamInsts.begin(),
                                      EFSM->ParamInsts.end());

                SymTab.Bind(Name, new VarDecl(Name, StateVarType));


                // Add any symmetric parameters to the list of
                // symmetric types marked as used

                auto const& EFSMParams = EFSM->Params;
                u64 TotalPermSize = 1;
                for (auto const& Param : EFSMParams) {
                    UsedSymmTypes.insert(Param->GetType());
                    TotalPermSize *= (Factorial(Param->GetType()->GetCardinalityNoUndef()));
                    if (TotalPermSize > UINT32_MAX) {
                        throw ESMCError((string)"Product of permutation sizes of " +
                                        "symmetric types exceeds the maximum supported " +
                                        "value");
                    }
                }
                auto const& Decls = EFSM->SymTab.Bot()->GetDeclMap();
                for (auto const& Decl : Decls) {
                    if (Decl.second->Is<VarDecl>() &&
                        Decl.second->GetType()->Is<ExprSymmetricType>()) {
                        UsedSymmTypes.insert(Decl.second->GetType());
                    }
                }
            }

            // Again, not strictly our job, but it makes it
            // easier to add the extension data to the types
            // right here as well
            u32 TypeIDCounter = 0;
            u32 TypeOffsetCounter = 0;
            for (auto const& SymmType : UsedSymmTypes) {
                SymmType->AddExtension(new LTSTypeExtensionT());
                auto Extension = SymmType->GetExtension<LTSTypeExtensionT>();
                Extension->TypeID = TypeIDCounter++;
                Extension->TypeOffset = TypeOffsetCounter;
                TypeOffsetCounter += SymmType->As<ExprSymmetricType>()->GetCardinalityNoUndef();
            }

            // Make a list of channel buffers that require
            // to be sorted
            for (auto& ChanEFSM : ChannelEFSMs) {
                auto Chan = ChanEFSM.second;
                if (!Chan->Ordered) {
                    auto const& Name = Chan->Name;
                    for (auto const& ParamInst : Chan->ParamInsts) {
                        auto VarExp = Mgr->MakeVar(Name, Chan->StateVarType);
                        for (auto const& Param : ParamInst) {
                            VarExp = Mgr->MakeExpr(LTSOps::OpIndex, VarExp, Param);
                        }
                        auto FAType = Mgr->MakeType<Exprs::ExprFieldAccessType>();
                        auto BufferExp = Mgr->MakeExpr(LTSOps::OpField, VarExp,
                                                       Mgr->MakeVar("MsgBuffer", FAType));
                        auto CountExp = Mgr->MakeExpr(LTSOps::OpField, VarExp,
                                                      Mgr->MakeVar("MsgCount", FAType));

                        ChanBuffersToSort.push_back(make_pair(BufferExp, CountExp));
                    }
                }
            }

            MakeMsgCanonMap();
        }

        void LabelledTS::FreezeMsgs()
        {
            if (MsgsFrozen) {
                return;
            }

            MsgsFrozen = true;

            // create the union type
            set<ExprTypeRef> UnionMembers;
            for (auto const& MsgType : MsgTypes) {
                UnionMembers.insert(MsgType.second);
            }

            auto TypeIDFieldType = Mgr->MakeType<ExprRangeType>(0, 65000);
            UnifiedMsgType = Mgr->MakeType<ExprUnionType>("UnifiedMsgType",
                                                          UnionMembers,
                                                          TypeIDFieldType);
        }

        const vector<vector<LTSAssignRef>>&
        LabelledTS::GetInitStateGenerators() const
        {
            return InitStateGenerators;
        }

        const vector<GCmdRef>& LabelledTS::GetGuardedCmds() const
        {
            return GuardedCommands;
        }

        const vector<pair<ExpT, ExpT>>& LabelledTS::GetChanBuffersToSort() const
        {
            return ChanBuffersToSort;
        }

        const vector<ExpT>& LabelledTS::GetStateVectorVars() const
        {
            return StateVectorVars;
        }

        void LabelledTS::CheckTypeName(const string& Name) const
        {
            if (boost::algorithm::contains(Name, "]") ||
                boost::algorithm::contains(Name, "[")) {
                throw ESMCError((string)"Type names cannot contain '[' or ']' characters");
            }
            if (boost::algorithm::ends_with(Name, "StateT")) {
                throw ESMCError((string)"Type names cannot end with \"StateT\"");
            }
            if (Name == "UnifiedMsgType") {
                throw ESMCError((string)"Type name \"UnifiedMsgType\" is reserved");
            }
            if (NamedTypes.find(Name) != NamedTypes.end()) {
                throw ESMCError((string)"A type named \"" + Name + "\" already exists. " +
                                "Use GetNamedType() to retrieve it");
            }
        }

        ExprTypeRef LabelledTS::MakeBoolType()
        {
            return Mgr->MakeType<ExprBoolType>();
        }

        ExprTypeRef LabelledTS::MakeRangeType(i64 Low, i64 High)
        {
            if (High - Low + 1 >= UINT32_MAX ||
                High >= INT64_MAX / 2) {
                throw ESMCError((string)"Bounds for range type exceed limits. " +
                                "We require High - Low + 1 < " + to_string(UINT32_MAX) +
                                " and HIGH < " + to_string(INT64_MAX / 2));
            }
            return Mgr->MakeType<ExprRangeType>(Low, High);
        }

        ExprTypeRef LabelledTS::MakeArrayType(const ExprTypeRef& IndexType,
                                              const ExprTypeRef& ValueType)
        {
            return Mgr->MakeType<ExprArrayType>(IndexType, ValueType);
        }

        ExprTypeRef LabelledTS::MakeEnumType(const string& Name,
                                             const set<string>& Members)
        {
            AssertNotFrozen();
            CheckTypeName(Name);
            auto Retval = Mgr->MakeType<ExprEnumType>(Name, Members);
            NamedTypes[Name] = Retval;
            return Retval;
        }

        ExprTypeRef LabelledTS::MakeRecordType(const string& Name,
                                               const vector<pair<string, ExprTypeRef>>& Members)
        {
            AssertNotFrozen();
            CheckTypeName(Name);
            auto Retval = Mgr->MakeType<ExprRecordType>(Name, Members);
            NamedTypes[Name] = Retval;
            return Retval;
        }

        ExprTypeRef LabelledTS::MakeFieldAccessType()
        {
            return Mgr->MakeType<ExprFieldAccessType>();
        }

        MgrT* LabelledTS::GetMgr() const
        {
            return Mgr;
        }

        bool LabelledTS::CheckMessageType(const ExprTypeRef& MsgType) const
        {
            AssertMsgsFrozen();

            if (MsgType->Is<ExprParametricType>()) {
                auto TypeAsPar = MsgType->SAs<ExprParametricType>();
                auto BaseRecType = TypeAsPar->GetBaseType()->As<ExprRecordType>();
                auto it = ParametricMsgTypes.find(BaseRecType->GetName());
                if (it == ParametricMsgTypes.end()) {
                    return false;
                } else {
                    return true;
                }
            } else if (MsgType->Is<ExprRecordType>()) {
                auto TypeAsRec = MsgType->SAs<ExprRecordType>();
                auto it = MsgTypes.find(TypeAsRec->GetName());
                if (it == MsgTypes.end()) {
                    return false;
                } else {
                    return true;
                }
            } else {
                return false;
            }
        }

        const ExprTypeRef& LabelledTS::GetUnifiedMType() const
        {
            AssertMsgsFrozen();
            return UnifiedMsgType;
        }

        const ExprTypeRef& LabelledTS::MakeSymmType(const string& Name,
                                                    u32 Size)
        {
            AssertNotFrozen();
            CheckTypeName(Name);

            if (Size > 12) {
                throw ESMCError((string)"Symmetric types of size > 12 cannot " +
                                "be created. The factorial representation gets " +
                                "too large");
            }

            NamedTypes[Name] = SymmTypes[Name] =
                Mgr->MakeType<ExprSymmetricType>(Name, Size);

            return SymmTypes[Name];
        }

        const ExprTypeRef& LabelledTS::MakeMsgType(const string& Name,
                                                   const vector<pair<string, ExprTypeRef>> &Members,
                                                   bool IncludePrimed)
        {
            AssertMsgsNotFrozen();
            CheckTypeName(Name);

            if (MsgTypes.find(Name) != MsgTypes.end() ||
                ParametricMsgTypes.find(Name) != ParametricMsgTypes.end()) {
                throw ESMCError((string)"Message type name \"" + Name + "\" has already been " +
                                "created");
            }

            for (auto const& Member : Members) {
                if (!Member.second->Is<ExprScalarType>()) {
                    throw ESMCError((string)"Message fields can only be scalar typed");
                }
            }

            NamedTypes[Name] = MsgTypes[Name] = Mgr->MakeType<ExprRecordType>(Name, Members);
            if (IncludePrimed) {
                auto PrimedName = Name + "'";
                NamedTypes[PrimedName] = MsgTypes[PrimedName] =
                    Mgr->MakeType<ExprRecordType>(PrimedName, Members);
                TypeToPrimed[MsgTypes[Name]] = MsgTypes[PrimedName];
            }

            return MsgTypes[Name];
        }

        const ExprTypeRef& LabelledTS::MakeMsgTypes(const vector<ExpT>& Params,
                                                    const ExpT& Constraint,
                                                    const string& Name,
                                                    const vector<pair<string, ExprTypeRef>>& Members,
                                                    bool IncludePrimed)
        {
            AssertMsgsNotFrozen();
            CheckTypeName(Name);

            SymTab.Push();
            CheckParams(Params, Constraint, SymTab, Mgr, true);
            SymTab.Pop();

            auto&& ParamInsts = InstantiateParams(Params, Constraint, Mgr);
            if (ParamInsts.size() == 0) {
                throw ESMCError((string)"Instantiation of parametric type \"" + Name +
                                "\" is empty");
            }

            if (ParametricMsgTypes.find(Name) != ParametricMsgTypes.end() ||
                MsgTypes.find(Name) != MsgTypes.end()) {
                throw ESMCError((string)"Message type name \"" + Name + "\" has already been " +
                                "created");
            }

            for (auto const& Member : Members) {
                if (!Member.second->Is<ExprScalarType>()) {
                    throw ESMCError((string)"Message fields can only be scalar typed");
                }
            }

            auto BaseRecType = Mgr->MakeType<ExprRecordType>(Name, Members);
            vector<ExprTypeRef> ParamTypes;
            for (auto const& Param : Params) {
                ParamTypes.push_back(Param->GetType());
            }

            auto PType = Mgr->MakeType<ExprParametricType>(BaseRecType, ParamTypes);
            NamedTypes[Name] = ParametricMsgTypes[Name] = PType;

            ExprTypeRef PBaseRecType;
            ExprTypeRef PPType = nullptr;
            string PName;
            if (IncludePrimed) {
                PName = Name + "'";
                PBaseRecType = Mgr->MakeType<ExprRecordType>(PName, Members);
                PPType = Mgr->MakeType<ExprParametricType>(PBaseRecType, ParamTypes);
                NamedTypes[PName] = ParametricMsgTypes[PName] = PPType;
                TypeToPrimed[ParametricMsgTypes[Name]] = ParametricMsgTypes[PName];
            }

            for (auto const& ParamInst : ParamInsts) {
                auto InstType = InstantiateType(PType, ParamInst, Mgr);
                auto const& InstName = InstType->As<ExprRecordType>()->GetName();
                MsgTypes[InstName] = NamedTypes[InstName] = InstType;
                ParamTypeInsts[PType].push_back(InstType);
                PInstToParams[InstType] = ParamInst;
                PInstToParamType[InstType] = PType;

                if (IncludePrimed) {
                    auto PInstType = InstantiateType(PPType, ParamInst, Mgr);
                    auto const& PInstName = PInstType->As<ExprRecordType>()->GetName();
                    NamedTypes[PInstName] = MsgTypes[PInstName] = PInstType;
                    TypeToPrimed[MsgTypes[InstType->As<ExprRecordType>()->GetName()]] =
                        MsgTypes[PInstType->As<ExprRecordType>()->GetName()];
                    ParamTypeInsts[PPType].push_back(PInstType);
                    PInstToParams[PInstType] = ParamInst;
                    PInstToParamType[PInstType] = PPType;
                }
            }

            return ParametricMsgTypes[Name];
        }

        const ExprTypeRef& LabelledTS::GetNamedType(const string& TypeName) const
        {
            auto it = NamedTypes.find(TypeName);
            if (it == NamedTypes.end()) {
                return ExprTypeRef::NullPtr;
            }
            return it->second;
        }

        const ExprTypeRef& LabelledTS::GetEFSMType(const string& EFSMName) const
        {
            auto it = AllEFSMs.find(EFSMName);
            if (it == AllEFSMs.end()) {
                throw ESMCError((string)"Could not find EFSM named \"" + EFSMName + "\"");
            }
            return it->second->StateVarType;
        }

        const ExprTypeRef& LabelledTS::GetPrimedType(const ExprTypeRef& Type) const
        {
            auto it = TypeToPrimed.find(Type);
            if (it == TypeToPrimed.end()) {
                throw ESMCError((string)"Could not find primed type for type:\n" +
                                Type->ToString());
            }
            return it->second;
        }

        // Expression creation
        ExpT LabelledTS::MakeTrue()
        {
            return Mgr->MakeTrue();
        }

        ExpT LabelledTS::MakeFalse()
        {
            return Mgr->MakeFalse();
        }

        ExpT LabelledTS::MakeVar(const string& Name, const ExprTypeRef& Type)
        {
            if (boost::algorithm::starts_with(Name, "_")) {
                throw ESMCError((string)"Variable names beginning with an underscore " +
                                "are reserved for use by the ESMC system");
            }
            return Mgr->MakeVar(Name, Type);
        }

        ExpT LabelledTS::MakeBoundVar(i64 Idx, const ExprTypeRef& Type)
        {
            return Mgr->MakeBoundVar(Type, Idx);
        }

        ExpT LabelledTS::MakeVal(const string& ValString, const ExprTypeRef& Type)
        {
            if (ValString == "clear") {
                return Mgr->MakeVal(ValString, Type);
            } else {
                if (Type->Is<ExprSymmetricType>()) {
                    throw ESMCError((string)"Only constant of a symmetric type that " +
                                    "can be created is the \"clear\" constant");
                }
                if (!Type->Is<ExprScalarType>()) {
                    throw ESMCError((string)"Cannot create constant of type \"" +
                                    Type->ToString() + "\" only boolean, numeric and enumerated " +
                                    " type constants are allowed to be created");
                }
            }
            return Mgr->MakeVal(ValString, Type);
        }

        ExpT LabelledTS::MakeOp(i64 OpCode, const vector<ExpT>& Operands)
        {
            if (OpCode == LTSOps::OpEQ) {
                if (!Operands[0]->GetType()->Is<ExprScalarType>() ||
                    !Operands[1]->GetType()->Is<ExprScalarType>()) {
                    throw ESMCError((string)"Equality allowed only between scalar types");
                }
            }
            return Mgr->MakeExpr(OpCode, Operands);
        }

        ExpT LabelledTS::MakeOp(i64 OpCode, const ExpT& Operand1)
        {
            vector<ExpT> Operands = { Operand1 };
            return Mgr->MakeExpr(OpCode, Operands);
        }

        ExpT LabelledTS::MakeOp(i64 OpCode, const ExpT& Operand1, const ExpT& Operand2)
        {
            vector<ExpT> Operands = { Operand1, Operand2 };
            if (OpCode == LTSOps::OpEQ) {
                if (!Operand1->GetType()->Is<ExprScalarType>() ||
                    !Operand2->GetType()->Is<ExprScalarType>()) {
                    throw ESMCError((string)"Equality allowed only between scalar types");
                }
            }

            return Mgr->MakeExpr(OpCode, Operands);
        }

        ExpT LabelledTS::MakeOp(i64 OpCode, const ExpT& Operand1, const ExpT& Operand2,
                                const ExpT& Operand3)
        {
            vector<ExpT> Operands = { Operand1, Operand2, Operand3 };
            return Mgr->MakeExpr(OpCode, Operands);
        }

        ExpT LabelledTS::MakeOp(i64 OpCode, const ExpT& Operand1, const ExpT& Operand2,
                                const ExpT& Operand3, const ExpT& Operand4)
        {
            vector<ExpT> Operands = { Operand1, Operand2, Operand3, Operand4 };
            return Mgr->MakeExpr(OpCode, Operands);
        }

        ExpT LabelledTS::MakeExists(const vector<ExprTypeRef>& QVarTypes, const ExpT& Body)
        {
            return Mgr->MakeExists(QVarTypes, Body);
        }

        ExpT LabelledTS::MakeForAll(const vector<ExprTypeRef>& QVarTypes, const ExpT& Body)
        {
            return Mgr->MakeForAll(QVarTypes, Body);
        }

        i64 LabelledTS::MakeUF(const string& Name, const vector<ExprTypeRef>& Domain,
                               const ExprTypeRef& Range)
        {
            return Mgr->MakeUninterpretedFunction(Name, Domain, Range);
        }

        GeneralEFSM* LabelledTS::MakeGenEFSM(const string& Name, const vector<ExpT>& Params,
                                             const ExpT& Constraint, LTSFairnessType Fairness)
        {
            return MakeEFSM<GeneralEFSM>(Name, Params, Constraint, Fairness);
        }

        DetEFSM* LabelledTS::MakeDetEFSM(const string& Name, const vector<ExpT>& Params,
                                         const ExpT& Constraint, LTSFairnessType Fairness)
        {
            return MakeEFSM<DetEFSM>(Name, Params, Constraint, Fairness);
        }

        vector<EFSMBase*>
        LabelledTS::GetEFSMs(const function<bool(const EFSMBase*)>& MatchPred) const
        {
            vector<EFSMBase*> Retval;
            for (auto const& NameEFSM : AllEFSMs) {
                if (MatchPred(NameEFSM.second)) {
                    Retval.push_back(NameEFSM.second);
                }
            }
            return Retval;
        }

        ChannelEFSM* LabelledTS::MakeChannel(const string& Name, const vector<ExpT> &Params,
                                             const ExpT& Constraint, u32 Capacity,
                                             bool Lossy, bool Ordered, bool Duplicating,
                                             bool Blocking, LTSFairnessType Fairness)
        {
            AssertMsgsFrozen();
            AssertAutomataNotFrozen();

            if (AllEFSMs.find(Name) != AllEFSMs.end()) {
                throw ESMCError((string)"A machine named \"" + Name + "\" has already " +
                                "been created in the LTS");
            }

            auto Retval = new ChannelEFSM(this, Name, Params, Constraint, Capacity,
                                          Lossy, Ordered, Duplicating, Blocking, Fairness);
            AllEFSMs[Name] = Retval;
            ChannelEFSMs[Name] = Retval;
            return Retval;
        }

        void LabelledTS::InstantiateInitState(const InitStateRef& InitState)
        {
            auto const& Params = InitState->GetParams();
            auto const& Constraint = InitState->GetConstraint();
            auto const& Updates = InitState->GetUpdates();

            SymTab.Push();
            CheckParams(Params, Constraint, SymTab, Mgr, true);

            auto&& ParamInsts = InstantiateParams(Params, Constraint, Mgr);
            const u32 NumParams = Params.size();
            for (auto const& ParamInst : ParamInsts) {
                InitStateGenerators.push_back(vector<LTSAssignRef>());

                MgrT::SubstMapT SubstMap;
                for (u32 i = 0; i < NumParams; ++i) {
                    SubstMap[Params[i]] = ParamInst[i];
                }

                for (auto const& Update : Updates) {
                    if (Update->Is<LTSAssignSimple>()) {
                        CheckUpdates({ Update }, SymTab, Mgr, false, "");
                        auto SubstLHS = Mgr->Substitute(SubstMap, Update->GetLHS());
                        auto SubstRHS = Mgr->Substitute(SubstMap, Update->GetRHS());
                        auto NewAsgn = new LTSAssignSimple(SubstLHS, SubstRHS);
                        auto&& Vars = Mgr->Gather(SubstRHS, Detail::VarGatherer());
                        if (Vars.size() > 0) {
                            throw ESMCError((string)"Init state updates cannot refer " +
                                            "to variables");
                        }
                        InitStateGenerators.back().push_back(NewAsgn);
                    } else {
                        auto UpdateAsParam = Update->As<LTSAssignParam>();
                        auto const& AsgnParams = UpdateAsParam->GetParams();
                        auto const& AsgnConstraint = UpdateAsParam->GetConstraint();

                        CheckUpdates({ Update }, SymTab, Mgr, false, "");

                        auto&& AsgnParamInsts = InstantiateParams(AsgnParams,
                                                                  AsgnConstraint, Mgr);
                        const u32 NumAsgnParams = AsgnParams.size();

                        for (auto& AsgnParamInst : AsgnParamInsts) {
                            MgrT::SubstMapT LocalSubstMap = SubstMap;
                            for (u32 i = 0; i < NumAsgnParams; ++i) {
                                LocalSubstMap[AsgnParams[i]] = AsgnParamInst[i];
                            }

                            auto SubstLHS = Mgr->Substitute(LocalSubstMap, Update->GetLHS());
                            auto SubstRHS = Mgr->Substitute(LocalSubstMap, Update->GetRHS());
                            auto NewAsgn = new LTSAssignSimple(SubstLHS, SubstRHS);
                            auto&& Vars = Mgr->Gather(SubstRHS, Detail::VarGatherer());
                            if (Vars.size() > 0) {
                                throw ESMCError((string)"Init state updates cannot refer " +
                                                "to variables");
                            }
                            InitStateGenerators.back().push_back(NewAsgn);
                        }
                    }
                }

                // Add the initial states for the channels as well
                for (auto const& NameChanEFSM : ChannelEFSMs) {
                    auto ChanEFSM = NameChanEFSM.second;
                    InitStateGenerators.back().insert(InitStateGenerators.back().end(),
                                                      ChanEFSM->InitStateUpdates.begin(),
                                                      ChanEFSM->InitStateUpdates.end());
                }
            }

            vector<vector<LTSAssignRef>> FlattenedISGens;
            for (auto const& ISGen : InitStateGenerators) {
                auto&& FlattenedISGen = ExpandUpdates(ISGen);
                FlattenedISGens.push_back(FlattenedISGen);
            }

            InitStateGenerators = FlattenedISGens;

            SymTab.Pop();
        }

        void LabelledTS::AddInitStates(const vector<InitStateRef>& InitStates)
        {
            AssertAutomataFrozen();
            AssertNotFrozen();

            for (auto const& InitState : InitStates) {
                InstantiateInitState(InitState);
            }
        }

        void LabelledTS::AddInvariant(const ExpT& Invariant)
        {
            AssertAutomataFrozen();
            AssertNotFrozen();
            auto ElimExp = Mgr->UnrollQuantifiers(Invariant, false);
            CheckExpr(ElimExp, SymTab, Mgr);
            InvariantExp = Mgr->MakeExpr(LTSOps::OpAND, InvariantExp, ElimExp);
            InvariantExp = Mgr->Simplify(InvariantExp);
        }

        const unordered_map<i64, set<ExpT>>& LabelledTS::GetConstraintsByOp() const
        {
            return ConstraintsByOp;
        }

        const unordered_map<i64, pair<ExpT, ExpT>>&
        LabelledTS::GetUpdateOpToUpdateLValue() const
        {
            return UpdateOpToUpdateLValue;
        }

    } /* end namespace LTS */
} /* end namespace ESMC */

//
// LabelledTS.cpp ends here
