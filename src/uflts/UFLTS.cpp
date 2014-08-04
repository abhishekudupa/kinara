// UFLTS.cpp --- 
// 
// Filename: UFLTS.cpp
// Author: Abhishek Udupa
// Created: Fri Aug  1 14:25:57 2014 (-0400)
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

#include <algorithm>

#include "UFLTS.hpp"
#include "LTSUtils.hpp"
#include "UFEFSM.hpp"
#include "ChannelEFSM.hpp"
#include "FrozenEFSM.hpp"
#include "CompiledLTS.hpp"

namespace ESMC {
    namespace LTS {

        const string MTypeFieldName = (string)"__MTYPE__";
        const u32 MaxMessageTypes = 65000;
        const string UnifiedMTypeName = (string)"__unified_mtype__";

        UFLTS::UFLTS()
            : Mgr(new MgrType()), Frozen(false), MsgsFrozen(false),
              EFSMsFrozen(false)
        {
            MessageIDType = Mgr->MakeType<Exprs::ExprRangeType>(0, MaxMessageTypes);
        }

        UFLTS::~UFLTS()
        {
            delete Mgr;
        }

        MgrType* UFLTS::GetMgr() const
        {
            return Mgr;
        }

        const ExprTypeRef& UFLTS::MakeSymmetricType(const string& Name,
                                                    u32 NumElements)
        {
            auto it = SymmetricTypes.find(Name);
            if (it != SymmetricTypes.end()) {
                throw ESMCError((string)"Type named \"" + Name + "\" has already been created");
            }
            SymmetricTypes[Name] = Mgr->MakeType<Exprs::ExprSymmetricType>(Name, NumElements);
            return SymmetricTypes[Name];
        }

        const ExprTypeRef& UFLTS::MakeMessageType(const string& Name,
                                                  const vector<pair<string, ExprTypeRef>>& Fields,
                                                  bool IncludePrimed)
        {
            if (MsgsFrozen) {
                throw ESMCError((string)"Cannot create messages after a call to " + 
                                "UFLTS::FreezeMsgs()");
            }
            for (auto const& Char : Name) {
                if (Char == '[' || Char == ']') {
                    throw ESMCError((string)"Illegal characters in Message Type Name \"" + 
                                    Name + "\". Names cannot contain '[' or ']' " + 
                                    "characters.");
                }
            }

            for (auto const& Field : Fields) {
                if (Field.first == MTypeFieldName) {
                    throw ESMCError((string)"The field name \"" + MTypeFieldName + "\" is " + 
                                    "reserved in messages");
                }
            }

            auto MTypePair = make_pair(MTypeFieldName, MessageIDType);
            auto ActFields = Fields;
            ActFields.insert(ActFields.begin(), MTypePair);

            auto Retval = Mgr->MakeType<Exprs::ExprRecordType>(Name, ActFields);
            MTypes[Name] = Retval;
            auto TypeUID = MTypeUIDGen.GetUID();
            if (TypeUID > MaxMessageTypes) {
                throw ESMCError((string)"Exceeded maximum number of message types");
            }
            MTypeIDs[Name] = TypeUID;

            if (IncludePrimed) {
                auto Type = Mgr->MakeType<Exprs::ExprRecordType>(Name + "'", ActFields);
                MTypes[Name + "'"] = Type;
                auto TypeUID = MTypeUIDGen.GetUID();
                if (TypeUID > MaxMessageTypes) {
                    throw ESMCError((string)"Exceeded maximum number of message types");
                }

                MTypeIDs[Name] = TypeUID;
                TypeToPrimed[Retval] = Type;
            }
            
            return MTypes[Name];
        }

        const ExprTypeRef& UFLTS::MakeMessageType(const string& Name,
                                                  const vector<pair<string, ExprTypeRef>>& Fields,
                                                  const vector<ExpT>& Params,
                                                  const ExpT& Constraint,
                                                  bool IncludePrimed)
        {
            if (MsgsFrozen) {
                throw ESMCError((string)"Cannot create messages after a call to " + 
                                "UFLTS::FreezeMsgs()");
            }

            if (PMTypes.find(Name) != PMTypes.end()) {
                throw ESMCError((string)"Message type named \"" + Name + "\" already declared");
            }
            const u32 NumParams = Params.size();
            vector<ExprTypeRef> ParamTypes(NumParams);

            for (u32 i = 0; i < NumParams; ++i) {
                ParamTypes[i] = Params[i]->GetType();
            }

            for (auto const& Field : Fields) {
                if (Field.first == MTypeFieldName) {
                    throw ESMCError((string)"The field name \"" + MTypeFieldName + "\" is " + 
                                    "reserved in messages");
                }
            }

            auto MTypePair = make_pair(MTypeFieldName, MessageIDType);
            auto ActFields = Fields;
            ActFields.insert(ActFields.begin(), MTypePair);

            auto RecType = Mgr->MakeType<Exprs::ExprRecordType>(Name, ActFields);
            ExprTypeRef PrimedRecType = nullptr;
            if (IncludePrimed) {
                PrimedRecType = Mgr->MakeType<Exprs::ExprRecordType>(Name + "'", ActFields);
            }

            auto PType = RecType;
            auto PrimedPType = PrimedRecType;
            for (auto it = ParamTypes.rbegin(); it != ParamTypes.rend(); ++it) {
                PType = Mgr->MakeType<Exprs::ExprParametricType>(PType, *it);
                if (IncludePrimed) {
                    PrimedPType = Mgr->MakeType<Exprs::ExprParametricType>(PType, *it);
                }
            }
            
            PMTypes[Name] = PType;
            if (IncludePrimed) {
                PMTypes[Name + "'"] = PrimedPType;
            }

            // Instantiate the parametric type
            auto&& ParamVec = InstantiateParams(Params, Constraint, Mgr);
            for (auto const& ParamVals : ParamVec) {
                auto InstType = Mgr->InstantiateType(PType, ParamVals);
                auto const& TypeName = InstType->SAs<Exprs::ExprRecordType>()->GetName();
                MTypes[TypeName] = InstType;
                auto TypeUID = MTypeUIDGen.GetUID();
                if (TypeUID > MaxMessageTypes) {
                    throw ESMCError((string)"Exceeded maximum number of message types");
                }
                MTypeIDs[TypeName] = TypeUID;

                if (IncludePrimed) {
                    auto PrimedInstType = Mgr->InstantiateType(PrimedPType, ParamVals);
                    auto const& PrimedTypeName = InstType->SAs<Exprs::ExprRecordType>()->GetName();
                    MTypes[PrimedTypeName] = PrimedInstType;
                    auto TypeUID = MTypeUIDGen.GetUID();
                    if (TypeUID > MaxMessageTypes) {
                        throw ESMCError((string)"Exceeded maximum number of message types");
                    }
                    
                    TypeToPrimed[InstType] = PrimedInstType;
                    MTypeIDs[PrimedTypeName] = TypeUID;
                }
            }

            return PMTypes[Name];
        }

        const ExprTypeRef& UFLTS::GetMessageType(const string& Name) const
        {
            auto it = MTypes.find(Name);
            if (it == MTypes.end()) {
                auto it = PMTypes.find(Name);
                if (it == PMTypes.end()) {
                    return ExprTypeRef::NullPtr;
                } else {
                    return it->second;
                }
            } else {
                return it->second;
            }
        }

        const ExprTypeRef& UFLTS::GetPrimedType(const ExprTypeRef& Type) const
        {
            auto it = TypeToPrimed.find(Type);
            if (it == TypeToPrimed.end()) {
                throw ESMCError((string)"Type " + Type->ToString() + " does not " + 
                                "have a primed type associated with it!");
            } else {
                return it->second;
            }
        }

        i32 UFLTS::GetTypeIDForMessageType(const string& Name) const
        {
            auto it = MTypeIDs.find(Name);
            if (it == MTypeIDs.end()) {
                return -1;
            } else {
                return it->second;
            }
        }

        bool UFLTS::CheckMessageType(const ExprTypeRef& MType)
        {
            auto TypeAsRec = MType->As<Exprs::ExprRecordType>();
            if (TypeAsRec == nullptr) {
                return false;
            }
            if (MTypes.find(TypeAsRec->GetName()) == MTypes.end()) {
                return false;
            }
            return true;
        }

        void UFLTS::FreezeMessages()
        {
            set<ExprTypeRef> UnionTypes;
            for (auto const& MType : MTypes) {
                UnionTypes.insert(MType.second);
            }
            UnifiedMType = Mgr->MakeType<Exprs::ExprUnionType>(UnifiedMTypeName, UnionTypes);
            MessageSize = UnifiedMType->GetByteSize();
            MsgsFrozen = true;
        }

        u32 UFLTS::GetMessageSize() const
        {
            if (!MsgsFrozen) {
                throw ESMCError((string)"UFLTS::GetMessageSize() can only be called after " + 
                                "freezing the message declarations");
            }
            return MessageSize;
        }

        const ExprTypeRef& UFLTS::GetMessageIDType() const
        {
            return MessageIDType;
        }

        const ExprTypeRef& UFLTS::GetUnifiedMType() const
        {
            return UnifiedMType;
        }

        UFEFSM* UFLTS::MakeEFSM(const string& Name,
                                const vector<ExpT>& Params,
                                const ExpT& Constraint)
        {
            if (!MsgsFrozen) {
                throw ESMCError((string)"UFLTS::MakeEFSM() can only be called after " + 
                                "freezing the message declarations");
            }

            if (EFSMsFrozen) {
                throw ESMCError((string)"UFLTS::MakeEFSM() cannot be called after " + 
                                "freezing the EFSMs");
            }

            for (auto const& EFSM : EFSMs) {
                if (EFSM->GetName() == Name) {
                    throw ESMCError((string)"EFSM with name \"" + Name + "\" already created");
                }
            }

            for (auto const& Channel : Channels) {
                if (Channel->GetName() == Name) {
                    throw ESMCError((string)"Channel with name \"" + Name + "\" already created");
                }
            }

            auto Retval = new UFEFSM(this, Name, Params, Constraint);
            EFSMs.push_back(Retval);
            return Retval;
        }

        ChannelEFSM* UFLTS::MakeChannel(const string& Name, 
                                        const vector<ExpT>& Params, 
                                        const ExpT& Constraint,
                                        u32 Capacity, bool Ordered, bool Lossy,
                                        bool Duplicating, bool Blocking)
        {
            if (!MsgsFrozen) {
                throw ESMCError((string)"UFLTS::MakeChannel() can only be called after " + 
                                "freezing the message declarations");
            }
            
            if (EFSMsFrozen) {
                throw ESMCError((string)"UFLTS::MakeChannel() cannot be called after " + 
                                "freezing the EFSMs");
            }

            for (auto const& EFSM : EFSMs) {
                if (EFSM->GetName() == Name) {
                    throw ESMCError((string)"EFSM with name \"" + Name + "\" already created");
                }
            }

            for (auto const& Channel : Channels) {
                if (Channel->GetName() == Name) {
                    throw ESMCError((string)"Channel with name \"" + Name + "\" already created");
                }
            }
            auto Channel = new ChannelEFSM(this, Name, Params, Constraint, 
                                           Capacity, Ordered, Lossy,
                                           Duplicating, Blocking);
            Channels.push_back(Channel);
            return Channel;
        }

        template <typename T>
        static inline bool IsInterEmpty(const set<T>& Set1, const set<T>& Set2)
        {
            auto MaxSize = Set1.size() > Set2.size() ? Set1.size() : Set2.size();

            vector<T> InterSet(MaxSize);
            set_intersection(Set1.begin(), Set1.end(), Set2.begin(), Set2.end(), InterSet.begin());
            return (InterSet.size() == 0);
        }

        void UFLTS::FreezeEFSMs()
        {
            if (!MsgsFrozen) {
                throw ESMCError((string)"Cannot freeze EFSMs before freezing messages");
            }
            
            if (EFSMsFrozen) {
                return;
            }

            EFSMsFrozen = true;
            for (auto EFSM : EFSMs) {
                EFSM->FreezeAll();
                auto const& CurEFSMs = EFSM->GetFrozenEFSMs();
                FrozenEFSMs.insert(FrozenEFSMs.end(), CurEFSMs.begin(), CurEFSMs.end());
            }
            for (auto Channel : Channels) {
                Channel->FreezeAll();
                auto const& CurChans = Channel->GetFrozenEFSMs();
                FrozenChannels.insert(FrozenChannels.end(), CurChans.begin(),
                                      CurChans.end());
            }

            
            // Check for closedness
            set<ExprTypeRef> Inputs;
            set<ExprTypeRef> Outputs;

            map<ExprTypeRef, FrozenEFSM*> Producers;
            map<ExprTypeRef, vector<FrozenEFSM*>> Consumers;

            for (auto EFSM : FrozenEFSMs) {
                auto const& CurInputs = EFSM->GetInputs();
                auto const& CurOutputs = EFSM->GetOutputs();
                if (!IsInterEmpty(CurInputs, CurOutputs)) {
                    throw ESMCError((string)"Inputs and outputs of EFSM \"" + 
                                    EFSM->GetBaseName() + "\" are not disjoint");
                }
                
                // The outputs cannot have any intersection with
                // the outputs already seen
                if (!IsInterEmpty(CurOutputs, Outputs)) {
                    throw ESMCError((string)"Outputs of EFSMs are not disjoint");
                }

                Outputs.insert(CurOutputs.begin(), CurOutputs.end());
                Inputs.insert(CurInputs.begin(), CurInputs.end());

                for (auto const& Output : Outputs) {
                    Producers[Output] = EFSM;
                }
                for (auto const& Input : Inputs) {
                    auto it = Consumers.find(Input);
                    if (it == Consumers.end()) {
                        Consumers[Input] = vector<FrozenEFSM*>();
                    }
                    Consumers[Input].push_back(EFSM);
                }
            }


            for (auto EFSM : FrozenChannels) {
                auto const& CurInputs = EFSM->GetInputs();
                auto const& CurOutputs = EFSM->GetOutputs();
                if (!IsInterEmpty(CurInputs, CurOutputs)) {
                    throw ESMCError((string)"Inputs and outputs of Channel \"" + 
                                    EFSM->GetBaseName() + "\" are not disjoint");
                }
                
                // The outputs cannot have any intersection with
                // the outputs already seen
                if (!IsInterEmpty(CurOutputs, Outputs)) {
                    throw ESMCError((string)"Outputs of EFSMs are not disjoint");
                }

                Outputs.insert(CurOutputs.begin(), CurOutputs.end());
                Inputs.insert(CurInputs.begin(), CurInputs.end());

                for (auto const& Output : Outputs) {
                    Producers[Output] = EFSM;
                }
                for (auto const& Input : Inputs) {
                    auto it = Consumers.find(Input);
                    if (it == Consumers.end()) {
                        Consumers[Input] = vector<FrozenEFSM*>();
                    }
                    Consumers[Input].push_back(EFSM);
                }
            }

            if (!includes(Inputs.begin(), Inputs.end(),
                          Outputs.begin(), Outputs.end()) ||
                !includes(Outputs.begin(), Outputs.end(),
                          Inputs.begin(), Inputs.begin())) {
                throw ESMCError((string)"EFSMs are not closed");
            }

            // 

            // Compute the product
            for (auto const& MsgType : Inputs) {
                vector<vector<TransitionT>> TransVec;
                TransVec.push_back(Producers[MsgType]->GetTransitionsOnMsg(MsgType));

                for (auto const& Listener : Consumers[MsgType]) {
                    TransVec.push_back(Listener->GetTransitionsOnMsg(MsgType));
                }

                // Compute the cross product
                auto&& TransCP = CrossProduct<TransitionT>(TransVec.begin(),
                                                           TransVec.end());

                // TODO: 
                // Make a guarded command for each element of the 
                // cross product
            }

        }

    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// UFLTS.cpp ends here
