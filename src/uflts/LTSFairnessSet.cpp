// LTSFairnessSet.cpp ---
//
// Filename: LTSFairnessSet.cpp
// Author: Abhishek Udupa
// Created: Fri Aug  8 17:18:20 2014 (-0400)
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

#include "LTSFairnessSet.hpp"
#include "LTSTransitions.hpp"
#include "LTSEFSMBase.hpp"

namespace ESMC {
    namespace LTS {

        UIDGenerator LTSFairnessObject::FairnessUIDGenerator;

        LTSFairnessObject::LTSFairnessObject(LTSFairnessSet* FairnessSet,
                                             const string& Name, EFSMBase* TheEFSM,
                                             const vector<ExpT>& IndexInst,
                                             FairSetFairnessType Fairness, u32 InstanceID)
            : Name(Name), TheEFSM(TheEFSM), IndexInst(IndexInst),
              InstanceID(InstanceID), Fairness(Fairness),
              FairnessID(FairnessUIDGenerator.GetUID()),
              FairnessSetID(FairnessSet->GetFairnessSetID()),
              FairnessSet(FairnessSet)
        {
            // Instantiate the name with the index instance
            for (auto const& IndexExp : IndexInst) {
                this->Name += ((string)"[" + IndexExp->ToString() + "]");
            }
        }

        LTSFairnessObject::~LTSFairnessObject()
        {
            // Nothing here
        }

        const string& LTSFairnessObject::GetName() const
        {
            return Name;
        }

        EFSMBase* LTSFairnessObject::GetEFSM() const
        {
            return TheEFSM;
        }

        const vector<ExpT>& LTSFairnessObject::GetIndexInst() const
        {
            return IndexInst;
        }

        FairSetFairnessType LTSFairnessObject::GetFairnessType() const
        {
            return Fairness;
        }

        u32 LTSFairnessObject::GetFairnessID() const
        {
            return FairnessID;
        }

        u32 LTSFairnessObject::GetInstanceID() const
        {
            return InstanceID;
        }

        u32 LTSFairnessObject::GetFairnessSetID() const
        {
            return FairnessSetID;
        }

        void LTSFairnessObject::ResetFairnessUID()
        {
            FairnessUIDGenerator.Reset();
        }

        LTSFairnessSet* LTSFairnessObject::GetFairnessSet() const
        {
            return FairnessSet;
        }


        UIDGenerator LTSFairnessSet::FairnessSetUIDGenerator;

        LTSFairnessSet::LTSFairnessSet(LTSProcessFairnessGroup* PFGroup,
                                       const string& Name, EFSMBase* TheEFSM,
                                       const vector<vector<ExpT>>& AllInstances,
                                       FairSetFairnessType Fairness)
            : Name(Name), TheEFSM(TheEFSM), AllInstances(AllInstances),
              Fairness(Fairness), FairnessSetID(FairnessSetUIDGenerator.GetUID()),
              PFGroup(PFGroup)
        {
            // Instantiate a fairness object for every instance
            NumInstances = AllInstances.size();

            for (u32 i = 0; i < NumInstances; ++i) {
                auto CurFairObj = new LTSFairnessObject(this, Name, TheEFSM,
                                                        AllInstances[i], Fairness, i);
                if (i == 0) {
                    FairnessIDLow = CurFairObj->GetFairnessID();
                }
                if (i == NumInstances - 1) {
                    FairnessIDHigh = CurFairObj->GetFairnessID();
                }
                FairnessObjects[AllInstances[i]] = CurFairObj;
            }
        }

        LTSFairnessSet::~LTSFairnessSet()
        {
            // Nothing here
        }

        u32 LTSFairnessSet::GetFairnessSetID() const
        {
            return FairnessSetID;
        }

        const string& LTSFairnessSet::GetName() const
        {
            return Name;
        }

        EFSMBase* LTSFairnessSet::GetEFSM() const
        {
            return TheEFSM;
        }

        const vector<vector<ExpT>>& LTSFairnessSet::GetAllInstances() const
        {
            return AllInstances;
        }

        const map<vector<ExpT>, LTSFairObjRef>& LTSFairnessSet::GetFairnessObjs() const
        {
            return FairnessObjects;
        }

        u32 LTSFairnessSet::GetFairnessIDLow() const
        {
            return FairnessIDLow;
        }

        u32 LTSFairnessSet::GetFairnessIDHigh() const
        {
            return FairnessIDHigh;
        }

        u32 LTSFairnessSet::GetNumInstances() const
        {
            return NumInstances;
        }

        FairSetFairnessType LTSFairnessSet::GetFairnessType() const
        {
            return Fairness;
        }

        const LTSFairObjRef&
        LTSFairnessSet::GetFairnessObj(const vector<ExpT>& Instance) const
        {
            auto it = FairnessObjects.find(Instance);
            if (it == FairnessObjects.end()) {
                return LTSFairObjRef::NullPtr;
            }

            return it->second;
        }

        LTSProcessFairnessGroup* LTSFairnessSet::GetPFGroup() const
        {
            return PFGroup;
        }


        LTSProcessFairnessGroup::LTSProcessFairnessGroup(EFSMBase* TheEFSM)
            : TheEFSM(TheEFSM)
        {
            // Nothing here
        }

        LTSProcessFairnessGroup::~LTSProcessFairnessGroup()
        {
            // Nothing here
        }

        void LTSProcessFairnessGroup::AddFairnessSet(const string& Name,
                                                     FairSetFairnessType FairnessType) const
        {
            if (FairnessSets.find(Name) != FairnessSets.end()) {
                throw ESMCError((string)"Fairness set named \"" + Name + "\" already " +
                                "declared in EFSM named \"" + TheEFSM->GetName() + "\"");
            }
            FairnessSets[Name] = new LTSFairnessSet(const_cast<LTSProcessFairnessGroup*>(this),
                                                    Name, TheEFSM, TheEFSM->ParamInsts,
                                                    FairnessType);
        }

        EFSMBase* LTSProcessFairnessGroup::GetEFSM() const
        {
            return TheEFSM;
        }

        const map<string, LTSFairSetRef>& LTSProcessFairnessGroup::GetFairnessSets() const
        {
            return FairnessSets;
        }

        const LTSFairSetRef&
        LTSProcessFairnessGroup::GetFairnessSet(const string &Name) const
        {
            auto it = FairnessSets.find(Name);
            if (it == FairnessSets.end()) {
                throw ESMCError((string)"Fairness set named \"" + Name + "\"" +
                                "not declared in EFSM \"" + TheEFSM->GetName() + "\"");
            }

            return it->second;
        }

        const LTSFairObjRef&
        LTSProcessFairnessGroup::GetFairnessObj(const string &Name,
                                                const vector<ExpT> &Instance) const
        {
            auto const& FSet = GetFairnessSet(Name);
            return FSet->GetFairnessObj(Instance);
        }

    } /* end namespace LTS */
} /* end namespace ESMC */

//
// LTSFairnessSet.cpp ends here
