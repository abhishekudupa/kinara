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
#include "LTSUtils.hpp"

namespace ESMC {
    namespace LTS {

        // Implementation of LTSFairnessObject

        UIDGenerator LTSFairnessObject::InstanceUIDGenerator;

        LTSFairnessObject::LTSFairnessObject(const string& BaseName,
                                             const vector<ExpT>& InstanceParams,
                                             LTSFairnessSet* FairnessSet,
                                             u32 InstanceNumber)
            : Name(BaseName), InstanceParams(InstanceParams),
              InstanceID(InstanceUIDGenerator.GetUID()),
              InstanceNumber(InstanceNumber), FairnessSet(FairnessSet)
        {
            for (auto const& InstParam : InstanceParams) {
                Name += ((string)"[" +
                         InstParam->As<ConstExpression>()->GetConstValue() + "]");
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

        const vector<ExpT>& LTSFairnessObject::GetInstanceParams() const
        {
            return InstanceParams;
        }

        u32 LTSFairnessObject::GetInstanceID() const
        {
            return InstanceID;
        }

        u32 LTSFairnessObject::GetInstanceNumber() const
        {
            return InstanceNumber;
        }

        LTSFairnessSet* LTSFairnessObject::GetFairnessSet() const
        {
            return FairnessSet;
        }

        const string& LTSFairnessObject::GetBaseName() const
        {
            return FairnessSet->GetName();
        }

        const vector<ExpT>& LTSFairnessObject::GetFairnessSetParams() const
        {
            return FairnessSet->GetParameters();
        }

        const ExpT& LTSFairnessObject::GetFairnessSetConstraint() const
        {
            return FairnessSet->GetConstraint();
        }

        const vector<vector<ExpT>>& LTSFairnessObject::GetSiblingInstances() const
        {
            return FairnessSet->GetAllInstances();
        }

        u32 LTSFairnessObject::GetFairnessSetNumInstances() const
        {
            return FairnessSet->GetNumInstances();
        }

        FairSetFairnessType LTSFairnessObject::GetFairnessType() const
        {
            return FairnessSet->GetFairnessType();
        }

        EFSMBase* LTSFairnessObject::GetEFSM() const
        {
            return FairnessSet->GetEFSM();
        }

        const LTSProcessFairnessGroup* LTSFairnessObject::GetFairnessGroup() const
        {
            return FairnessSet->GetFairnessGroup();
        }

        string LTSFairnessObject::ToString(u32 Verbosity) const
        {
            return Name;
        }


        // Implementation of LTSFairnessSet
        UIDGenerator LTSFairnessSet::ClassUIDGenerator;

        LTSFairnessSet::LTSFairnessSet(const string& Name,
                                       const vector<ExpT>& Parameters,
                                       const ExpT& Constraint,
                                       FairSetFairnessType FairnessType,
                                       const LTSProcessFairnessGroup* FairnessGroup)
            : Name(Name), Parameters(Parameters), Constraint(Constraint),
              AllInstances(InstantiateParams(Parameters, Constraint,
                                             Constraint->GetMgr())),
              NumInstances(AllInstances.size()), ClassID(ClassUIDGenerator.GetUID()),
              FairnessType(FairnessType), FairnessGroup(FairnessGroup)

        {
            // create the fairness objects
            for (u32 i = 0; i < NumInstances; ++i) {
                auto const& Inst = AllInstances[i];
                FairnessObjects[Inst] = new LTSFairnessObject(Name, Inst, this, i);
            }
            InstanceIDLow = FairnessObjects[AllInstances[0]]->GetInstanceID();
            InstanceIDHigh = FairnessObjects[AllInstances.back()]->GetInstanceID();
        }

        LTSFairnessSet::~LTSFairnessSet()
        {
            // Nothing here
        }

        const string& LTSFairnessSet::GetName() const
        {
            return Name;
        }

        const vector<ExpT>& LTSFairnessSet::GetParameters() const
        {
            return Parameters;
        }

        const ExpT& LTSFairnessSet::GetConstraint() const
        {
            return Constraint;
        }

        const vector<vector<ExpT>>& LTSFairnessSet::GetAllInstances() const
        {
            return AllInstances;
        }

        u32 LTSFairnessSet::GetNumInstances() const
        {
            return NumInstances;
        }

        u32 LTSFairnessSet::GetClassID() const
        {
            return ClassID;
        }

        const LTSFairObjRef& LTSFairnessSet::GetFairnessObj(const vector<ExpT>& Instance) const
        {
            auto it = FairnessObjects.find(Instance);
            if (it == FairnessObjects.end()) {
                ostringstream sstr;
                sstr << "Could not find fairness object for instance:" << endl;
                sstr << "<";
                for (auto const& Exp : Instance) {
                    sstr << " " << Exp;
                }
                sstr << " >" << endl;
                sstr << "In fairness set " << Name << endl;
                sstr << "At: " << __FUNCTION__ << ", " << __FILE__ << ":" << __LINE__;
                throw InternalError(sstr.str());
            } else {
                return it->second;
            }
        }

        const LTSFairObjRef& LTSFairnessSet::GetFairnessObj(u32 InstanceNumber) const
        {
            if (InstanceNumber >= NumInstances) {
                throw InternalError((string)"Fairness Set \"" + Name + "\" has only " +
                                    to_string(NumInstances) + " instances, but a request " +
                                    "was made for instance number " + to_string(InstanceNumber) +
                                    "\nAt: " + __FUNCTION__ + ", " + __FILE__ + ":" +
                                    to_string(__LINE__));
            }
            return GetFairnessObj(AllInstances[InstanceNumber]);
        }

        FairSetFairnessType LTSFairnessSet::GetFairnessType() const
        {
            return FairnessType;
        }

        const LTSProcessFairnessGroup* LTSFairnessSet::GetFairnessGroup() const
        {
            return FairnessGroup;
        }

        u32 LTSFairnessSet::GetInstanceIDLow() const
        {
            return InstanceIDLow;
        }

        u32 LTSFairnessSet::GetInstanceIDHigh() const
        {
            return InstanceIDHigh;
        }

        string LTSFairnessSet::ToString(u32 Verbosity) const
        {
            ostringstream sstr;
            sstr << Name;
            for (auto const& Param : Parameters) {
                auto ParamAsVar = Param->As<VarExpression>();
                sstr << "[" << ParamAsVar->GetVarName() << ":"
                     << ParamAsVar->GetVarType()->ToString() << "]";
            }
            return sstr.str();
        }

        EFSMBase* LTSFairnessSet::GetEFSM() const
        {
            return FairnessGroup->GetEFSM();
        }


        // Implementation of LTSProcessFairnessGroup
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
                                                     FairSetFairnessType FairnessType,
                                                     const vector<ExpT>& Params,
                                                     const ExpT& Constraint) const
        {
            auto it = FairnessSets.find(Name);
            if (it != FairnessSets.end()) {
                throw InternalError((string)"Fairness set named \"" + Name + "\" already " +
                                    "exists in fairness group for EFSM \"" + TheEFSM->GetName() +
                                    "\"\nAt: " + __FUNCTION__ + ", " + __FILE__ + ":" +
                                    to_string(__LINE__));
            }
            FairnessSets[Name] = new LTSFairnessSet(Name, Params, Constraint,
                                                    FairnessType, this);
        }

        EFSMBase* LTSProcessFairnessGroup::GetEFSM() const
        {
            return TheEFSM;
        }

        const map<string, LTSFairSetRef>& LTSProcessFairnessGroup::GetAllFairnessSets() const
        {
            return FairnessSets;
        }

        const LTSFairSetRef& LTSProcessFairnessGroup::GetFairnessSet(const string& Name) const
        {
            auto it = FairnessSets.find(Name);
            if (it == FairnessSets.end()) {
                return LTSFairSetRef::NullPtr;
            }
            return it->second;
        }

        const LTSFairObjRef&
        LTSProcessFairnessGroup::GetFairnessObj(const string& Name,
                                                const vector<ExpT>& Instance) const
        {
            auto const& FairSet = GetFairnessSet(Name);
            return FairSet->GetFairnessObj(Instance);
        }

        const LTSFairObjRef& LTSProcessFairnessGroup::GetFairnessObj(const string& Name,
                                                                     u32 InstanceNumber) const
        {
            auto const& FairSet = GetFairnessSet(Name);
            return FairSet->GetFairnessObj(InstanceNumber);
        }

        string LTSProcessFairnessGroup::ToString(u32 Verbosity) const
        {
            return (string)"Process Fairness Group for EFSM \"" + TheEFSM->GetName() + "\"";
        }
    } /* end namespace LTS */
} /* end namespace ESMC */

//
// LTSFairnessSet.cpp ends here
