// LTSFairnessSet.hpp ---
//
// Filename: LTSFairnessSet.hpp
// Author: Abhishek Udupa
// Created: Fri Aug  8 17:09:14 2014 (-0400)
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

#if !defined ESMC_LTS_FAIRNESS_SET_HPP_
#define ESMC_LTS_FAIRNESS_SET_HPP_

#include <vector>
#include <map>

#include "../containers/RefCountable.hpp"
#include "../utils/UIDGenerator.hpp"

#include "LTSDecls.hpp"

namespace ESMC {
    namespace LTS {

        class LTSFairnessObject : public RefCountable, public Stringifiable
        {
        private:
            string Name;
            vector<ExpT> InstanceParams;
            u32 InstanceID;
            u32 InstanceNumber;
            LTSFairnessSet* FairnessSet;

            static UIDGenerator InstanceUIDGenerator;

        public:
            LTSFairnessObject(const string& BaseName,
                              const vector<ExpT>& InstanceParams,
                              LTSFairnessSet* FairnessSet,
                              u32 InstanceNumber);
            virtual ~LTSFairnessObject();

            const string& GetName() const;
            const vector<ExpT>& GetInstanceParams() const;
            u32 GetInstanceID() const;
            u32 GetInstanceNumber() const;
            LTSFairnessSet* GetFairnessSet() const;

            // Dispatch to FairnessSet
            const string& GetBaseName() const;
            const vector<ExpT>& GetFairnessSetParams() const;
            const ExpT& GetFairnessSetConstraint() const;
            const vector<vector<ExpT>>& GetSiblingInstances() const;
            u32 GetFairnessSetNumInstances() const;
            const map<vector<ExpT>, LTSFairObjRef>& GetSiblings() const;
            FairSetFairnessType GetFairnessType() const;
            const LTSProcessFairnessGroup* GetFairnessGroup() const;
            EFSMBase* GetEFSM() const;

            virtual string ToString(u32 Verbosity = 0) const override;
        };

        class LTSFairnessSet : public RefCountable, public Stringifiable
        {
        private:
            string Name;
            vector<ExpT> Parameters;
            ExpT Constraint;
            vector<vector<ExpT>> AllInstances;
            u32 NumInstances;
            map<vector<ExpT>, LTSFairObjRef> FairnessObjects;
            u32 ClassID;
            FairSetFairnessType FairnessType;
            const LTSProcessFairnessGroup* FairnessGroup;
            u32 InstanceIDLow;
            u32 InstanceIDHigh;

            static UIDGenerator ClassUIDGenerator;

        public:
            LTSFairnessSet(const string& Name, const vector<ExpT>& Parameters,
                           const ExpT& Constraint,
                           FairSetFairnessType FairnessType,
                           const LTSProcessFairnessGroup* FairnessGroup);
            virtual ~LTSFairnessSet();

            const string& GetName() const;
            const vector<ExpT>& GetParameters() const;
            const ExpT& GetConstraint() const;
            const vector<vector<ExpT>>& GetAllInstances() const;
            u32 GetNumInstances() const;
            u32 GetClassID() const;
            const map<vector<ExpT>, LTSFairObjRef>& GetAllFairnessObjs() const;
            const LTSFairObjRef& GetFairnessObj(const vector<ExpT>& Instance) const;
            const LTSFairObjRef& GetFairnessObj(u32 InstanceNumber) const;
            FairSetFairnessType GetFairnessType() const;
            const LTSProcessFairnessGroup* GetFairnessGroup() const;

            u32 GetInstanceIDLow() const;
            u32 GetInstanceIDHigh() const;

            virtual string ToString(u32 Verbosity = 0) const override;

            // Dispatch to fairness group
            EFSMBase* GetEFSM() const;
        };

        class LTSProcessFairnessGroup : public RefCountable, public Stringifiable
        {
        private:
            EFSMBase* TheEFSM;
            mutable map<string, LTSFairSetRef> FairnessSets;

        public:
            LTSProcessFairnessGroup(EFSMBase* TheEFSM);
            virtual ~LTSProcessFairnessGroup();
            void AddFairnessSet(const string& Name,
                                FairSetFairnessType FairnessType,
                                const vector<ExpT>& Params,
                                const ExpT& Constraint) const;
            EFSMBase* GetEFSM() const;
            const map<string, LTSFairSetRef>& GetAllFairnessSets() const;
            const LTSFairSetRef& GetFairnessSet(const string& Name) const;
            const LTSFairObjRef& GetFairnessObj(const string& FairnessSetName,
                                                const vector<ExpT>& Instance) const;
            const LTSFairObjRef& GetFairnessObj(const string& FairnessSetName,
                                                u32 InstanceNumber) const;

            virtual string ToString(u32 Verbosity = 0) const override;
        };

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LTS_FAIRNESS_SET_HPP_ */

//
// LTSFairnessSet.hpp ends here
