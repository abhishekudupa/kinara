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

#include "../containers/RefCountable.hpp"
#include "../utils/UIDGenerator.hpp"

#include "LTSTypes.hpp"

namespace ESMC {
    namespace LTS {

        // One fairness object, that can consist 
        // of multiple transitions, all parameterized
        // by the same index vector
        class LTSFairnessObject : public RefCountable
        {
        private:
            string Name;
            EFSMBase* TheEFSM;
            vector<ExpT> IndexInst;
            u32 InstanceID;
            FairSetFairnessType Fairness;
            u32 FairnessID;
            // Raw pointer to avoid ref count cycles
            LTSFairnessSet* FairnessSet;

            static UIDGenerator FairnessUIDGenerator;

        public:
            LTSFairnessObject(LTSFairnessSet* FairnessSet,
                              const string& Name, EFSMBase* TheEFSM,
                              const vector<ExpT>& IndexInst,
                              FairSetFairnessType Fairness, u32 InstanceID);
            virtual ~LTSFairnessObject();

            const string& GetName() const;
            EFSMBase* GetEFSM() const;
            const vector<ExpT>& GetIndexInst() const;
            FairSetFairnessType GetFairnessType() const;
            u32 GetFairnessID() const;
            u32 GetInstanceID() const;

            LTSFairnessSet* GetFairnessSet() const;

            static void ResetFairnessUID();
        };

        // A parametrized fairness object, which consists of 
        // one or more instances of fairness objects
        class LTSFairnessSet : public RefCountable
        {
            friend class EFSMBase;

        private:
            string Name;
            EFSMBase* TheEFSM;
            vector<vector<ExpT>> AllInstances;
            map<vector<ExpT>, LTSFairObjRef> FairnessObjects;
            u32 FairnessIDLow;
            u32 FairnessIDHigh;
            u32 NumInstances;
            FairSetFairnessType Fairness;
            
            // Raw pointer to avoid ref counted cycles
            LTSProcessFairnessGroup* PFGroup;
            
        public:
            LTSFairnessSet(LTSProcessFairnessGroup* PFGroup,
                           const string& Name, EFSMBase* TheEFSM,
                           const vector<vector<ExpT>>& AllInstances,
                           FairSetFairnessType Fairness);
            virtual ~LTSFairnessSet();

            const string& GetName() const;
            EFSMBase* GetEFSM() const;
            const vector<vector<ExpT>>& GetAllInstances() const;
            const map<vector<ExpT>, LTSFairObjRef>& GetFairnessObjs() const;
            u32 GetFairnessIDLow() const;
            u32 GetFairnessIDHigh() const;
            u32 GetNumInstances() const;
            FairSetFairnessType GetFairnessType() const;

            LTSProcessFairnessGroup* GetPFGroup() const;

            const LTSFairObjRef& GetFairnessObj(const vector<ExpT>& Instance) const;
        };

        class LTSProcessFairnessGroup : public RefCountable
        {
            friend class EFSMBase;
        private:
            EFSMBase* TheEFSM;
            mutable map<string, LTSFairSetRef> FairnessSets;
            
        public:
            LTSProcessFairnessGroup(EFSMBase* TheEFSM);
            virtual ~LTSProcessFairnessGroup();
            
            void AddFairnessSet(const string& Name, FairSetFairnessType FairnessType) const;
            EFSMBase* GetEFSM() const;
            const map<string, LTSFairSetRef>& GetFairnessSets() const;
            
            const LTSFairSetRef& GetFairnessSet(const string& Name) const;
            const LTSFairObjRef& GetFairnessObj(const string& Name, 
                                                const vector<ExpT>& Instance) const;
        };
        
    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LTS_FAIRNESS_SET_HPP_ */

// 
// LTSFairnessSet.hpp ends here











