// LTSEFSMBase.hpp ---
//
// Filename: LTSEFSMBase.hpp
// Author: Abhishek Udupa
// Created: Fri Aug 15 12:09:30 2014 (-0400)
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

#if !defined ESMC_LTS_EFSM_BASE_HPP_
#define ESMC_LTS_EFSM_BASE_HPP_

#include "LTSDecls.hpp"
#include "LTSState.hpp"
#include "LTSAutomaton.hpp"

namespace ESMC {
namespace LTS {

// Used to remember what messages were declared
// as a set of symmetric message types
class SymmetricMessageDecl : public RefCountable, public Stringifiable
{
private:
    TypeRef MessageType;
    vector<ExpT> NewParams;
    ExpT Constraint;
    vector<ExpT> MessageParams;
    bool Input;

public:
    SymmetricMessageDecl(const TypeRef& MessageType,
                         const vector<ExpT>& NewParams,
                         const ExpT& Constraint,
                         const vector<ExpT>& MessageParams,
                         bool Input);
    virtual ~SymmetricMessageDecl();

    const TypeRef& GetMessageType() const;
    const TypeRef& GetBaseMessageType() const;
    const vector<ExpT>& GetNewParams() const;
    const ExpT& GetConstraint() const;
    const vector<ExpT>& GetMessageParams() const;
    bool IsInput() const;
    bool IsOutput() const;
    virtual string ToString(u32 Verbosity = 0) const override;
};

class EFSMBase : public AutomatonBase
{
    friend class LabelledTS;
    friend class LTSProcessFairnessGroup;

protected:
    LTSFairnessType Fairness;
    bool VarsFrozen;
    bool EFSMFrozen;
    // State variable type
    TypeRef StateVarType;
    // SubstMap to rebase expressions
    // to use the state var fields
    map<vector<ExpT>, MgrT::SubstMapT> RebaseSubstMaps;
    // Symbolic Transitions
    vector<LTSSymbTransRef> SymbolicTransitions;
    // Transitions per instance
    map<vector<ExpT>, vector<LTSTransRef>> Transitions;
    LTSPFGRef Fairnesses;

    // Inputs and outputs per instance
    map<vector<ExpT>, set<TypeRef>> Inputs;
    map<vector<ExpT>, set<TypeRef>> Outputs;
    vector<SymmMsgDeclRef> SymmetricMessages;

    ExpT ErrorCondition;
    ExpT FinalCondition;

    // helper methods

    void AssertVarsFrozen() const;
    void AssertVarsNotFrozen() const;
    void AssertEFSMFrozen() const;
    void AssertEFSMNotFrozen() const;

    void AssertInput(const vector<ExpT>& ParamInst, const TypeRef& MessageType) const;
    void AssertOutput(const vector<ExpT>& ParamInst, const TypeRef& MessageType) const;

    void CheckMsgType(const TypeRef& Type) const;
    void CheckFairnessSets(const set<string>& FairnessSetNames) const;

    TypeRef InstantiateMessageType(const vector<ExpT>& Params,
                                   const MgrT::SubstMapT& SubstMap,
                                   const TypeRef& MsgType);

    SymmMsgDeclRef AddMsg(const TypeRef& MsgType,
                          const vector<ExpT>& Params,
                          bool IsInput);

    SymmMsgDeclRef AddMsgs(const vector<ExpT>& NewParams,
                           const ExpT& Constraint,
                           const TypeRef& MsgType,
                           const vector<ExpT>& Params,
                           bool IsInput);

    vector<LTSAssignRef> InstantiateUpdates(const MgrT::SubstMapT& ParamSubst,
                                            const vector<LTSAssignRef>& Updates,
                                            const string& MessageName,
                                            const TypeRef& MessageType,
                                            const TypeRef& ActMType);

    vector<LTSAssignRef> RebaseUpdates(const vector<ExpT>& ParamInst,
                                       const vector<LTSAssignRef>& Updates) const;
    vector<LTSAssignRef> MsgTransformUpdates(const vector<LTSAssignRef>& Updates,
                                             const string& MessageName,
                                             const TypeRef& MessageType);
    vector<LTSAssignRef> SimplifyUpdates(const vector<LTSAssignRef>& Updates);

    // For internal use. e.g. in the case of channels
    // expects final state to be part of updates
    void AddInputTransForInstance(u32 InstanceID,
                                  const MgrT::SubstMapT& SubstMap,
                                  const string& InitialState,
                                  const ExpT& Guard,
                                  const vector<LTSAssignRef>& Updates,
                                  const string& MessageName,
                                  const TypeRef& MessageType,
                                  const TypeRef& ActMType,
                                  const LTSSymbTransRef& SymbTrans);

    void AddOutputTransForInstance(u32 InstanceID,
                                   const MgrT::SubstMapT& SubstMap,
                                   const string& InitState,
                                   const ExpT& Guard,
                                   const vector<LTSAssignRef>& Updates,
                                   const string& MessageName,
                                   const TypeRef& MessageType,
                                   const TypeRef& ActMType,
                                   const FairObjSetT& FairnessObjsSatisfied,
                                   const LTSSymbTransRef& SymbTrans);

    void AddInternalTransForInstance(u32 InstanceID,
                                     const MgrT::SubstMapT& SubstMap,
                                     const string& InitState,
                                     const ExpT& Guard,
                                     const vector<LTSAssignRef>& Updates,
                                     const FairObjSetT& FairnessObjsSatisfied,
                                     const LTSSymbTransRef& SymbTrans);

public:
    EFSMBase(LabelledTS* TheLTS, const string& Name,
             const vector<ExpT>& Params, const ExpT& Constraint,
             LTSFairnessType Fairness = LTSFairnessType::None);
    virtual ~EFSMBase();

    // Methods not intended to be overridden.
    // This helps preserve the structure of the EFSM
    set<TypeRef> GetInputs() const;
    set<TypeRef> GetInputsForInstance(u32 InstanceID) const;

    set<TypeRef> GetOutputs() const;
    set<TypeRef> GetOutputsForInstance(u32 InstanceID) const;

    vector<LTSTransRef> GetOutputTransitionsOnMsg(const TypeRef& MsgType) const;
    vector<vector<LTSTransRef>> GetInputTransitionsOnMsg(const TypeRef& MsgType) const;
    vector<LTSTransRef> GetInternalTransitions() const;

    virtual void AddState(const string& StateName,
                          bool Initial = false, bool Final = false,
                          bool Accepting = false, bool Error = false) override;

    virtual LTSFairnessType GetFairnessType() const;

    virtual void AddFairnessSet(const string& Name, FairSetFairnessType FairnessType);
    virtual void AddFairnessSet(const string& Name, FairSetFairnessType FairnessType,
                                const vector<ExpT> NewParams, const ExpT& Constraint);

    virtual const LTSFairSetRef& GetFairnessSet(const string& FairnessName) const;
    virtual const LTSFairObjRef& GetFairnessObj(const string& FairnessName,
                                                const vector<ExpT>& InstParams) const;
    virtual const LTSFairObjRef& GetFairnessObj(const string& FairnessName,
                                                u32 InstanceNumber) const;

    virtual const LTSPFGRef& GetFairnessGroup() const;

    virtual void FreezeStates() override;
    virtual void FreezeVars();
    virtual void Freeze();

    virtual SymmMsgDeclRef AddInputMsg(const TypeRef& MessageType,
                                       const vector<ExpT>& Params = vector<ExpT>());

    virtual SymmMsgDeclRef AddInputMsgs(const vector<ExpT>& NewParams,
                                        const ExpT& Constraint,
                                        const TypeRef& MessageType,
                                        const vector<ExpT>& MessageParams);

    virtual SymmMsgDeclRef AddOutputMsg(const TypeRef& MessageType,
                                        const vector<ExpT>& Params = vector<ExpT>());

    virtual SymmMsgDeclRef AddOutputMsgs(const vector<ExpT>& NewParams,
                                         const ExpT& Constraint,
                                         const TypeRef& MessageType,
                                         const vector<ExpT>& MEssageParams);

    virtual void AddVariable(const string& VarName, const TypeRef& VarType);

    virtual void AddInputTransition(const string& InitState,
                                    const string& FinalState,
                                    const ExpT& Guard,
                                    const vector<LTSAssignRef>& Updates,
                                    const string& MessageName,
                                    const TypeRef& MessageType,
                                    const vector<ExpT>& MessageParams,
                                    bool Tentative = false);

    // Final state specified in update
    virtual void AddInputTransition(const string& InitState,
                                    const ExpT& Guard,
                                    const vector<LTSAssignRef>& Updates,
                                    const string& MessageName,
                                    const TypeRef& MessageType,
                                    const vector<ExpT>& MessageParams,
                                    bool Tentative = false);

    virtual void AddInputTransitions(const vector<ExpT>& TransParams,
                                     const ExpT& Constraint,
                                     const string& InitState,
                                     const string& FinalState,
                                     const ExpT& Guard,
                                     const vector<LTSAssignRef>& Updates,
                                     const string& MessageName,
                                     const TypeRef& MessageType,
                                     const vector<ExpT>& MessageParams,
                                     bool Tentative = false);

    // Final state specified in updates
    virtual void AddInputTransitions(const vector<ExpT>& TransParams,
                                     const ExpT& Constraint,
                                     const string& InitState,
                                     const ExpT& Guard,
                                     const vector<LTSAssignRef>& Updates,
                                     const string& MessageName,
                                     const TypeRef& MessageType,
                                     const vector<ExpT>& MessageParams,
                                     bool Tentative = false);

    virtual void AddOutputTransition(const string& InitState,
                                     const string& FinalState,
                                     const ExpT& Guard,
                                     const vector<LTSAssignRef>& Updates,
                                     const string& MessageName,
                                     const TypeRef& MessageType,
                                     const vector<ExpT>& MessageParams,
                                     const set<string>& AddToFairnessSets =
                                     set<string>(),
                                     bool Tentative = false);

    // Final state specified in updates
    virtual void AddOutputTransition(const string& InitState,
                                     const ExpT& Guard,
                                     const vector<LTSAssignRef>& Updates,
                                     const string& MessageName,
                                     const TypeRef& MessageType,
                                     const vector<ExpT>& MessageParams,
                                     const set<string>& AddToFairnessSets =
                                     set<string>(),
                                     bool Tentative = false);

    virtual void AddOutputTransitions(const vector<ExpT>& TransParams,
                                      const ExpT& Constraint,
                                      const string& InitState,
                                      const string& FinalState,
                                      const ExpT& Guard,
                                      const vector<LTSAssignRef>& Updates,
                                      const string& MessageName,
                                      const TypeRef& MessageType,
                                      const vector<ExpT>& MessageParams,
                                      const set<string>& GroupFairnessSets = set<string>(),
                                      const set<string>& IndividualFairnessSets =
                                      set<string>(),
                                      bool Tentative = false);

    // Final state specified in updates
    virtual void AddOutputTransitions(const vector<ExpT>& TransParams,
                                      const ExpT& Constraint,
                                      const string& InitState,
                                      const ExpT& Guard,
                                      const vector<LTSAssignRef>& Updates,
                                      const string& MessageName,
                                      const TypeRef& MessageType,
                                      const vector<ExpT>& MessageParams,
                                      const set<string>& GroupFairnessSets = set<string>(),
                                      const set<string>& IndividualFairnessSets =
                                      set<string>(),
                                      bool Tentative = false);

    virtual void AddInternalTransition(const string& InitState,
                                       const string& FinalState,
                                       const ExpT& Guard,
                                       const vector<LTSAssignRef>& Updates,
                                       const set<string>& AddToFairnessSets =
                                       set<string>(),
                                       bool Tentative = false);

    // Final state specified in updates
    virtual void AddInternalTransition(const string& InitState,
                                       const ExpT& Guard,
                                       const vector<LTSAssignRef>& Updates,
                                       const set<string>& AddToFairnessSets =
                                       set<string>(),
                                       bool Tentative = false);

    virtual void AddInternalTransitions(const vector<ExpT>& TransParams,
                                        const ExpT& Constraint,
                                        const string& InitState,
                                        const string& FinalState,
                                        const ExpT& Guard,
                                        const vector<LTSAssignRef>& Updates,
                                        const set<string>& GroupFairnessSets =
                                        set<string>(),
                                        const set<string>& IndividualFairnessSets =
                                        set<string>(),
                                        bool Tentative = false);

    // Final state specified in updates
    virtual void AddInternalTransitions(const vector<ExpT>& TransParams,
                                        const ExpT& Constraint,
                                        const string& InitState,
                                        const ExpT& Guard,
                                        const vector<LTSAssignRef>& Updates,
                                        const set<string>& GroupFairnessSets =
                                        set<string>(),
                                        const set<string>& IndividualFairnessSets =
                                        set<string>(),
                                        bool Tentative = false);

    virtual string ToString(u32 Verbosity = 0) const override;
    const MgrT::SubstMapT& GetRebaseSubstMap(const vector<ExpT>& ParamInst) const;

    const vector<LTSSymbTransRef>& GetSymbolicTransitions() const;

    vector<LTSSymbTransRef>
    GetSymbolicTransitions(const function<bool(const LTSSymbTransRef&)>& MatchPred) const;

    const vector<SymmMsgDeclRef>& GetSymmetricMessages() const;

    vector<SymmMsgDeclRef>
    GetSymmetricMessages(const function<bool(const SymmMsgDeclRef&)>& MatchPred) const;
};

} /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LTS_EFSM_BASE_HPP_ */

//
// LTSEFSMBase.hpp ends here
