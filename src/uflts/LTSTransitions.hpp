// LTSTransitions.hpp ---
//
// Filename: LTSTransitions.hpp
// Author: Abhishek Udupa
// Created: Fri Aug  8 14:01:14 2014 (-0400)
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

#if !defined ESMC_LTS_TRANSITIONS_HPP_
#define ESMC_LTS_TRANSITIONS_HPP_

#include "../containers/RefCountable.hpp"

#include "LTSDecls.hpp"
#include "LTSState.hpp"


namespace ESMC {
namespace LTS {

class AutomatonTransitionBase : public RefCountable, public Stringifiable
{
protected:
    AutomatonBase* Automaton;
    LTSState InitState;
    ExpT Guard;
    // The parameters with which I was
    // instantiated
    vector<ExpT> ParamInst;

public:
    AutomatonTransitionBase(AutomatonBase* Automaton,
                            const vector<ExpT>& ParamInst,
                            const LTSState& InitState,
                            const ExpT& Guard);
    virtual ~AutomatonTransitionBase();

    AutomatonBase* GetAutomaton() const;
    const LTSState& GetInitState() const;
    const ExpT& GetGuard() const;
    const vector<ExpT>& GetParamInst() const;

    template <typename T>
    T* As()
    {
        return dynamic_cast<T*>(this);
    }

    template <typename T>
    const T* As() const
    {
        return dynamic_cast<const T*>(this);
    }

    template <typename T>
    T* SAs()
    {
        return static_cast<T*>(this);
    }

    template <typename T>
    const T* SAs() const
    {
        return static_cast<const T*>(this);
    }

    template <typename T>
    bool Is() const
    {
        return (dynamic_cast<const T*>(this) != nullptr);
    }
};

class LTSSymbTransitionBase : public RefCountable, public Stringifiable
{
protected:
    vector<ExpT> TransParams;
    vector<ExpT> Params;
    ExpT Constraint;
    AutomatonBase* Automaton;
    LTSState InitState;
    ExpT Guard;
    vector<LTSAssignRef> Updates;
    bool Tentative;

public:
    LTSSymbTransitionBase(const vector<ExpT>& TransParams,
                          const vector<ExpT>& Params,
                          const ExpT& Constraint,
                          AutomatonBase* Automaton,
                          const LTSState& InitState,
                          const ExpT& Guard,
                          const vector<LTSAssignRef>& Updates,
                          bool Tentative);
    virtual ~LTSSymbTransitionBase();

    const vector<ExpT>& GetTransParams() const;
    const vector<ExpT>& GetParams() const;
    const ExpT& GetConstraint() const;
    AutomatonBase* GetAutomaton() const;
    const LTSState& GetInitState() const;
    const ExpT& GetGuard() const;
    const vector<LTSAssignRef>& GetUpdates() const;
    bool IsTentative() const;

    template <typename T>
    inline T* As()
    {
        return dynamic_cast<T*>(this);
    }

    template <typename T>
    inline const T* As() const
    {
        return dynamic_cast<const T*>(this);
    }

    template <typename T>
    inline T* SAs()
    {
        return static_cast<T*>(this);
    }

    template <typename T>
    inline const T* SAs() const
    {
        return static_cast<const T*>(this);
    }

    template <typename T>
    inline bool Is() const
    {
        return (dynamic_cast<const T*>(this) != nullptr);
    }
};

class LTSSymbIOTransitionBase : public LTSSymbTransitionBase
{
protected:
    string MessageName;
    TypeRef MessageType;
    vector<ExpT> MessageParams;

public:
    LTSSymbIOTransitionBase(const vector<ExpT>& TransParams,
                            const vector<ExpT>& Params,
                            const ExpT& Constraint,
                            AutomatonBase* Automaton,
                            const LTSState& InitState,
                            const ExpT& Guard,
                            const vector<LTSAssignRef>& Updates,
                            const string& MessageName,
                            const TypeRef& MessageType,
                            const vector<ExpT>& MessageParams,
                            bool Tentative);
    virtual ~LTSSymbIOTransitionBase();

    const string& GetMessageName() const;
    const TypeRef& GetMessageType() const;
    const vector<ExpT>& GetMessageParams() const;
};

class LTSSymbInputTransition : public LTSSymbIOTransitionBase
{
public:
    using LTSSymbIOTransitionBase::LTSSymbIOTransitionBase;
    virtual ~LTSSymbInputTransition();

    virtual string ToString(u32 Verbosity) const override;
};

class LTSSymbOutputTransition : public LTSSymbIOTransitionBase
{
public:
    using LTSSymbIOTransitionBase::LTSSymbIOTransitionBase;
    virtual ~LTSSymbOutputTransition();

    virtual string ToString(u32 Verbosity) const override;
};

class LTSSymbInternalTransition : public LTSSymbTransitionBase
{
    using LTSSymbTransitionBase::LTSSymbTransitionBase;
    virtual ~LTSSymbInternalTransition();

    virtual string ToString(u32 Verbosity) const override;
};

class LTSTransitionBase : public AutomatonTransitionBase
{
protected:
    // The EFSM that this transition is a part of
    vector<LTSAssignRef> Updates;
    LTSSymbTransRef SymbolicTransition;

public:
    LTSTransitionBase(EFSMBase* TheEFSM,
                      const vector<ExpT>& ParamInst,
                      const LTSState& InitState,
                      const ExpT& Guard,
                      const vector<LTSAssignRef>& Updates,
                      const LTSSymbTransRef& SymbolicTransition);
    virtual ~LTSTransitionBase();

    EFSMBase* GetEFSM() const;
    const vector<LTSAssignRef>& GetUpdates() const;
    const LTSSymbTransRef& GetSymbolicTransition() const;
    bool IsTentative() const;
};

class LTSTransitionIOBase : public LTSTransitionBase
{
protected:
    string MessageName;
    TypeRef MessageType;

public:
    LTSTransitionIOBase(EFSMBase* TheEFSM,
                        const vector<ExpT>& ParamInst,
                        const LTSState& InitState,
                        const ExpT& Guard,
                        const vector<LTSAssignRef>& Updates,
                        const string& MessageName,
                        const TypeRef& MessageType,
                        const LTSSymbTransRef& SymbolicTransition);
    virtual ~LTSTransitionIOBase();

    const string& GetMessageName() const;
    const TypeRef& GetMessageType() const;
};

class LTSTransitionInput : public LTSTransitionIOBase
{
public:
    LTSTransitionInput(EFSMBase* TheEFSM,
                       const vector<ExpT>& ParamInst,
                       const LTSState& InitState,
                       const ExpT& Guard,
                       const vector<LTSAssignRef>& Updates,
                       const string& MessageName,
                       const TypeRef& MessageType,
                       const LTSSymbTransRef& SymbolicTransition);
    virtual ~LTSTransitionInput();

    virtual string ToString(u32 Verbosity = 0) const override;
};

class LTSTransitionOutput : public LTSTransitionIOBase
{
private:
    FairObjSetT FairnessObjsSatisfied;

public:
    LTSTransitionOutput(EFSMBase* TheEFSM,
                        const vector<ExpT>& ParamInst,
                        const LTSState& InitState,
                        const ExpT& Guard,
                        const vector<LTSAssignRef>& Updates,
                        const string& MessageName,
                        const TypeRef& MessageType,
                        const FairObjSetT& FairnessObjsSatisfied,
                        const LTSSymbTransRef& SymbolicTransition);
    virtual ~LTSTransitionOutput();

    const FairObjSetT& GetFairnessObjsSatisfied() const;

    virtual string ToString(u32 Verbosity = 0) const override;
};

class LTSTransitionInternal : public LTSTransitionBase
{
private:
    FairObjSetT FairnessObjsSatisfied;

public:
    LTSTransitionInternal(EFSMBase* TheEFSM,
                          const vector<ExpT>& ParamInst,
                          const LTSState& InitState,
                          const ExpT& Guard,
                          const vector<LTSAssignRef>& Updates,
                          const FairObjSetT& FairnessObjsSatisfied,
                          const LTSSymbTransRef& SymbolicTransition);
    virtual ~LTSTransitionInternal();

    const FairObjSetT& GetFairnessObjsSatisfied() const;
    virtual string ToString(u32 Verbosity = 0) const override;
};

class LTSGuardedCommand : public RefCountable, public Stringifiable
{
private:
    MgrT* Mgr;
    ExpT Guard;
    vector<ExpT> GuardComps;
    mutable ExpT LoweredGuard;
    vector<LTSAssignRef> Updates;
    mutable vector<LTSAssignRef> LoweredUpdates;
    TypeRef MsgType;
    i32 MsgTypeID;
    vector<LTSFairObjRef> FairnessObjsSatisfied;
    vector<LTSTransRef> ProductTrans;
    mutable u32 CmdID;
    const bool Tentative;
    mutable bool FullyInterpreted;
    ExpT FixedInterpretation;

public:
    LTSGuardedCommand(MgrT* Mgr,
                      const vector<ExpT>& GuardComps,
                      const vector<LTSAssignRef>& Updates,
                      const TypeRef& MsgType, i32 MsgTypeID,
                      const vector<LTSTransRef>& ProductTrans);
    virtual ~LTSGuardedCommand();

    MgrT* GetMgr() const;
    const ExpT& GetGuard() const;
    const vector<ExpT>& GetGuardComps() const;
    const vector<LTSAssignRef>& GetUpdates() const;
    const TypeRef& GetMsgType() const;
    i32 GetMsgTypeID() const;
    const vector<LTSFairObjRef>& GetFairnessObjsSatisfied() const;
    const vector<LTSTransRef>& GetProductTransition() const;
    u32 GetCmdID() const;
    void SetCmdID(u32 CmdID) const;
    virtual string ToString(u32 Verbosity = 0) const override;
    bool IsTentative() const;
    bool IsFullyInterpreted() const;
    void SetFullyInterpreted(bool NewValue) const;
    const ExpT& GetFixedInterpretation() const;
    const vector<LTSAssignRef>& GetLoweredUpdates() const;
    void SetLoweredUpdates(const vector<LTSAssignRef>& LoweredUpdates) const;
    const ExpT& GetLoweredGuard() const;
    void SetLoweredGuard(const ExpT& LoweredGuard) const;
};

class LTSInitState : public RefCountable
{
private:
    vector<ExpT> Params;
    ExpT Constraint;
    vector<LTSAssignRef> Updates;

public:
    LTSInitState(const vector<ExpT>& Params,
                 const ExpT& Constraint,
                 const vector<LTSAssignRef>& Updates);
    virtual ~LTSInitState();

    const vector<ExpT>& GetParams() const;
    const ExpT& GetConstraint() const;
    const vector<LTSAssignRef>& GetUpdates() const;
};

class LTSInitStateGenerator : public RefCountable, public Stringifiable
{
private:
    vector<LTSAssignRef> Updates;
    mutable vector<LTSAssignRef> LoweredUpdates;

public:
    LTSInitStateGenerator(const vector<LTSAssignRef>& Updates);
    virtual ~LTSInitStateGenerator();

    const vector<LTSAssignRef>& GetUpdates() const;
    void SetLoweredUpdates(const vector<LTSAssignRef>& LoweredUpdates) const;
    const vector<LTSAssignRef>& GetLoweredUpdates() const;
    virtual string ToString(u32 Verbosity = 0) const override;
};

} /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LTS_TRANSITIONS_HPP_ */

//
// LTSTransitions.hpp ends here
