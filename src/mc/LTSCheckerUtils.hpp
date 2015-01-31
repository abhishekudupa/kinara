// LTSCheckerUtils.hpp ---
// Filename: LTSCheckerUtils.hpp
// Author: Abhishek Udupa
// Created: Sat Jan 31 13:37:09 2015 (-0500)
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

#if !defined ESMC_LTS_CHECKER_UTILS_HPP_
#define ESMC_LTS_CHECKER_UTILS_HPP_

#include <deque>
#include <vector>

#include "../uflts/LTSDecls.hpp"
#include "../utils/BitSet.hpp"
#include "../hash/SpookyHash.hpp"
#include "../common/ESMCFwdDecls.hpp"
#include "../uflts/LTSFairnessSet.hpp"

#include "AQStructure.hpp"

namespace ESMC {
namespace MC {
namespace Detail {

using namespace LTS;

class DFSStackEntry
{
private:
    StateVec* State;
    i64 LastFired;

public:
    DFSStackEntry();
    DFSStackEntry(StateVec* State);
    DFSStackEntry(StateVec* State, u32 LastFired);
    DFSStackEntry(const DFSStackEntry& Other);
    ~DFSStackEntry();

    DFSStackEntry& operator = (const DFSStackEntry& Other);
    bool operator == (const DFSStackEntry& Other) const;

    i64& GetLastFired();
    i64 GetLastFired() const;
    const StateVec* GetState() const;
    StateVec* GetState();

    void SetLastFired(i64 NewLastFired);
};

// A configurable BFS queue that can use a prio queue
// or a normal deque. Used to prioritize searches along
// minimally tainted paths
class BFSQueueT
{
private:
    deque<StateVec*>* BFSDeque;
    vector<deque<StateVec*>*>* PrioQueues;
    static const u32 MaxNumBuckets;
    u32 NumElems;

public:
    BFSQueueT(bool UsePrioQueue = false);
    ~BFSQueueT();
    void Push(StateVec* State, u32 TaintLevel = 0);
    StateVec* Pop(u32& TaintLevel);
    u32 Size() const;

    template <typename ForwardIterator>
    inline void Push(const ForwardIterator& Begin,
                     const ForwardIterator& End, u32 TaintLevel)
    {
        for (auto it = Begin; it != End; ++it) {
            Push(*it, TaintLevel);
        }
    }
};

class FairnessChecker : public Stringifiable
{
    friend class ESMC::MC::TraceBase;
    friend class ESMC::MC::LTSChecker;

private:
    // Info that needs to be reset before each ThreadedBFS
    bool Enabled;
    bool Disabled;
    bool Executed;
    // Set of states where I'm enabled, but not taken.
    // Needed for removing unfair states from SCCs.
    unordered_set<const ProductState*> EnabledStates;
    // end of info that needs to reset before each ThreadedBFS

    // The fairness set that I check satisfaction of
    LTSFairSetRef FairSet;
    // How many instances do I have
    u32 NumInstances;
    // Are we tracking a strong fairness
    bool IsStrong;
    // The global system index set
    SystemIndexSet* SysIdxSet;
    // The fairness class ID of the fairness set
    u32 ClassID;
    // Guarded commands I need to respond to for each instance
    vector<BitSet> GCmdsToRespondTo;
    // Same info as above, but in the form of sets
    vector<unordered_set<u32>> GCmdIDsToRespondTo;
    LTSChecker* Checker;


    // Fields that help in trace generation
    // These fields need to be reset for each SCC

    // The instances that have already satisfied in the trace so far
    // Info about how instances were satisfied
    BitSet EnabledPerInstance;
    BitSet ExecutedPerInstance;
    BitSet DisabledPerInstance;
    // End of fields that need to be reset for each SCC

    // This field is used only in trace generation
    mutable BitSet SatisfiedInTrace;
    // End of fields to help in trace generation

public:
    FairnessChecker(const LTSFairSetRef& FairSet,
                    SystemIndexSet* SysIdxSet,
                    const vector<GCmdRef>& GuardedCommands,
                    LTSChecker* Checker);
    ~FairnessChecker();

    void InitializeForNewSCC();
    void InitializeForNewThread();

    // Reset everything as if just constructed
    void Reset();
    void AcceptSCCState(const ProductState* State,
                        const ProductEdgeSetT& Edges,
                        u32 CurrentTrackedIndex,
                        u32 OriginalTrackedIndex);

    bool IsEnabled() const;
    bool IsExecuted() const;
    bool IsDisabled() const;

    bool IsFair() const;
    bool IsStrongFairness() const;
    virtual string ToString(u32 Verbosity = 0) const override;

    const unordered_set<const ProductState*>& GetEnabledStates() const;


    // Methods to help in trace generation
    void ClearTraceSatisfactionBits() const;
    bool IsInstanceSatisfiedInTrace(u32 Instance) const;
    void SetInstanceSatisfiedInTrace(u32 Instance) const;
    bool CheckInstanceSatisfaction(u32 Instance) const;
    bool CheckInstanceSatisfaction(u32 Instance,
                                   u32 CmdID,
                                   const ProductState* ReachedState) const;

    // Permutes the info stored in this checker
    // This is currently IRREVERSIBLE!
    // (no use case yet for undoing this)
    // Of course, one can remember the permutation
    // originally applied and then apply the
    // corresponding inverse permutation to reverse it!
    void Permute(const vector<u08>& Permutation);
    bool IsEnabled(u32 Instance) const;
    bool IsDisabled(u32 Instance) const;
    bool IsExecuted(u32 Instance) const;
    // end of methods to help in trace generation
};

} /* end namespace Detail */
} /* end namespace MC */
} /* end namespace ESMC */

#endif /* ESMC_LTS_CHECKER_UTILS_HPP_ */

//
// LTSCheckerUtils.hpp ends here
