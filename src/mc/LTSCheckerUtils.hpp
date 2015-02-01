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

#include <boost/pool/object_pool.hpp>

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


class PathFingerprint;
class PathFingerprintSet;

class PathFingerprint : public Stringifiable
{
private:
    unordered_set<u32> CommandsAlongPath;
    vector<u32> CommandsAlongPathVec;
    BitSet CommandsAlongPathBS;
    PathFingerprintSet* PathFPSet;
    u64 HashCode;

    inline i32 Compare(const PathFingerprint& Other) const;

public:
    PathFingerprint();
    template <typename ForwardIterator>
    inline PathFingerprint(const ForwardIterator& Begin,
                           const ForwardIterator& End,
                           u32 TotalNumCommands,
                           PathFingerprintSet* PathFPSet)
        : CommandsAlongPath(Begin, End),
          CommandsAlongPathVec(Begin, End),
          CommandsAlongPathBS(TotalNumCommands),
          PathFPSet(PathFPSet)
    {
        sort(CommandsAlongPathVec.begin(), CommandsAlongPathVec.end());
        for (auto const& CommandID : CommandsAlongPathVec) {
            CommandsAlongPathBS.Set(CommandID);
        }
        HashCode =
            SpookyHash::SpookyHash::Hash64(CommandsAlongPathVec.data(),
                                           sizeof(u32) * CommandsAlongPathVec.size(),
                                           0xDEEDBEADBEEFFEEDULL);
    }

    PathFingerprint(const PathFingerprint& Other);
    PathFingerprint(PathFingerprint&& Other);

    PathFingerprint& operator = (const PathFingerprint& Other);
    PathFingerprint& operator = (PathFingerprint&& Other);

    bool operator == (const PathFingerprint& Other) const;
    bool operator < (const PathFingerprint& Other) const;

    virtual ~PathFingerprint();
    // accessors
    const unordered_set<u32>& GetCommandsAlongPath() const;
    const vector<u32>& GetCommandsAlongPathVec() const;
    bool IsCommandAlongPath(u32 CommandID) const;
    const PathFingerprint* GetPathFPForAddedCommand(u32 CommandID, bool& IsFPNew) const;
    float GetCoverageForCommand(u32 CommandID) const;
    u32 GetNumCommandsAlongPath() const;
    u64 Hash() const;
    virtual string ToString(u32 Verbosity = 0) const override;
};

class PathFingerprintPtrHasher
{
public:
    inline u64 operator () (const PathFingerprint* FP) const
    {
        return FP->Hash();
    }
};

class PathFingerprintPtrEquals
{
public:
    inline bool operator () (const PathFingerprint* FP1,
                             const PathFingerprint* FP2) const
    {
        return (*FP1 == *FP2);
    }
};

class PathFingerprintPtrCompare
{
public:
    inline bool operator () (const PathFingerprint* FP1,
                             const PathFingerprint* FP2) const
    {
        return (*FP1 < *FP2);
    }
};


class PathFingerprintSet
{
private:
    typedef unordered_set<PathFingerprint*,
                          PathFingerprintPtrHasher,
                          PathFingerprintPtrEquals> PathFPSetT;
    PathFPSetT PathFPSet;
    boost::object_pool<PathFingerprint> PathFPPool;
    u32 TotalNumCommands;

public:
    PathFingerprintSet(u32 TotalNumCommands);
    ~PathFingerprintSet();

    template <typename ForwardIterator>
    inline const PathFingerprint* MakeFP(const ForwardIterator& Begin,
                                         const ForwardIterator& End,
                                         bool& IsFPNew)
    {
        auto NewFP = new (PathFPPool.malloc()) PathFingerprint(Begin, End, TotalNumCommands, this);
        auto it = PathFPSet.find(NewFP);
        if (it == PathFPSet.end()) {
            PathFPSet.insert(NewFP);
            IsFPNew = true;
            return NewFP;
        } else {
            PathFPPool.destroy(NewFP);
            IsFPNew = false;
            return *it;
        }
    }

    u32 GetTotalNumCommands() const;
};

class PathFPPrioritizer : public Stringifiable
{
private:
    unordered_map<u32, float> CommandCoverage;
    vector<const PathFingerprint*> FPPriorities;
    unordered_set<const PathFingerprint*> KnownPathFPs;
    float DesiredCoverage;
    const PathFingerprint* ZeroFP;

    inline void RedoFPPriorities();

public:
    PathFPPrioritizer(const vector<GCmdRef>& GuardedCommands,
                      const PathFingerprint* ZeroFP,
                      float DesiredCoverage);
    ~PathFPPrioritizer();

    void NotifyNewPathFP(const PathFingerprint* PathFP);
    void UpdatePriorities(const PathFingerprint* PathFPToError);
    const vector<const PathFingerprint*>& GetPriorities() const;
    bool IsCompletelyCovered() const;
    virtual string ToString(u32 Verbosity = 0) const override;
};

// A configurable BFS queue that can use a prio queue
// or a normal deque. Used to prioritize searches along
// minimally tainted paths
class BFSQueueT
{
private:
    typedef unordered_map<const PathFingerprint*, deque<StateVec*>*> CoveragePrioQueueT;

    deque<StateVec*>* BFSDeque;
    vector<deque<StateVec*>*>* SimplePrioQueues;
    CoveragePrioQueueT* CoveragePrioQueues;
    const PathFPPrioritizer* Prioritizer;
    static const u32 MaxNumBuckets;
    u32 NumElems;

public:
    BFSQueueT(BFSPrioMethodT PrioMethod,
              const PathFPPrioritizer* Prioritizer);
    ~BFSQueueT();
    void Push(StateVec* State);
    void Push(StateVec* State, u32 TaintLevel);
    void Push(StateVec* State, const PathFingerprint* FP);
    StateVec* Pop();
    StateVec* Pop(u32& TaintLevel);
    StateVec* Pop(const PathFingerprint*& FP);
    u32 Size() const;

    template <typename ForwardIterator, typename... ArgTypes>
    inline void Push(const ForwardIterator& Begin,
                     const ForwardIterator& End, ArgTypes&&... Args)
    {
        for (auto it = Begin; it != End; ++it) {
            Push(*it, forward<ArgTypes>(Args)...);
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
