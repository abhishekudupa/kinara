// Trace.hpp ---
//
// Filename: Trace.hpp
// Author: Abhishek Udupa
// Created: Mon Aug 25 15:10:30 2014 (-0400)
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

#if !defined ESMC_TRACE_HPP_
#define ESMC_TRACE_HPP_

#include <vector>
#include <boost/functional/hash.hpp>
#include <unordered_set>
#include <tuple>

#include "../common/ESMCFwdDecls.hpp"
#include "../uflts/LTSDecls.hpp"

namespace ESMC {
    namespace MC {

        using LTS::GCmdRef;
        using LTS::LabelledTS;
        using LTS::ExpT;
        using Symm::Canonicalizer;
        using LTS::LTSAssignRef;

        namespace Detail {

            class FairnessChecker;

            class PSPermPairHasher
            {
            public:
                inline u64 operator () (const pair<const ProductState*, u32>& ThePair) const
                {
                    u64 Retval = 0;
                    boost::hash_combine(Retval, ThePair.first);
                    boost::hash_combine(Retval, ThePair.second);
                    return Retval;
                }
            };

            typedef pair<const ProductState*, u32> PSPermPairT;
            // for storing unwound states
            typedef unordered_set<PSPermPairT, PSPermPairHasher> PSPermSetT;
            // Product state, inverse permutation, inverse sort permutation
            typedef tuple<const ProductState*, u32, u32> BFSQueueEntryT;

            class BFSQueueEntryHasher
            {
            public:
                inline u64 operator () (const BFSQueueEntryT& Entry) const
                {
                    u64 Retval = 0;
                    boost::hash_combine(Retval, get<0>(Entry));
                    boost::hash_combine(Retval, get<1>(Entry));
                    boost::hash_combine(Retval, get<2>(Entry));
                    return Retval;
                }
            };

            // For storing unwound edges
            // unwound(first) <-- unwound(second.second) with command (second.first)
            typedef unordered_map<BFSQueueEntryT,
                                  pair<u32, BFSQueueEntryT>,
                                  BFSQueueEntryHasher> UnwoundPredMapT;

        } /* end namespace detail */

        using namespace Detail;

        // A straightforward permuted path
        // where states are successors modulo some
        // permutation
        // Assumption: nodes and edges are NOT owned by me
        template <typename STATETYPE>
        class PermutedPath
        {
        public:
            typedef AnnotatedEdge<STATETYPE>* PathElemType;

        private:
            const STATETYPE* Origin;
            vector<PathElemType> PathElems;

        public:
            inline PermutedPath(const STATETYPE* Origin,
                                const vector<PathElemType>& PathElems)
                : Origin(Origin), PathElems(PathElems)
            {
                // Nothing here
            }

            inline ~PermutedPath()
            {
                // Nothing here
            }

            inline const STATETYPE* GetOrigin() const
            {
                return Origin;
            }

            inline const vector<PathElemType>& GetPathElems() const
            {
                return PathElems;
            }
        };

        class TraceBase : public Stringifiable
        {
        protected:
            StateVecPrinter* Printer;

        public:
            TraceBase(StateVecPrinter* Printer);
            virtual ~TraceBase();

            StateVecPrinter* GetPrinter() const;

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

        private:
            static inline const StateVec*
            UnwindPermPath(AQSPermPath* PermPath,
                           LTSChecker* Checker,
                           vector<TraceElemT>& PathElems);
            static inline const StateVec*
            UnwindPermPath(AQSPermPath* PermPath,
                           LTSChecker* CHecker,
                           vector<TraceElemT>& PathElems,
                           u32& InvPermAlongPathOut);

            static inline const ProductState*
            UnwindPermPath(PSPermPath* PermPath,
                           LTSChecker* Checker,
                           vector<PSTraceElemT>& PathElems,
                           u32& InvPermAlongPath);

            static inline unordered_set<const ProductState*>
            ExpandSCC(const ProductState* SCCRoot, LTSChecker* Checker);

            // returns a pair of states:
            // 1. The permuted state in the product structure corresponding
            //    to the last unwound state in the path (for subsequent calls to
            //    DoUnwoundBFS)
            // 2. The inverse permutation along the path is returned in InvPermAlongPathOut
            static inline const ProductState*
            DoUnwoundBFS(const ProductState* CanonicalRoot,
                         const LTSChecker* Checker,
                         u32& InvPermAlongPathOut,
                         u32& InvSortPermForRoot,
                         FairnessChecker* FChecker,
                         u32 InstanceID,
                         const function<bool(u32, const ProductState*)>& MatchPred,
                         vector<PSTraceElemT>& PathElems,
                         const unordered_set<const ProductState*>& Bounds);

            static inline bool CheckFairnessSat(const vector<PSTraceElemT>& PathSoFar,
                                                const Detail::FairnessChecker* FChecker,
                                                const vector<GCmdRef>& GuardedCmds,
                                                u32 InstanceID);

            static inline void
            MarkFairnessesSatisfied(const vector<pair<PSTraceElemT, u32>>& PathSegment,
                                    const LTSChecker* Checker);


        public:
            static SafetyViolation* MakeSafetyViolation(const StateVec* ErrorState,
                                                        LTSChecker* Checker,
                                                        const ExpT& BlownInvariant);
            static SafetyViolation* MakeBoundsViolation(const StateVec* ErrorState,
                                                        LTSChecker* Checker);
            static DeadlockViolation* MakeDeadlockViolation(const StateVec* ErrorState,
                                                            LTSChecker* Checker);
            // Accepts the root of a fair accepting (green) SCC.
            static LivenessViolation* MakeLivenessViolation(const ProductState* SCCRoot,
                                                            LTSChecker* Checker);

            static SafetyViolation* MakeSafetyViolation(AQSPermPath* PermPath,
                                                        LTSChecker* Checker,
                                                        const ExpT& BlownInvariant);
            static SafetyViolation* MakeBoundsViolation(AQSPermPath* PermPath,
                                                        LTSChecker* Checker);

            static DeadlockViolation* MakeDeadlockViolation(AQSPermPath* PermPath,
                                                            LTSChecker* Checker);
        };

        class SafetyViolation : public TraceBase
        {
            friend class TraceBase;

        protected:
            const StateVec* InitialState;
            vector<TraceElemT> TraceElems;
            ExpT BlownInvariant;

            SafetyViolation(const StateVec* InitialState,
                            const vector<TraceElemT>& TraceElems,
                            StateVecPrinter* Printer,
                            const ExpT& BlownInvariant);

        public:
            virtual ~SafetyViolation();
            const StateVec* GetInitialState() const;
            const vector<TraceElemT>& GetTraceElems() const;
            virtual string ToString(u32 Verbosity = 0) const override;
            const ExpT& GetInvariantBlown() const;
        };

        class DeadlockViolation : public SafetyViolation
        {
            friend class TraceBase;

        protected:
            DeadlockViolation(const StateVec* InitialState,
                              const vector<TraceElemT>& TraceElems,
                              StateVecPrinter* Printer,
                              const ExpT& BlownInvariant);
        public:
            virtual ~DeadlockViolation();
        };

        class LivenessViolation : public TraceBase
        {
            friend class TraceBase;

        private:
            const ProductState* InitialState;
            const ProductStructure* ThePS;
            vector<PSTraceElemT> StemPath;
            // The updates that need to be made
            // after executing the last command in the stem
            // to get the true stem last state
            vector<LTSAssignRef> StemSortPermutation;
            vector<PSTraceElemT> LoopPath;
            // The updates to be made after executing the
            // last command in the loop to get back to the
            // last state of the stem
            vector<LTSAssignRef> LoopSortPermutation;

            LivenessViolation(const ProductState* InitialState,
                              const vector<PSTraceElemT>& Stem,
                              const vector<LTSAssignRef>& StemSortPermutation,
                              const vector<PSTraceElemT>& Loop,
                              const vector<LTSAssignRef>& LoopSortPermutation,
                              StateVecPrinter* Printer,
                              const ProductStructure* ThePS);

        public:
            virtual ~LivenessViolation();

            const ProductState* GetInitialState() const;
            const vector<PSTraceElemT>& GetStem() const;
            const vector<PSTraceElemT>& GetLoop() const;

            virtual string ToString(u32 Verbosity = 0) const override;
            const vector<LTSAssignRef>& GetStemSortPermutation() const;
            const vector<LTSAssignRef>& GetLoopSortPermutation() const;
        };

    } /* end namespace MC */
} /* end namespace ESMC */

#endif /* ESMC_TRACE_HPP_ */

//
// Trace.hpp ends here
