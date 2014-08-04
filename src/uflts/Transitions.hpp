// Transitions.hpp --- 
// 
// Filename: Transitions.hpp
// Author: Abhishek Udupa
// Created: Sun Jul 27 11:26:32 2014 (-0400)
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

#if !defined ESMC_TRANSITIONS_HPP_
#define ESMC_TRANSITIONS_HPP_

#include "../common/FwdDecls.hpp"
#include "../expr/Expressions.hpp"
#include "../expr/ExprTypes.hpp"
#include "../symexec/Analyses.hpp"
#include "../containers/RefCountable.hpp"
#include "../containers/SmartPtr.hpp"


#include <boost/functional/hash.hpp>

namespace ESMC {
    namespace LTS {

        enum class TransitionKind { INTERNAL, INPUT, OUTPUT };
        
        template <typename E, template <typename> class S, 
                  typename ExpType, typename STATETYPE>
        class Transition
        {
        private:

            typedef Exprs::Expr<E, S> ExpT;
            typedef Analyses::Assignment<E, S> AsgnT;


            mutable unordered_set<u32> FairnessSet;
            TransitionKind Kind;

            STATETYPE InitState;
            STATETYPE FinalState;
            ExpT Guard;
            vector<AsgnT> Updates;
            
            string MessageName;
            ExpType MessageType;

        public:
            Transition()
            {
                // Nothing here
            }

            Transition(const Transition& Other)
                : Kind(Other.Kind), InitState(Other.InitState),
                  FinalState(Other.FinalState), Guard(Other.Guard),
                  Updates(Other.Updates), MessageName(Other.MessageName),
                  MessageType(Other.MessageType)
            {
                // Nothing here
            }

            ~Transition()
            {
                // Nothing here
            }
            
            inline Transition& operator = (const Transition& Other)
            {
                if (&Other == this) {
                    return *this;
                }

                Kind = Other.Kind;
                InitState = Other.InitState;
                FinalState = Other.FinalState;
                Guard = Other.Guard;
                Updates = Other.Updates;
                MessageName = Other.MessageName;
                MessageType = Other.MessageType;
                return *this;
            }

            inline const unordered_set<u32>& GetFairnessSet() const
            {
                return FairnessSet;
            }

            inline void SetFairnessSet(const unordered_set<u32>& FairnessSet) const
            {
                if (Kind == TransitionKind::INPUT) {
                    throw ESMCError((string)"An input transition cannot be part of " + 
                                    "a fairness constraint");
                }
                this->FairnessSet = FairnessSet;
            }

            inline void AddToFairnessSet(u32 Fairness)
            {
                this->FairnessSet.insert(Fairness);
            }

            inline TransitionKind GetKind() const
            {
                return Kind;
            }

            inline const STATETYPE& GetInitState() const
            {
                return InitState;
            }

            inline const STATETYPE& GetFinalState() const
            {
                return FinalState;
            }

            inline const ExpT& GetGuard() const
            {
                return Guard;
            }

            inline const vector<AsgnT>& GetUpdates() const
            {
                return Updates;
            }

            inline const string& GetMessageName() const
            {
                return MessageName;
            }

            inline const ExpType& GetMessageType() const
            {
                return MessageType;
            }

            inline bool IsInput() const
            {
                return (Kind == TransitionKind::INPUT);
            }

            inline bool IsOutput() const
            {
                return (Kind == TransitionKind::OUTPUT);
            }
            inline bool IsInternal() const
            {
                return (Kind == TransitionKind::INTERNAL);
            }

            static Transition MakeInputTransition(const STATETYPE& InitState,
                                                  const STATETYPE& FinalState,
                                                  const ExpT& Guard,
                                                  const vector<AsgnT>& Updates,
                                                  const string& MessageName,
                                                  const ExpType& MessageType)
            {
                Transition Retval;
                Retval.Kind = TransitionKind::INPUT;
                Retval.InitState = InitState;
                Retval.FinalState = FinalState;
                Retval.Guard = Guard;
                Retval.Updates = Updates;
                Retval.MessageName = MessageName;
                Retval.MessageType = MessageType;
                return Retval;
            }

            static Transition MakeOutputTransition(const STATETYPE& InitState,
                                                   const STATETYPE& FinalState,
                                                   const ExpT& Guard,
                                                   const vector<AsgnT>& Updates,
                                                   const string& MessageName,
                                                   const ExpType& MessageType,
                                                   const unordered_set<u32>& FairnessSet)
            {
                Transition Retval;
                Retval.Kind = TransitionKind::OUTPUT;
                Retval.InitState = InitState;
                Retval.FinalState = FinalState;
                Retval.Guard = Guard;
                Retval.Updates = Updates;
                Retval.MessageName = MessageName;
                Retval.MessageType = MessageType;
                Retval.FairnessSet = FairnessSet;
                return Retval;
            }

            static Transition MakeInternalTransition(const STATETYPE& InitState,
                                                     const STATETYPE& FinalState,
                                                     const ExpT& Guard,
                                                     const vector<AsgnT>& Updates,
                                                     const unordered_set<u32>& FairnessSet)
            {
                Transition Retval;
                Retval.Kind = TransitionKind::INTERNAL;
                Retval.InitState = InitState;
                Retval.FinalState = FinalState;
                Retval.Guard = Guard;
                Retval.Updates = Updates;
                Retval.FairnessSet = FairnessSet;
                return Retval;
            }
        };

        template <typename E, template <typename> class S>
        class GuardedCommand
        {
        private:
            typedef Exprs::Expr<E, S> ExpT;
            typedef Analyses::Assignment<E, S> AsgnT;
            typedef Exprs::ExprTypeRef ExprTypeRef;

            ExpT Guard;
            ExprTypeRef MessageType;
            string MessageName;
            vector<AsgnT> LeaderUpdates;
            vector<AsgnT> OtherUpdates;
            unordered_set<u32> FairnessSets;

        public:
            GuardedCommand(const ExpT& Guard,
                           const ExprTypeRef& MessageType,
                           const string& MessageName,
                           const vector<AsgnT>& LeaderUpdates,
                           const vector<AsgnT>& OtherUpdates,
                           const unordered_set<u32>& FairnessSets)
                : Guard(Guard), MessageType(MessageType),
                  MessageName(MessageName), LeaderUpdates(LeaderUpdates),
                  OtherUpdates(OtherUpdates), FairnessSets(FairnessSets)
            {
                // Nothing here
            }

            ~GuardedCommand()
            {
                // Nothing here
            }

            inline const ExpT& GetGuard() const
            {
                return Guard;
            }

            inline const vector<AsgnT>& GetLeaderUpdates() const
            {
                return LeaderUpdates;
            }

            inline const vector<AsgnT>& GetOtherUpdates() const
            {
                return OtherUpdates;
            }

            inline const unordered_set<u32>& GetFairnessSets() const
            {
                return FairnessSets;
            }

            inline const string& GetMessageName() const
            {
                return MessageName;
            }

            inline const string& GetMessageType() const
            {
                return MessageType;
            }
        };
    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_TRANSITIONS_HPP_ */

// 
// Transitions.hpp ends here
