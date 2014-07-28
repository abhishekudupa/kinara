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
#include "../symexec/Analyses.hpp"
#include "../containers/RefCountable.hpp"
#include "../containers/SmartPtr.hpp"

#include "LTSTypes.hpp"

#include <boost/functional/hash.hpp>

namespace ESMC {
    namespace LTS {

        enum class TransitionKind { INTERNAL, INPUT, OUTPUT };
        
        template <typename E, template <typename> class S, 
                  typename STATETYPE>
        class Transition
        {
        private:
            i32 FairnessSet;
            TransitionKind Kind;

            STATETYPE InitState;
            STATETYPE FinalState;
            Expr<E, S> Guard;
            vector<Assignment<E, S>> Updates;
            
            string MessageName;
            LTSTypeRef MessageType;

            // We use static constructor methods
            Transition()
                : Kind(INTERNAL)
            {
                // Nothing here
            }


        public:
            Transition(const Transition& Other)
                : Kind(Other.Kind), InitState(Other.InitState),
                  FinalState(Other.FinalState), Guard(Other.Guard),
                  Updates(Other.Updates), MessageName(Other.MessageName),
                  MessageType(Other.MessageType)
            {
                // Nothing here
            }

            ~Transition();
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

            inline i32 GetFairnessSet() const
            {
                return FairnessSet;
            }

            inline void SetFairnessSet(i32 FairnessSet) const
            {
                this->FairnessSet = FairnessSet;
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

            inline const Expr<E, S>& GetGuard() const
            {
                return Guard;
            }

            inline const string& GetMessageName() const
            {
                return MessageName;
            }

            inline const LTSTypeRef& GetMessageType() const
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
                                                  const Expr<E, S>& Guard,
                                                  const vector<Assignment<E, S>>& Updates,
                                                  const string& MessageName,
                                                  const LTSTypeRef& MessageType)
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
                                                   const Expr<E, S>& Guard,
                                                   const vector<Assignment<E, S>>& Updates,
                                                   const string& MessageName,
                                                   const LTSTypeRef& MessageType,
                                                   i32 FairnessSet = -1)
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
                                                     const Expr<E, S>& Guard,
                                                     const vector<Assignment<E, S>>& Updates,
                                                     i32 FairnessSet = -1)
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

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_TRANSITIONS_HPP_ */

// 
// Transitions.hpp ends here
