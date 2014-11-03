// LTSAutomaton.hpp ---
//
// Filename: LTSAutomaton.hpp
// Author: Abhishek Udupa
// Created: Fri Aug 15 12:02:09 2014 (-0400)
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

#if !defined ESMC_LTS_AUTOMATON_HPP_
#define ESMC_LTS_AUTOMATON_HPP_

#include "../utils/UIDGenerator.hpp"

#include "LTSTypes.hpp"
#include "LTSState.hpp"
#include "SymbolTable.hpp"

namespace ESMC {
    namespace LTS {

        class AutomatonBase
        {
        protected:
            LabelledTS* TheLTS;
            string Name;
            vector<ExpT> Params;
            ExpT Constraint;
            SymbolTable SymTab;
            map<string, LTSState> States;
            ExprTypeRef StateType;
            vector<vector<ExpT>> ParamInsts;
            vector<MgrT::SubstMapT> ParamSubsts;
            u32 ClassID;

            bool StatesFrozen;

            void AssertStatesFrozen() const;
            void AssertStatesNotFrozen() const;
            void CheckState(const string& Name) const;

        public:
            AutomatonBase(LabelledTS* TheLTS, const string& Name,
                          const vector<ExpT>& Params, const ExpT& Constraint);
            virtual ~AutomatonBase();

            virtual void FreezeStates();
            virtual void AddState(const string& StateName,
                                  bool Initial = false, bool Final = false,
                                  bool Accepting = false, bool Error = false);

            const string& GetName() const;
            vector<LTSState> GetStates() const;
            const ExprTypeRef& GetStateType() const;
            const vector<vector<ExpT>>& GetParamInsts() const;
            const vector<MgrT::SubstMapT>& GetParamSubsts() const;
            u32 GetNumInstances() const;
            u32 GetNumInstancesUnconstrained() const;
            u32 GetClassID() const;
            virtual string ToString() const = 0;

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

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LTS_AUTOMATON_HPP_ */

//
// LTSAutomaton.hpp ends here
