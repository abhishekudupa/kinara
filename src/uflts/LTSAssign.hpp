// LTSAssign.hpp --- 
// 
// Filename: LTSAssign.hpp
// Author: Abhishek Udupa
// Created: Fri Aug  8 14:04:10 2014 (-0400)
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

#if !defined ESMC_LTS_ASSIGN_HPP_
#define ESMC_LTS_ASSIGN_HPP_ 

#include "../containers/RefCountable.hpp"

#include "LTSTypes.hpp"

namespace ESMC {
    namespace LTS {

        class LTSAssignBase : public RefCountable
        {
        protected:
            ExpT LHS;
            ExpT RHS;

        public:
            LTSAssignBase();
            LTSAssignBase(const ExpT& LHS, const ExpT& RHS);
            virtual ~LTSAssignBase();

            const ExpT& GetLHS() const;
            const ExpT& GetRHS() const;
            virtual string ToString() const = 0;
            virtual vector<LTSAssignRef> ExpandNonScalarUpdates() const = 0;

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
            T* SAs() const
            {
                return static_cast<const T*>(this);
            }

            template <typename T>
            bool Is() const
            {
                return (dynamic_cast<const T*>(this) != nullptr);
            }
        };

        class LTSAssignSimple : public LTSAssignBase
        {
        public:
            using LTSAssignBase::LTSAssignBase;
            virtual ~LTSAssignSimple();

            virtual string ToString() const override;
            virtual vector<LTSAssignRef> ExpandNonScalarUpdates() const override;
        };

        class LTSAssignParam : public LTSAssignBase
        {
        private:
            vector<ExpT> Params;
            ExpT Constraint;

        public:
            LTSAssignParam(const vector<ExpT>& Params,
                           const ExpT& Constraint,
                           const ExpT& LHS, const ExpT& RHS);
            virtual ~LTSAssignParam();
            const vector<ExpT>& GetParams() const;
            const ExpT& GetConstraint() const;

            virtual string ToString() const override;
            virtual vector<LTSAssignRef> ExpandNonScalarUpdates() const override;
        };

        // helper method to expand a set of LTSAssignRefs
        static inline vector<LTSAssignRef> ExpandUpdates(const vector<LTSAssignRef>& Updates)
        {
            vector<LTSAssignRef> Retval;
            for (auto const& Update : Updates) {
                if (!Update->Is<LTSAssignSimple>()) {
                    throw InternalError((string)"ExpandUpdates() called on a non-simple " + 
                                        "update:\n" + Update->ToString() + "\nAt: " + 
                                        __FILE__ + ":" + to_string(__LINE__));
                }

                auto&& Expansions = Update->ExpandNonScalarUpdates();
                Retval.insert(Retval.end(), Expansions.begin(), Expansions.end());
            }
            return Retval;
        }
        

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_LTS_ASSIGN_HPP_ */

// 
// LTSAssign.hpp ends here
