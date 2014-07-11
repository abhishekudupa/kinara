// Analyses.hpp --- 
// 
// Filename: Analyses.hpp
// Author: Abhishek Udupa
// Created: Fri Jul 11 11:49:35 2014 (-0400)
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

#if !defined ESMC_ANALYSES_HPP_
#define ESMC_ANALYSES_HPP_

#include "../expr/Expressions.hpp"
#include "boost/functional/hash.hpp"
#include <type_traits>

namespace ESMC { 
    namespace Analyses {
        
        using namespace Exprs;

        template <typename E, template <typename> class S>
        class Assignment
        {
        private:
            typedef Expr<E, S> ExpT;
            ExpT LHS;
            ExpT RHS;
            u64 HashCode;

        public:
            inline Assignment();
            inline Assignment(const ExpT& LHS, const ExpT& RHS);
            inline Assignment(const Assignment& Other);
            inline ~Assignment();
            
            inline bool operator == (const Assignment& Other) const;
            inline bool operator < (const Assignment& Other) const;
            inline Assignment& operator = (const Assignment& Other) const;

            inline const ExpT& GetLHS() const;
            inline const ExpT& GetRHS() const;

            inline u64 Hash() const;
            inline string ToString() const;
        };

        // implementation of assignment class
        template <typename E, template <typename> class S>
        inline Assignment<E, S>::Assignment()
            : LHS(ExpT::NullPtr), RHS(ExpT::NullPtr), HashCode(0)
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline Assignment<E, S>::Assignment(const ExpT& LHS, const ExpT& RHS)
            : LHS(LHS), RHS(RHS), HashCode(0)
        {
            if (LHS == ExpT::NullPtr || RHS == ExpT::NullPtr ||
                LHS->template As<VarExpression>() == nullptr) {
                throw ExprTypeError((string)"Args for constructing an assignment object " + 
                                    "must be non-null and LHS must be a VarExpression object");
            }
            boost::hash_combine(HashCode, LHS->Hash());
            boost::hash_combine(HashCode, RHS->Hash());
        }
        
        template <typename E, template <typename> class S>
        inline Assignment<E, S>::Assignment(const Assignment& Other)
            : LHS(Other.LHS), RHS(Other.RHS), HashCode(Other.HashCode)
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline Assignment<E, S>::~Assignment()
        {
            // Nothing here
        }

        template <typename E, template <typename> class S>
        inline bool Assignment<E, S>::operator == (const Assignment& Other) const
        {
            if (HashCode != Other.HashCode) {
                return false;
            }
            return (LHS == Other.LHS && RHS == Other.RHS);
        }

        template <typename E, template <typename> class S>
        inline bool Assignment<E, S>::operator < (const Assignment& Other) const
        {
            if (LHS < Other.LHS) {
                return true;
            } else if (LHS > Other.LHS) {
                return false;
            } else if (RHS < Other.RHS) {
                return true;
            } else {
                return false;
            }
        }

        template <typename E, template <typename> class S>
        inline Assignment<E, S>& Assignment<E, S>::operator = (const Assignment& Other) const
        {
            if (&Other == this) {
                return *this;
            }
            LHS = Other.LHS;
            RHS = Other.RHS;
            HashCode = Other.HashCode;
            return *this;
        }

        template <typename E, template <typename> class S>
        inline const typename Assignment<E, S>::ExpT&
        Assignment<E, S>::GetLHS() const
        {
            return LHS;
        }

        template <typename E, template <typename> class S>
        inline const typename Assignment<E, S>::ExpT&
        Assignment<E, S>::GetRHS() const
        {
            return RHS;
        }

        template <typename E, template <typename> class S>
        inline u64
        Assignment<E, S>::Hash() const
        {
            return HashCode;
        }

        template <typename E, template <typename> class S>
        inline string
        Assignment<E, S>::ToString() const
        {
            if (LHS == ExpT::NullPtr || RHS == ExpT::NullPtr) {
                return "nullasgn";
            }
            return (LHS->ToString() + " := " + RHS->ToString());
        }

        template<typename E, template <typename> class S>
        inline ostream& operator << (ostream& Out, const Assignment<E, S>& Asgn)
        {
            Out << Asgn.ToString();
            return Out;
        }

        // The actual analysis bits

        // Strongest post condition of a SEQUENCE of statements
        template <typename E, template <typename> class S,
                  class ForwardIterator>
        static inline Expr<E, S> StrongestPostSeq(const Expr<E, S>& Predicate,
                                                  const ForwardIterator& First,
                                                  const ForwardIterator& Last,
                                                  bool Simplify = false,
                                                  bool SimplifyStep = false)
        {
            auto Mgr = Predicate->GetMgr();
            typedef typename remove_pointer<decltype(Mgr)>::type MgrType;
            typedef typename MgrType::SemT SemT;
            typedef typename SemT::Ops Ops;
            auto Retval = Predicate;
            for (auto it = First; it != Last; ++it) {
                auto const& CurAsgn = *it;
                auto const& LHS = CurAsgn.GetLHS();
                auto const& RHS = CurAsgn.GetRHS();
                auto const LHSAsVar = LHS->template SAs<VarExpression>();
                auto EVar = Mgr->MakeBoundVar(LHSAsVar->GetVarType(), 0);

                typename MgrType::SubstMapT SubstMap = { { LHS, EVar } };
                auto SubstExp = Mgr->Substitute(SubstMap, Retval);
                auto SubstRHS = Mgr->Substitute(SubstMap, RHS);
                auto EQExp = Mgr->MakeExpr(Ops::OpEQ, LHS, SubstRHS);
                auto QBody = Mgr->MakeExpr(Ops::OpAND, EQExp, SubstExp);
                vector<i64> QVarTypes = { LHSAsVar->GetVarType() };
                Retval = Mgr->MakeExists(QVarTypes, QBody);

                if (SimplifyStep) {
                    Retval = Mgr->ElimQuantifiers(Retval);
                }
            }

            if (Simplify && !SimplifyStep) {
                Retval = Mgr->ElimQuantifiers(Retval);
            }
            return Retval;
        }

        // Expects the statements to be reversed already!
        // i.e., First is the LAST statement in the block
        template <typename E, template <typename> class S,
                  class ForwardIterator>
        static inline Expr<E, S> WeakestPreSeq(const Expr<E, S>& Predicate,
                                               const ForwardIterator& First,
                                               const ForwardIterator& Last)
        {
            auto Mgr = Predicate->GetMgr();
            typedef decltype(Mgr) MgrType;

            auto Retval = Predicate;
            for (auto it = First; it != Last; ++it) {
                auto const& CurAsgn = *it;
                auto const& LHS = CurAsgn.GetLHS();
                auto const& RHS = CurAsgn.GetRHS();
                typename MgrType::SubstMapT SubstMap = { { LHS, RHS } };
                Retval = Mgr->Substitute(SubstMap, Predicate);
            }
            return Retval;
        }

        // Analyses procedures for parallel assignment

        template <typename E, template <typename> class S,
                  class ForwardIterator>
        static inline Expr<E, S> StrongestPostPar(const Expr<E, S>& Predicate,
                                                  const ForwardIterator& First,
                                                  const ForwardIterator& Last,
                                                  bool Simplify = false)
        {
            auto Mgr = Predicate->GetMgr();
            typedef decltype(Mgr) MgrType;
            typedef Expr<E, S> ExpT;

            typedef typename MgrType::SemT SemT;
            typedef typename SemT::Ops Ops;
            
            auto Retval = Predicate;
            // Gather the list of variables to be quantified
            typename MgrType::SubstMapT SubstMap;

            auto CurIndex = 0;
            vector<i64> QVarTypesRev;
            for (auto it = First; it != Last; ++it) {
                auto const& CurAsgn = *it;
                auto const& LHS = CurAsgn.GetLHS();
                auto const& RHS = CurAsgn.GetRHS();
                auto const LHSAsVar = LHS->template SAs<VarExpression>();
                auto const EVar = Mgr->MakeBoundVar(LHSAsVar->GetVarType(), CurIndex++);
                QVarTypesRev.push_back(LHSAsVar->GetVarType());

                if (SubstMap.find(LHS) != SubstMap.end()) {
                    throw ExprTypeError((string)"Variable " + LHS->ToString() + " used " + 
                                        "multiple times in assignment list");
                }
                SubstMap[LHS] = EVar;
            }

            vector<i64> QVarTypes(QVarTypesRev.rbegin(), QVarTypesRev.rend());
            
            vector<ExpT> EQConstraints;
            for (auto it = First; it != Last; ++it) {
                auto SubstRHS = Mgr->Substitute(SubstMap, (*it).GetRHS());
                EQConstraints.push_back(Mgr->MakeExpr(Ops::OpEQ, (*it).GetLHS(), SubstRHS));
            }

            EQConstraints.push_back(Mgr->Substitute(SubstMap, Predicate));
            auto QBody = Mgr->MakeExpr(Ops::OpAND, EQConstraints);
            Retval = Mgr->MakeExists(QVarTypes, QBody);
            if (Simplify) {
                Retval = Mgr->ElimQuantifiers(Retval);
            }
            return Retval;
        }

        template <typename E, template <typename> class S,
                  class ForwardIterator>
        static inline Expr<E, S> WeakestPrePar(const Expr<E, S>& Predicate,
                                               const ForwardIterator& First,
                                               const ForwardIterator& Last)
        {
            auto Mgr = Predicate->GetMgr();
            typedef decltype(Mgr) MgrType;
            
            auto Retval = Predicate;
            typename MgrType::SubstMapT SubstMap;
            for (auto it = First; it != Last; ++it) {
                auto const& CurAsgn = *it;
                auto const& LHS = CurAsgn.GetLHS();
                auto const& RHS = CurAsgn.GetRHS();
                if (SubstMap.find(LHS) != SubstMap.end()) {
                    throw ExprTypeError((string)"Variable " + LHS->ToString() + " used " + 
                                        "multiple times in assignment list");
                }
                
                SubstMap[LHS] = RHS;
            }
            return Mgr->Substitute(SubstMap, Predicate);
        }
        
        
    } /* end namespace Analyses */
} /* end namespace ESMC */

#endif /* ESMC_ANALYSES_HPP_ */

// 
// Analyses.hpp ends here
