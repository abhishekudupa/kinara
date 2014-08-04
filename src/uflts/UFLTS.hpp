// UFLTS.hpp --- 
// 
// Filename: UFLTS.hpp
// Author: Abhishek Udupa
// Created: Wed Jul 23 15:13:43 2014 (-0400)
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


// This file contains the class definition of the UFLTS class.
// The UFLTS class is used to represent a labelled transition
// system with Uninterpreted Functions

#if !defined ESMC_UFLTS_HPP_
#define ESMC_UFLTS_HPP_

#include "../common/FwdDecls.hpp"
#include "../expr/Expressions.hpp"

#include "LTSTermSemanticizer.hpp"
#include "UFEFSM.hpp"
#include "UFLTSExtension.hpp"
#include "LTSUtils.hpp"

namespace ESMC {
    namespace LTS {
        
        extern const string MTypeFieldName;
        extern const u32 MaxMessageTypes;
        extern const string UnifiedMTypeName;

        class UFLTS 
        {
        private:
            MgrType* Mgr;
            bool Frozen;
            bool MsgsFrozen;
            UIDGenerator MTypeUIDGen;
            map<string, ExprTypeRef> MTypes;
            map<string, ExprTypeRef> PMTypes;
            map<ExprTypeRef, ExprTypeRef> TypeToPrimed;
            map<string, u32> MTypeIDs;
            ExprTypeRef UnifiedMType;
            u32 MessageSize;
            vector<UFEFSM*> EFSMs;
            vector<ChannelEFSM*> Channels;
            ExprTypeRef MessageIDType;

        public:
            UFLTS();
            ~UFLTS();
            MgrType* GetMgr() const;
            
            // Arbitrary types
            template <typename T, typename... ArgTypes>
            inline ExprTypeRef MakeType(ArgTypes&&... Args)
            {
                return Mgr->MakeType<T>(forward<ArgTypes>(Args)...);
            }

            // Non parametric MESSAGE type
            const ExprTypeRef& MakeMessageType(const string& Name, 
                                               const vector<pair<string, ExprTypeRef>>& Fields,
                                               bool IncludePrimed = false);

            // Parametric MESSAGE type
            const ExprTypeRef& MakeMessageType(const string& Name,
                                               const vector<pair<string, ExprTypeRef>>& Fields,
                                               const vector<ExpT>& ParamTypes,
                                               const ExpT& Constraint,
                                               bool IncludePrimed = false);

            void FreezeMessages();
            const ExprTypeRef& GetMessageType(const string& Name) const;
            const ExprTypeRef& GetPrimedType(const ExprTypeRef& Type) const;
            i32 GetTypeIDForMessageType(const string& Name) const;
            bool CheckMessageType(const ExprTypeRef& MType);
            u32 GetMessageSize() const;
            const ExprTypeRef& GetMessageIDType() const;
            const ExprTypeRef& GetUnifiedMType() const;

            UFEFSM* MakeEFSM(const string& Name, 
                             const vector<ExpT>& Params,
                             const ExpT& Constraint);
            
            ChannelEFSM* MakeChannel(const string& Name,
                                     const vector<ExpT>& Params,
                                     const ExpT& Constraint,
                                     u32 Capacity, bool Ordered, bool Lossy,
                                     bool Duplicating, bool Blocking);

            void Freeze();
        };

    } /* end namespace LTS */
} /* end namespace ESMC */

#endif /* ESMC_UFLTS_HPP_ */


// 
// UFLTS.hpp ends here
