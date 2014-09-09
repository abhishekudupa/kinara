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

#include "../common/FwdDecls.hpp"

namespace ESMC {
    namespace MC {

        // A straightforward permuted path
        // where states are successors modulo some 
        // permutation
        // Assumption: nodes and edges are NOT owned by me
        template <typename NODETYPE, typename EDGETYPE>
        class PermutedPath
        {
        public:
            typedef pair<EDGETYPE, NODETYPE> PathElemType;
            
        private:
            NODETYPE Origin;
            vector<PathElemType> PathElems;

        public:
            inline PermutedPath(NODETYPE Origin,
                                const vector<PathElemType>& PathElems)
                : Origin(Origin), PathElems(PathElems)
            {
                // Nothing here
            }

            inline ~PermutedPath()
            {
                // Nothing here
            }

            inline NODETYPE GetOrigin() const
            {
                return Origin;
            }

            inline const vector<PathElemType>& GetPathElems() const
            {
                return PathElems;
            }
        };

        class TraceBase 
        {
        protected:
            const StateVec* InitialState;
            
        public:
            TraceBase(const StateVec* InitialState);
            virtual ~TraceBase();
            
            const StateVec* GetInitialState() const;

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

        class SafetyViolation : public TraceBase
        {
        protected:
            vector<TraceElemT> TraceElems;

        public:
            SafetyViolation(const StateVec* InitialState,
                            const vector<TraceElemT>& TraceElems);
            virtual ~SafetyViolation();

            const vector<TraceElemT>& GetTraceElems() const;
            virtual string ToString() const override;
        };

        class DeadlockViolation : public SafetyViolation
        {
        public:
            using SafetyViolation::SafetyViolation;
        };

        class LivenessViolation : public SafetyViolation
        {
        private:
            // The loop begins from the last state in the TraceElems
            // i.e., the first element in the vector LoopElems
            // represents the command to be executed from the 
            // last state in the stem (TraceElems), and the state resulting 
            // from that
            // The last state in LoopElems is the same as the last state in
            // the stem (TraceElems)
            vector<TraceElemT> LoopElems;

        public:
            LivenessViolation(const StateVec* InitialState,
                              const vector<TraceElemT>& Stem,
                              const vector<TraceElemT>& Lasso);
            virtual ~LivenessViolation();

            const vector<TraceElemT>& GetStem() const;
            const vector<TraceElemT>& GetLasso() const;

            virtual string ToString() const override;
        };

    } /* end namespace MC */
} /* end namespace ESMC */

#endif /* ESMC_TRACE_HPP_ */

// 
// Trace.hpp ends here
