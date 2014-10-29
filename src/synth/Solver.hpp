// Solver.hpp --- 
// 
// Filename: Solver.hpp
// Author: Abhishek Udupa
// Created: Thu Oct 23 11:07:12 2014 (-0400)
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

#if !defined ESMC_SOLVER_HPP_
#define ESMC_SOLVER_HPP_ 

#include "../common/FwdDecls.hpp"
#include "../containers/RefCountable.hpp"
#include "../uflts/LTSTransitions.hpp"
#include "../utils/UIDGenerator.hpp"
#include "../mc/AQStructure.hpp"

namespace ESMC {
    namespace Synth {

        using namespace ESMC::LTS;
        using namespace ESMC::MC;
        using namespace ESMC::TP;
        using namespace ESMC::Synth;

        extern const u64 TentativeEdgeCost;

        namespace Detail {

            // A cost functor for shortest paths
            class SynthCostFunction
            {
            private:
                const unordered_set<u32>& TentativeCommands;

            public:
                inline SynthCostFunction(const unordered_set<u32>& TentativeCommands) 
                    : TentativeCommands(TentativeCommands)
                {
                    // Nothing here
                }

                ~SynthCostFunction()
                {
                    // Nothing here
                }

                inline u64 operator () (const StateVec* SVPtr, const AQSEdge* Edge) const
                {
                    auto it = TentativeCommands.find(Edge->GetGCmdIndex());
                    if (it == TentativeCommands.end()) {
                        return 1;
                    } else {
                        return TentativeEdgeCost;
                    }
                }
            };

        } /* end namespace Detail */
        
        class Solver : public RefCountable
        {
            friend class ESMC::Analyses::TraceAnalyses;
            
        private:
            static const string BoundsVarPrefix;
            TPRef TP;
            LabelledTS* TheLTS;
            LTSCompiler* Compiler;
            LTSChecker* Checker;
            u32 Bound;
            vector<GCmdRef> GuardedCommands;
            unordered_set<u32> EnabledCommands;
            unordered_set<i64> InterpretedOps;
            unordered_map<i64, ExpT> IndicatorExps;
            Z3TheoremProver* TPAsZ3;
            Z3Ctx Ctx;
            UIDGenerator IndicatorUIDGenerator;

            // makes an assertion. Also fixes up interpretations
            // and marks the appropriate set of commands as having a
            // fixed interpretation.
            inline void MakeAssertion(const ExpT& Pred);
            inline void AssertBoundsConstraint();
            inline void HandleSafetyViolations();
            inline void HandleOneSafetyViolation(const StateVec* ErrorState,
                                                 const ExpT& BlownInvariant);
            inline void HandleOneDeadlockViolation(const StateVec* ErrorState);
            inline void HandleLivenessViolation(const LivenessViolation* Trace,
                                                StateBuchiAutomaton* Monitor);

        public:
            Solver(LTSChecker* Checker);
            virtual ~Solver();

            void Solve();
        };

    } /* end namespace Synth */
} /* end namespace ESMC */

#endif /* ESMC_SOLVER_HPP_ */

// 
// Solver.hpp ends here
