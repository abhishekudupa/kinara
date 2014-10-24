// Solver.cpp --- 
// 
// Filename: Solver.cpp
// Author: Abhishek Udupa
// Created: Thu Oct 23 11:13:37 2014 (-0400)
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

#include "../tpinterface/TheoremProver.hpp"
#include "../mc/Trace.hpp"
#include "../uflts/LabelledTS.hpp"
#include "../uflts/LTSTypes.hpp"
#include "../uflts/LTSEFSM.hpp"

#include "Solver.hpp"

namespace ESMC {
    namespace Synth {
        
        using namespace ESMC::LTS;
        using namespace ESMC::MC;
        using namespace ESMC::TP;

        using LTS::ExpT;

        Solver::Solver(LabelledTS* TheLTS, LTSCompiler* Compiler)
            : TP(new Z3TheoremProver()), TheLTS(TheLTS), Compiler(Compiler), 
              Bound(0)
        {
            auto&& IncompleteEFSMs = 
                TheLTS->GetEFSMs([&](const EFSMBase* EFSM) -> bool
                                 {
                                     return EFSM->Is<IncompleteEFSM>();
                                 });
            for (auto const& EFSM : IncompleteEFSMs) {
                auto IncEFSM = EFSM->SAs<IncompleteEFSM>();
                auto const& Constraints = IncEFSM->Constraints;
                vector<ExpT> ConstraintVec(Constraints.begin(), Constraints.end());
                TP->Assert(ConstraintVec, true);
            }
        }

        Solver::~Solver()
        {
            // Nothing here
        }

        void Solver::Solve()
        {
            cout << "Beginning Solve... ";
            flush(cout);
            auto Res = TP->CheckSat();
            cout << "Done!" << endl;
            if (Res == TPResult::SATISFIABLE) {
                auto Model = TP->SAs<Z3TheoremProver>()->GetModel();
                cout << "Model:" << endl;
                cout << Model.ToString() << endl;
            } else if (Res == TPResult::UNSATISFIABLE) {
                cout << "Unsat!" << endl;
            } else if (Res == TPResult::UNKNOWN) {
                cout << "Unknown!" << endl;
            }
            
        }

    } /* end namespace Synth */
} /* end namespace ESMC */

// 
// Solver.cpp ends here
