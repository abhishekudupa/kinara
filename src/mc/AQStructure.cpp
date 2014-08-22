 // AQStructure.cpp --- 
 // 
 // Filename: AQStructure.cpp
 // Author: Abhishek Udupa
 // Created: Tue Aug 19 11:51:55 2014 (-0400)
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

 #include "AQStructure.hpp"

 namespace ESMC {
     namespace MC {

         AQSEdge::AQSEdge()
             : Target(nullptr), Permutation(0)
         {
             // Nothing here
         }

         AQSEdge::AQSEdge(const StateVec* Target, u32 Permutation)
             : Target(Target), Permutation(Permutation)
         {
             // Nothing here
         }

         AQSEdge::AQSEdge(const AQSEdge& Other) 
             : Target(Other.Target), Permutation(Other.Permutation)
        {
            // Nothing here
        }

         AQSEdge::~AQSEdge()
         {
             // Nothing here
         }

         AQSEdge& AQSEdge::operator = (const AQSEdge& Other)
         {
             if (&Other == this) {
                 return *this;
             }
             Target = Other.Target;
             Permutation = Other.Permutation;
             return *this;
         }

         bool AQSEdge::operator == (const AQSEdge& Other) const
         {
             return (Target == Other.Target && Permutation == Other.Permutation);
         }

         const StateVec* AQSEdge::GetTarget() const
         {
             return Target;
         }

         u32 AQSEdge::GetPermutation() const
         {
             return Permutation;
         }

         AQStructure::AQStructure()
         {
             // Nothing here
         }

         AQStructure::~AQStructure()
         {
             // Nothing here
         }

         StateVec* AQStructure::Find(StateVec* SV) const
         {
             auto it = StateSet.find(SV);
             if (it == StateSet.end()) {
                 return nullptr;
             } else {
                 return it->first;
             }
         }

         void AQStructure::Insert(StateVec* SV)
         {
             if (Find(SV)) {
                 return;
             }
             StateSet[SV] = AQSEdgeSetT();
         }

         void AQStructure::InsertInitState(StateVec* SV)
         {
             if (Find(SV)) {
                 return;
             }
             StateSet[SV] = AQSEdgeSetT();
             InitStates.push_back(SV);
         }

         void AQStructure::AddEdge(StateVec* Source,
                                   StateVec* Target,
                                   u32 Permutation)
         {
             auto it = StateSet.find(Source);
             AQSEdge NewEdge(Target, Permutation);
             auto it2 = it->second.find(NewEdge);
             if (it2 != it->second.end()) {
                 return;
             }
             it->second.insert(NewEdge);
         }

         u64 AQStructure::GetNumStates() const
         {
             return StateSet.size();
         }

    } /* end namespace MC */
} /* end namespace ESMC */

// 
// AQStructure.cpp ends here
