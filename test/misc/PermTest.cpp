// PermTest.cpp --- 
// 
// Filename: PermTest.cpp
// Author: Abhishek Udupa
// Created: Wed Aug 13 18:17:53 2014 (-0400)
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

#include <vector>
#include "../../src/symmetry/Permutations.hpp"

using namespace std;
using namespace ESMC;
using namespace Symm;

#define NUMTYPES 2
#define NUMELEMS 4

static inline void PrintVec(const vector<u08>& Vec)
{
    for (u32 i = 0; i < NUMTYPES; ++i) {
        cout << "{ ";
        for (u32 j = 0; j < NUMELEMS; ++j) {
            cout << (u32)Vec[i * NUMELEMS + j] << " ";
        }
        cout << "} ";
    }
    cout << endl;
}

static inline void CheckInverse(const vector<u08>& Perm,
                                const vector<u08>& Inv)
{
    if (Perm.size() != Inv.size()) {
        cout << "Error: Size of inverse doesn't match!" << endl;
        exit(1);
    }
    for (unsigned int i = 0; i < NUMTYPES; ++i) {
        for (unsigned int j = 0; j < NUMELEMS; ++j) {
            if (Perm[i*NUMELEMS + Inv[i*NUMELEMS + j]] != j) {
                cout << "Error: Inverse check failed!" << endl;
                exit(1);
            }
        }
    }
}

int main()
{
    vector<u32> TypeSizes;
    for (u32 i = 0; i < NUMTYPES; ++i) {
        TypeSizes.push_back(NUMELEMS);
    }

    cout << "Testing compact permutation sets... " << endl << endl;
    // test for compact permutation sets
    auto Compact = new PermutationSet(TypeSizes, true);
    for (auto it = Compact->Begin(); it != Compact->End(); ++it) {
        cout << "Permutation:" << endl;
        PrintVec(it.GetPerm());
        auto it2 = Compact->GetIterator(it.GetIndex());
        auto Copy1 = it2.GetPerm();
        auto Copy2 = it.GetPerm();
        if (Copy1 != Copy2) {
            cout << "Error in iterator!" << endl;
            exit(1);
        }
        auto it3 = Compact->GetIteratorForInv(it.GetIndex());
        cout << "Got inverse:" << endl;
        PrintVec(it3.GetPerm());
        auto invperm = it3.GetPerm();
        CheckInverse(it.GetPerm(), invperm);
    }

    delete Compact;

    cout << endl << "Testing full permutation sets... " << endl << endl;

    auto Full = new PermutationSet(TypeSizes, false);
    for (auto it = Full->Begin(); it != Full->End(); ++it) {
        cout << "Permutation:" << endl;
        PrintVec(it.GetPerm());
        auto it2 = Full->GetIterator(it.GetIndex());
        auto Copy1 = it2.GetPerm();
        auto Copy2 = it.GetPerm();
        if (Copy1 != Copy2) {
            cout << "Error in iterator!" << endl;
            exit(1);
        }
        auto it3 = Full->GetIteratorForInv(it.GetIndex());
        cout << "Got inverse:" << endl;
        PrintVec(it3.GetPerm());
        auto invperm = it3.GetPerm();
        CheckInverse(it.GetPerm(), invperm);
    }
}

// 
// PermTest.cpp ends here
