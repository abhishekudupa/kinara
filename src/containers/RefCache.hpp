// RefCache.hpp ---
//
// Filename: RefCache.hpp
// Author: Abhishek Udupa
// Created: Thu Jul 24 11:01:32 2014 (-0400)
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

#if !defined ESMC_REF_CACHE_HPP_
#define ESMC_REF_CACHE_HPP_

#include "../common/FwdDecls.hpp"
#include <unordered_set>
#include <vector>

namespace ESMC {

    // Set NUMWEAKREFS to a negative value for a permanent store,
    // i.e., GC will never kick in
    template <typename OBJTYPE, typename HASHERTYPE, typename EQTYPE,
              template <typename> class REFTYPE = SmartPtr, i64 NUMWEAKREFS=0>
    class RefCache
    {
    private:
        typedef REFTYPE<OBJTYPE> PtrType;
        typedef unordered_set<PtrType, HASHERTYPE, EQTYPE> SetType;

        SetType Cache;
        u32 NextGC;
        float GCGrowthFactor;

        inline void AutoGC()
        {
            if (NUMWEAKREFS < 0) {
                return;
            }
            if (Cache.size() >= NextGC) {
                GC();
                NextGC = (u32)((float)Cache.size() * GCGrowthFactor);
            }
        }

    public:
        inline RefCache(u32 InitialCapacity = 1024, float GrowthFactor = 1.5f)
            : NextGC(InitialCapacity), GCGrowthFactor(GrowthFactor)
        {
            // Nothing here
        }

        inline ~RefCache()
        {
            Cache.clear();
        }

        template <typename T, typename... ArgTypes>
        inline PtrType Get(ArgTypes&&... Args)
        {
            PtrType NewExp = new T(forward<ArgTypes>(Args)...);
            return Get(NewExp);
        }

        inline PtrType Get(const PtrType& Obj)
        {
            AutoGC();
            auto it = Cache.find(Obj);
            if (it == Cache.end()) {
                Cache.insert(Obj);
                return Obj;
            } else {
                return (*it);
            }
        }

        inline PtrType Find(const PtrType& Obj)
        {
            auto it = Cache.find(Obj);
            if (it == Cache.end()) {
                return PtrType::NullPtr;
            } else {
                return (*it);
            }
        }

        // assumes the value is not already
        // in the cache
        inline PtrType Put(const PtrType& Obj)
        {
            Cache.insert(Obj);
            return Obj;
        }

        template <typename T, typename... ArgTypes>
        inline PtrType Put(ArgTypes&&... Args)
        {
            PtrType NewObj = new T(forward<ArgTypes>(Args)...);
            Cache.insert(NewObj);
            return NewObj;
        }

        inline void GC()
        {
            if (NUMWEAKREFS < 0) {
                return;
            }
            vector<typename SetType::iterator> ToDelete;
            do {
                for(auto& Obj : ToDelete) {
                    Cache.erase(Obj);
                }
                ToDelete.clear();
                for (auto it = Cache.begin(); it != Cache.end(); ++it) {
                    if (((*it)->GetRefCnt_() - NUMWEAKREFS) <= (i64)1) {
                        ToDelete.push_back(it);
                    }
                }
            } while (ToDelete.size() > 0);
        }

        inline void Clear()
        {
            Cache.clear();
        }
    };

} /* end namespace ESMC */

#endif /* ESMC_REF_CACHE_HPP_ */

//
// RefCache.hpp ends here
