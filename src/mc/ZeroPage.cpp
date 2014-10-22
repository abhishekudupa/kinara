// ZeroPage.cpp --- 
// 
// Filename: ZeroPage.cpp
// Author: Abhishek Udupa
// Created: Mon Aug 18 23:12:58 2014 (-0400)
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

#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>

#include "ZeroPage.hpp"

namespace ESMC {
    namespace MC {

        u08* ZeroPage::ThePage = nullptr;
        const u32 ZeroPage::PageSize = (1 << 20);

        ZeroPage::ZeroPage()
        {
            // Nothing here
        }

        ZeroPage& ZeroPage::operator = (const ZeroPage& Other)
        {
            return *this;
        }

        u08* ZeroPage::Get()
        {
            if (ThePage != nullptr) {
                return ThePage;
            }
#ifdef __APPLE__
            return (u08*) calloc(100000, sizeof(u08));
#endif
            int fd = open("/dev/zero", O_RDONLY);
            ThePage = (u08*)mmap(nullptr, PageSize, PROT_READ, MAP_PRIVATE, fd, 0);
            close(fd);
            atexit(ZeroPage::Fin);
            return ThePage;
        }

        void ZeroPage::Fin()
        {
            if (ThePage != nullptr) {
                munmap(ThePage, PageSize);
            }
        }

    } /* end namespace LTS */
} /* end namespace ESMC */

// 
// ZeroPage.cpp ends here
