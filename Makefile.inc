
Z3DIR=$(PROJECT_ROOT)/src/extern/z3-4.3.1
Z3LIBLINK=-lz3
Z3BUILDDIR=$(Z3DIR)/build

CXXFLAGS+=-Wall -pedantic -D__STDC_LIMIT_MACROS -DDISABLE_STATIC_BP -Wno-unused-local-typedefs
CXXFLAGS+=-I $(Z3DIR)/src/api/ -I $(PROJECT_ROOT)/src/extern/boost-local/boost_install/include/
CXXFLAGS+=-pipe -std=c++11
LINKFLAGS+=-L $(Z3BUILDDIR) -L $(PROJECT_ROOT)/lib/$(BUILD_SUFFIX)
LINKFLAGS+=-Wl,-Bstatic -lesolver -Wl,-Bstatic $(Z3LIBLINK) -Wl,-Bdynamic -lrt
LINKFLAGS+=-lboost_program_options -lboost_system -fopenmp
CXXFLAGS+=$(FEAT_FLAGS)
AR=ar
ARFLAGS=rcs

CXXPRINTNAME="[CXX:$(PROJECT_NAME)]"
ARPRINTNAME="[AR:$(PROJECT_NAME)]"
LDPRINTNAME="[LD:$(PROJECT_NAME)]"
DEPPRINTNAME="[DEP:$(PROJECT_NAME)]"
.PHONY:				debug clean opt prof all default eprof eopt optlto proflto eoptlto eproflto
