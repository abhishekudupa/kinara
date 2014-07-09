PROJECT_NAME=esmc
PROJECT_ROOT=$(realpath .)
BASE_OBJ_DIR=$(PROJECT_ROOT)/obj
BASE_LIB_DIR=$(PROJECT_ROOT)/lib
BASE_BIN_DIR=$(PROJECT_ROOT)/bin
BASE_GEN_DIR=$(PROJECT_ROOT)/gen
BASE_SRC_DIR=$(PROJECT_ROOT)/src

CXXFLAGS=-std=c++11 -Wall -pedantic
CXXFLAGS+=-I $(PROJECT_ROOT)/src/extern/z3-4.3.1/src/api
CXXFLAGS+=-I $(PROJECT_ROOT)/src/extern/boost-local/boost_install/include
CXXFLAGS+=-Wno-unused-local-typedefs

PROJECT_MODULES= \
	exprmgr \
	common \
	containers \
	ast \
	utils \
	main \


PROJECT_EXECUTABLES=esmc
esmc_SYS_LIBS=z3 rt
esmc_DEP_LIBS=esmc
esmc_OBJS=main.o
esmc_LIB_PATHS= \
	$(PROJECT_ROOT)/src/extern/z3-4.3.1/install \
	$(PROJECT_ROOT)/src/extern/boost-local/boost_install/lib \
	$(PROJECT_ROOT)/lib/$(BUILD_SUFFIX)


PROJECT_LIBS=esmc
libesmc_OBJS= \
	Z3Semanticizer.o \


include $(PROJECT_ROOT)/Makefile.util

clean:
	rm -rf obj/debug/*
	rm -rf obj/opt/*
	rm -rf obj/prof/*
	rm -rf lib/debug/*
	rm -rf lib/opt/*
	rm -rf lib/prof/*
	rm -rf bin/debug/*
	rm -rf bin/opt/*
	rm -rf bin/prof/*

ifneq ($(MAKECMDGOALS), clean)
ifneq ($(MAKECMDGOALS), distclean)
-include $(PROJECT_DEPS)
endif
endif
