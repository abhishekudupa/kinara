PROJECT_NAME=esmc
PROJECT_ROOT=$(realpath .)
BASE_OBJ_DIR=$(PROJECT_ROOT)/obj
BASE_LIB_DIR=$(PROJECT_ROOT)/lib
BASE_BIN_DIR=$(PROJECT_ROOT)/bin
BASE_GEN_DIR=$(PROJECT_ROOT)/gen
BASE_SRC_DIR=$(PROJECT_ROOT)/src

CXX=g++

CXXFLAGS=-std=c++11 -Wall -pedantic -fopenmp
CXXFLAGS+=-I $(PROJECT_ROOT)/src/extern/z3-4.3.1/src/api
CXXFLAGS+=-I $(PROJECT_ROOT)/src/extern/boost-local/boost_install/include

ifeq "x$(CXX)" "xg++"
CXXFLAGS+=-Wno-unused-local-typedefs
endif

PROJECT_MODULES= \
	expr \
	common \
	containers \
	uflts \
	ast \
	utils \
	main \


PROJECT_EXECUTABLES=esmc
esmc_SYS_LIBS=z3 rt
esmc_DEP_LIBS=esmc
esmc_EXT_LIBS=boost z3
libboost_FULL_PATH=$(PROJECT_ROOT)/src/extern/boost-local/boost_install/install.ph
libboost_MAKE_DIR=$(PROJECT_ROOT)/src/extern/boost-local/
libz3_FULL_PATH=$(PROJECT_ROOT)/src/extern/z3-4.3.1/install/libz3.so
libz3_MAKE_DIR=$(PROJECT_ROOT)/src/extern/z3-4.3.1/
esmc_OBJS=main.o
esmc_LIB_PATHS= \
	$(PROJECT_ROOT)/src/extern/z3-4.3.1/install \
	$(PROJECT_ROOT)/src/extern/boost-local/boost_install/lib \
	$(PROJECT_ROOT)/lib/$(BUILD_SUFFIX)


PROJECT_LIBS=esmc
libesmc_OBJS= \
	LTSTypes.o \
	SemanticizerUtils.o \
	UIDGenerator.o \
	Z3Semanticizer.o \


PROJECT_EXT_LIBS=boost z3

libesmc_EXT_LIBS= \
	boost \
	z3 \

include $(PROJECT_ROOT)/Makefile.util

