PROJECT_NAME=ltstests
PROJECT_ROOT=$(realpath .)
BASE_OBJ_DIR=$(PROJECT_ROOT)/obj
BASE_LIB_DIR=$(PROJECT_ROOT)/lib
BASE_BIN_DIR=$(PROJECT_ROOT)/bin
BASE_GEN_DIR=$(PROJECT_ROOT)
BASE_SRC_DIR=$(PROJECT_ROOT)

CXX?=g++

ifeq "x$(CXX)" "xg++"
CXXFLAGS+=-Wno-unused-local-typedefs
endif

CXXFLAGS+=-std=c++11 -Wall -pedantic -fopenmp -pipe
CXXFLAGS+=-I $(PROJECT_ROOT)/../../thirdparty/z3-4.3.1/src/api
CXXFLAGS+=-I $(PROJECT_ROOT)/../../thirdparty/boost-local/boost_install/include
CXXFLAGS+=-I $(PROJECT_ROOT)/../../thirdparty/sparsehash

PROJECT_EXECUTABLES=PingPong TestChan SemTest SemTestLower

PingPong_SYS_LIBS=esmc z3 rt boost_system
PingPong_EXT_LIBS=boost z3 esmc

TestChan_SYS_LIBS=esmc z3 rt boost_system
TestChan_EXT_LIBS=boost z3 esmc

SemTest_SYS_LIBS=esmc z3 rt boost_system
SemTest_EXT_LIBS=boost z3 esmc

SemTestLower_SYS_LIBS=$(SemTest_SYS_LIBS)
SemTestLower_EXT_LIBS=$(SemTest_EXT_LIBS)

libboost_FULL_PATH=$(PROJECT_ROOT)/../../src/extern/boost-local/boost_install/install.ph
libboost_MAKE_DIR=$(PROJECT_ROOT)/../../src/extern/boost-local/
libz3_FULL_PATH=$(PROJECT_ROOT)/../../src/extern/z3-4.3.1/install/libz3.so
libz3_MAKE_DIR=$(PROJECT_ROOT)/../../src/extern/z3-4.3.1/
libesmc_FULL_PATH=$(PROJECT_ROOT)/../../lib/$(BUILD_SUFFIX)/libesmc.so
libesmc_MAKE_DIR=$(PROJECT_ROOT)/../../

PingPong_OBJS=PingPong.o
TestChan_OBJS=TestChan.o
SemTest_OBJS=SemTest.o
SemTestLower_OBJS=SemTestLower.o

PingPong_LIB_PATHS= \
    $(PROJECT_ROOT)/../../thirdparty/z3-4.3.1/install \
    $(PROJECT_ROOT)/../../thirdparty/boost-local/boost_install/lib \
    $(PROJECT_ROOT)/../../lib/$(BUILD_SUFFIX)

TestChan_LIB_PATHS=$(PingPong_LIB_PATHS)
SemTest_LIB_PATHS=$(PingPong_LIB_PATHS)
SemTestLower_LIB_PATHS=$(SemTest_LIB_PATHS)


include $(PROJECT_ROOT)/../../Makefile.util
