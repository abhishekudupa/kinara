PROJECT_NAME=mctests
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

PROJECT_EXECUTABLES=PingPongChan Elevator Flip FlipSimple PingPongChanBug ABP \
	ABP_generated MSI Fischer LeaderElection Dijkstra4 MSI_no_busy_data_no_pending_unblock_e \
	MSIC2D PingPongChanSynth MSISynth ElevatorIncomplete Dijkstra4Synth ElevatorIncomplete2
#	WeakestPreconditionTest
#	WeakestPreconditionArrayTest
#   ElevatorBug

libboost_FULL_PATH=$(PROJECT_ROOT)/../../thirdparty/boost-local/boost_install/install.ph
libboost_MAKE_DIR=$(PROJECT_ROOT)/../../thirdparty/boost-local/
libz3_FULL_PATH=$(PROJECT_ROOT)/../../thirdparty/z3-4.3.1/install/libz3.so
libz3_MAKE_DIR=$(PROJECT_ROOT)/../../thirdparty/z3-4.3.1/
libesmc_FULL_PATH=$(PROJECT_ROOT)/../../lib/$(BUILD_SUFFIX)/libesmc.so
libesmc_MAKE_DIR=$(PROJECT_ROOT)/../../

$(foreach exec, $(PROJECT_EXECUTABLES), $(eval $(exec)_SYS_LIBS=esmc z3 boost_system rt))
$(foreach exec, $(PROJECT_EXECUTABLES), $(eval $(exec)_EXT_LIBS=boost z3 esmc))
$(foreach exec, $(PROJECT_EXECUTABLES), $(eval $(exec)_OBJS=$(exec).o))
$(foreach exec, $(PROJECT_EXECUTABLES), $(eval $(exec)_LIB_PATHS=$(PROJECT_ROOT)/../../lib/$(BUILD_SUFFIX) \
	$(PROJECT_ROOT)/../../thirdparty/z3-4.3.1/install \
	$(PROJECT_ROOT)/../../thirdparty/boost-local/boost_install/lib))


include $(PROJECT_ROOT)/../../Makefile.util
