PROJECT_NAME=mctests
PROJECT_ROOT=$(realpath .)
BASE_OBJ_DIR=$(PROJECT_ROOT)/obj
BASE_LIB_DIR=$(PROJECT_ROOT)/lib
BASE_BIN_DIR=$(PROJECT_ROOT)/bin
BASE_GEN_DIR=$(PROJECT_ROOT)
BASE_SRC_DIR=$(PROJECT_ROOT)

Z3_VERSION_TO_USE?=4.3.2

CXX?=g++

ifeq "x$(CXX)" "xg++"
CXXFLAGS+=-Wno-unused-local-typedefs
endif

CXXFLAGS+=-std=c++11 -Wall -pedantic -fopenmp -pipe
CXXFLAGS+=-I $(PROJECT_ROOT)/../../thirdparty/z3-$(Z3_VERSION_TO_USE)/src/api
CXXFLAGS+=-I $(PROJECT_ROOT)/../../thirdparty/boost-local/boost_install/include
CXXFLAGS+=-I $(PROJECT_ROOT)/../../thirdparty/sparsehash

PROJECT_EXECUTABLES?=PingPongChan Flip FlipSimple PingPongChanBug ABP \
	ABP_generated MSI Fischer Dijkstra4 \
	MSIC2D PingPongChanSimpleBug PingPongChanSynth MSIC2DSynth ElevatorIncomplete Dijkstra4Synth \


libboost_FULL_PATH=$(PROJECT_ROOT)/../../thirdparty/boost-local/boost_install/install.ph
libboost_MAKE_DIR=$(PROJECT_ROOT)/../../thirdparty/boost-local/
libz3_FULL_PATH=$(PROJECT_ROOT)/../../thirdparty/z3-$(Z3_VERSION_TO_USE)/install/libz3.so
libz3_MAKE_DIR=$(PROJECT_ROOT)/../../thirdparty/z3-$(Z3_VERSION_TO_USE)/
libesmc_FULL_PATH=$(PROJECT_ROOT)/../../lib/$(BUILD_SUFFIX)/libesmc.so
libesmc_MAKE_DIR=$(PROJECT_ROOT)/../../

$(foreach exec, $(PROJECT_EXECUTABLES), $(eval $(exec)_LINK_LIBS_COMMON=z3 rt boost_system boost_program_options boost_iostreams boost_bzip2 z))
$(foreach exec, $(PROJECT_EXECUTABLES), $(eval $(exec)_LINK_LIBS_LTO=esmc))
$(foreach exec, $(PROJECT_EXECUTABLES), $(eval $(exec)_OBJS=$(exec).o))
$(foreach exec, $(PROJECT_EXECUTABLES), $(eval $(exec)_LIB_PATHS=$(PROJECT_ROOT)/../../lib/$(BUILD_SUFFIX) \
	$(PROJECT_ROOT)/../../thirdparty/z3-$(Z3_VERSION_TO_USE)/install \
	$(PROJECT_ROOT)/../../thirdparty/boost-local/boost_install/lib))


include $(PROJECT_ROOT)/../../Makefile.util
