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
	ABP_generated MSI MSI_generated Fischer LeaderElection Dijkstra4 MSI_no_busy_data_no_pending_unblock_e \
	MSIC2D PingPongChanSynth \
	WeakestPreconditionTest \
	WeakestPreconditionArrayTest


PingPongChan_SYS_LIBS=esmc z3 rt boost_system
PingPongChan_EXT_LIBS=boost z3 esmc
PingPongChanSynth_SYS_LIBS=$(PingPongChan_SYS_LIBS)
PingPongChanSynth_EXT_LIBS=$(PingPongChan_EXT_LIBS)


PingPongChanBug_SYS_LIBS=$(PingPongChan_SYS_LIBS)
PingPongChanBug_EXT_LIBS=$(PingPongChan_EXT_LIBS)

Elevator_SYS_LIBS=esmc z3 rt boost_system
Elevator_EXT_LIBS=boost z3 esmc

Flip_SYS_LIBS=esmc z3 rt boost_system
Flip_EXT_LIBS=boost z3 esmc

FlipSimple_SYS_LIBS=esmc z3 rt boost_system
FlipSimple_EXT_LIBS=boost z3 esmc

ABP_SYS_LIBS=esmc z3 rt boost_system
ABP_EXT_LIBS=boost z3 esmc

ABP_generated_SYS_LIBS=esmc z3 rt boost_system
ABP_generated_EXT_LIBS=boost z3 esmc

MSI_generated_SYS_LIBS=esmc z3 rt boost_system
MSI_generated_EXT_LIBS=boost z3 esmc

MSI_SYS_LIBS=esmc z3 rt boost_system
MSI_EXT_LIBS=boost z3 esmc

MSIC2D_SYS_LIBS=$(MSI_SYS_LIBS)
MSIC2D_EXT_LIBS=$(MSI_EXT_LIBS)

MSI_no_busy_data_no_pending_unblock_e_SYS_LIBS=esmc z3 rt boost_system
MSI_no_busy_data_no_pending_unblock_e_EXT_LIBS=boost z3 esmc

Fischer_SYS_LIBS=esmc z3 rt boost_system
Fischer_EXT_LIBS=boost z3 esmc

LeaderElection_SYS_LIBS=esmc z3 rt boost_system
LeaderElection_EXT_LIBS=boost z3 esmc

WeakestPreconditionTest_SYS_LIBS=esmc z3 rt boost_system
WeakestPreconditionTest_EXT_LIBS=boost z3 esmc

WeakestPreconditionArrayTest_SYS_LIBS=esmc z3 rt boost_system
WeakestPreconditionArrayTest_EXT_LIBS=boost z3 esmc

Dijkstra4_SYS_LIBS=esmc z3 rt boost_system
Dijkstra4_EXT_LIBS=boost z3 esmc

libboost_FULL_PATH=$(PROJECT_ROOT)/../../thirdparty/boost-local/boost_install/install.ph
libboost_MAKE_DIR=$(PROJECT_ROOT)/../../thirdparty/boost-local/
libz3_FULL_PATH=$(PROJECT_ROOT)/../../thirdparty/z3-4.3.1/install/libz3.so
libz3_MAKE_DIR=$(PROJECT_ROOT)/../../thirdparty/z3-4.3.1/
libesmc_FULL_PATH=$(PROJECT_ROOT)/../../lib/$(BUILD_SUFFIX)/libesmc.so
libesmc_MAKE_DIR=$(PROJECT_ROOT)/../../

PingPongChan_OBJS=PingPongChan.o
PingPongChanBug_OBJS=PingPongChanBug.o
PingPongChanSynth_OBJS=PingPongChanSynth.o
Flip_OBJS=Flip.o
FlipSimple_OBJS=FlipSimple.o
Elevator_OBJS=Elevator.o
ABP_OBJS=ABP.o
ABP_generated_OBJS=ABP_generated.o
MSI_generated_OBJS=MSI_generated.o
Fischer_OBJS=Fischer.o
LeaderElection_OBJS=LeaderElection.o
MSI_OBJS=MSI.o
MSIC2D_OBJS=MSIC2D.o
MSI_no_busy_data_no_pending_unblock_e_OBJS=MSI_no_busy_data_no_pending_unblock_e.o
Dijkstra4_OBJS=Dijkstra4.o
WeakestPreconditionTest_OBSJ=WeakestPreconditionTest.o
WeakestPreconditionArrayTest_OBSJ=WeakestPreconditionArrayTest.o


PingPongChan_LIB_PATHS= \
    $(PROJECT_ROOT)/../../thirdparty/z3-4.3.1/install \
    $(PROJECT_ROOT)/../../thirdparty/boost-local/boost_install/lib \
    $(PROJECT_ROOT)/../../lib/$(BUILD_SUFFIX)

PingPongChanBug_LIB_PATHS=$(PingPongChan_LIB_PATHS)
PingPongChanSynth_LIB_PATHS=$(PingPongChan_LIB_PATHS)

Elevator_LIB_PATHS= \
    $(PROJECT_ROOT)/../../thirdparty/z3-4.3.1/install \
    $(PROJECT_ROOT)/../../thirdparty/boost-local/boost_install/lib \
    $(PROJECT_ROOT)/../../lib/$(BUILD_SUFFIX)

ABP_LIB_PATHS= \
    $(PROJECT_ROOT)/../../thirdparty/z3-4.3.1/install \
    $(PROJECT_ROOT)/../../thirdparty/boost-local/boost_install/lib \
    $(PROJECT_ROOT)/../../lib/$(BUILD_SUFFIX)

ABP_generated_LIB_PATHS= \
    $(PROJECT_ROOT)/../../thirdparty/z3-4.3.1/install \
    $(PROJECT_ROOT)/../../thirdparty/boost-local/boost_install/lib \
    $(PROJECT_ROOT)/../../lib/$(BUILD_SUFFIX)

MSI_generated_LIB_PATHS= \
    $(PROJECT_ROOT)/../../thirdparty/z3-4.3.1/install \
    $(PROJECT_ROOT)/../../thirdparty/boost-local/boost_install/lib \
    $(PROJECT_ROOT)/../../lib/$(BUILD_SUFFIX)

MSI_LIB_PATHS=$(MSI_generated_LIB_PATHS)
MSIC2D_LIB_PATHS=$(MSI_LIB_PATHS)

MSI_no_busy_data_no_pending_unblock_e_LIB_PATHS= \
    $(PROJECT_ROOT)/../../thirdparty/z3-4.3.1/install \
    $(PROJECT_ROOT)/../../thirdparty/boost-local/boost_install/lib \
    $(PROJECT_ROOT)/../../lib/$(BUILD_SUFFIX)

Fischer_LIB_PATHS= \
    $(PROJECT_ROOT)/../../thirdparty/z3-4.3.1/install \
    $(PROJECT_ROOT)/../../thirdparty/boost-local/boost_install/lib \
    $(PROJECT_ROOT)/../../lib/$(BUILD_SUFFIX)

LeaderElection_LIB_PATHS= \
    $(PROJECT_ROOT)/../../thirdparty/z3-4.3.1/install \
    $(PROJECT_ROOT)/../../thirdparty/boost-local/boost_install/lib \
    $(PROJECT_ROOT)/../../lib/$(BUILD_SUFFIX)

Flip_LIB_PATHS= \
    $(PROJECT_ROOT)/../../thirdparty/z3-4.3.1/install \
    $(PROJECT_ROOT)/../../thirdparty/boost-local/boost_install/lib \
    $(PROJECT_ROOT)/../../lib/$(BUILD_SUFFIX)

FlipSimple_LIB_PATHS= \
    $(PROJECT_ROOT)/../../thirdparty/z3-4.3.1/install \
    $(PROJECT_ROOT)/../../thirdparty/boost-local/boost_install/lib \
    $(PROJECT_ROOT)/../../lib/$(BUILD_SUFFIX)

Dijkstra4_LIB_PATHS= \
    $(PROJECT_ROOT)/../../thirdparty/z3-4.3.1/install \
    $(PROJECT_ROOT)/../../thirdparty/boost-local/boost_install/lib \
    $(PROJECT_ROOT)/../../lib/$(BUILD_SUFFIX)

WeakestPreconditionTest_LIB_PATHS==$(Dijkstra4_LIB_PATHS)

WeakestPreconditionArrayTest_LIB_PATHS==$(WeakestPreconditionTest_LIB_PATHS)


include $(PROJECT_ROOT)/../../Makefile.util
