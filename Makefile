PROJECT_NAME=esmc
PROJECT_ROOT=$(realpath .)
BASE_OBJ_DIR=$(PROJECT_ROOT)/obj
BASE_LIB_DIR=$(PROJECT_ROOT)/lib
BASE_BIN_DIR=$(PROJECT_ROOT)/bin
BASE_GEN_DIR=$(PROJECT_ROOT)/gen
BASE_SRC_DIR=$(PROJECT_ROOT)/src

CXX?=g++

CXXFLAGS=-std=c++11 -Wall -pedantic -fopenmp -pipe
CXXFLAGS+=-I $(PROJECT_ROOT)/thirdparty/z3-4.3.1/src/api
CXXFLAGS+=-I $(PROJECT_ROOT)/thirdparty/boost-local/boost_install/include
CXXFLAGS+=-I $(PROJECT_ROOT)/thirdparty/sparsehash

ifeq "x$(CXX)" "xg++"
CXXFLAGS+=-Wno-unused-local-typedefs
else
CXXFLAGS+=-Wno-gnu-folding-constant
endif

PROJECT_MODULES= \
	common \
	containers \
	expr \
	hash \
	main \
	mc \
	symexec \
	symmetry \
	synth \
	tpinterface \
	uflts \
	utils \


PROJECT_EXECUTABLES=esmc
esmc_LINK_LIBS_COMMON=z3 rt boost_system
esmc_LINK_LIBS_LTO=esmc

esmc_DEP_LIBS=esmc
esmc_EXT_LIBS=boost z3

esmc_LIB_PATHS= \
	$(PROJECT_ROOT)/thirdparty/z3-4.3.1/install \
	$(PROJECT_ROOT)/thirdparty/boost-local/boost_install/lib \
	$(PROJECT_ROOT)/lib/$(BUILD_SUFFIX)

esmc_OBJS=main.o

libboost_FULL_PATH=$(PROJECT_ROOT)/thirdparty/boost-local/boost_install/install.ph
libboost_MAKE_DIR=$(PROJECT_ROOT)/thirdparty/boost-local/
libz3_FULL_PATH=$(PROJECT_ROOT)/thirdparty/z3-4.3.1/install/libz3.so
libz3_MAKE_DIR=$(PROJECT_ROOT)/thirdparty/z3-4.3.1/

PROJECT_LIBS=esmc

libesmc_OBJS= \
	AQStructure.o \
	Compiler.o \
	IndexSet.o \
	LabelledTS.o \
	LTSAnalyses.o \
	LTSAssign.o \
	LTSAutomaton.o \
	LTSChannelEFSM.o \
	LTSChecker.o \
	LTSEFSM.o \
	LTSEFSMBase.o \
	LTSExtensions.o \
	LTSFairnessSet.o \
	LTSSemTypes.o \
	LTSState.o \
	LTSTermSemanticizer.o \
	LTSTransitions.o \
	LTSUtils.o \
	OmegaAutomaton.o \
	Permutations.o \
	Solver.o \
	SpookyHash.o \
	StateVec.o \
	StateVecPrinter.o \
	SymbolTable.o \
	SymmCanonicalizer.o \
	TheoremProver.o \
	Trace.o \
	UIDGenerator.o \
	Z3Objects.o \


PROJECT_EXT_LIBS=boost z3

libesmc_EXT_LIBS= \
	boost \
	z3 \


include $(PROJECT_ROOT)/Makefile.util
