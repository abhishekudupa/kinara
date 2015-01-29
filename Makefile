PROJECT_NAME=esmc
PROJECT_ROOT=$(realpath .)
BASE_OBJ_DIR=$(PROJECT_ROOT)/obj
BASE_LIB_DIR=$(PROJECT_ROOT)/lib
BASE_BIN_DIR=$(PROJECT_ROOT)/bin
BASE_GEN_DIR=$(PROJECT_ROOT)/gen
BASE_SRC_DIR=$(PROJECT_ROOT)/src

Z3_VERSION_TO_USE?=4.3.2

CXX?=g++

CXXFLAGS+=-std=c++11 -Wall -pedantic -pipe
CXXFLAGS+=-I $(PROJECT_ROOT)/thirdparty/z3-$(Z3_VERSION_TO_USE)/src/api
CXXFLAGS+=-I $(PROJECT_ROOT)/thirdparty/boost-local/boost_install/include
CXXFLAGS+=-I $(PROJECT_ROOT)/thirdparty/sparsehash

ifneq "x$(Z3_VERSION_TO_USE)" "x4.3.1"
CXXFLAGS+=-DUSE_Z3_4_3_2
endif

ifeq "x$(CXX)" "xg++"
CXXFLAGS+=-Wno-unused-local-typedefs -Wno-overflow -fopenmp
else
CXXFLAGS+=-Wno-gnu-folding-constant -openmp
endif

PROJECT_MODULES= \
	common \
	containers \
	decls \
	expr \
	hash \
	lib \
	main \
	mc \
	symexec \
	symmetry \
	synth \
	tpinterface \
	uflts \
	utils \

libboost_FULL_PATH=$(PROJECT_ROOT)/thirdparty/boost-local/boost_install/install.ph
libboost_MAKE_DIR=$(PROJECT_ROOT)/thirdparty/boost-local/
libz3_FULL_PATH=$(PROJECT_ROOT)/thirdparty/z3-$(Z3_VERSION_TO_USE)/install/libz3.so
libz3_MAKE_DIR=$(PROJECT_ROOT)/thirdparty/z3-$(Z3_VERSION_TO_USE)/

PROJECT_LIBS=esmc

libesmc_OBJS= \
	AQStructure.o \
	Compiler.o \
	ESMCLib.o \
	IndexSet.o \
	LabelledTS.o \
	LogManager.o \
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
	LTSTransformers.o \
	LTSTransitions.o \
	OmegaAutomaton.o \
	Permutations.o \
	ResourceLimitManager.o \
	Solver.o \
	SpookyHash.o \
	StateVec.o \
	StateVecPrinter.o \
	SymbolTable.o \
	SymmCanonicalizer.o \
	TheoremProver.o \
	TimeValue.o \
	Trace.o \
	UIDGenerator.o \
	Z3Objects.o \


PROJECT_EXT_LIBS=boost z3

libesmc_EXT_LIBS= \
	boost \
	z3 \


include $(PROJECT_ROOT)/Makefile.util
