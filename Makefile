PROJECT_NAME=esmc
PROJECT_ROOT=$(realpath .)
BASE_OBJ_DIR=$(PROJECT_ROOT)/obj
BASE_LIB_DIR=$(PROJECT_ROOT)/obj
BASE_BIN_DIR=$(PROJECT_ROOT)/obj
BASE_BIN_DIR=$(PROJECT_ROOT)/gen
BASE_SRC_DIR=$(PROJECT_ROOT)/src

CXXFLAGS=-I $(PROJECT_ROOT)/src/extern/z3-4.3.1/src/api/ -std=c++11 -Wall -pedantic
CXXFLAGS+=-Wno-unused-local-typedefs

PROJECT_MODULES= \
	exprmgr \
	common \
	containers \
	ast \
	utils \

PROJECT_SOURCES= \
	Z3Semanticizer.cpp \

include $(PROJECT_ROOT)/Makefile.util

all:	$(PROJECT_OBJS)

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
