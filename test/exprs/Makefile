OPTFLAGS=-ggdb3 -O0 -fno-inline

CXXFLAGS=-I ../../src/exprmgr/ -I ../../src/external/boost-local/boost_install/include
CXXFLAGS+=-I ../../src/external/z3-4.3.1/src/api -fopenmp -Wall -pedantic -fopenmp

LDFLAGS=-L ../../src/external/z3-4.3.1/install/
LDFLAGS+=-L ../../src/external/boost-local/boost_install/lib/
LDFLAGS+=-L ../../bin/debug/

LINKFLAGS=$(LDFLAGS)
LINKFLAGS+=-Wl,-Bstatic -lesmc -Wl,-Bdynamic -lz3 -lrt

CXXFLAGS+=$(OPTFLAGS)

default:	all

all: ExprTests

ExprTests: ExprTests.cpp
	$(CXX) 
