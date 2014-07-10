OPTFLAGS=-ggdb3 -O0 -fno-inline
#OPTFLAGS=-O3 -flto

CXX=g++

ifeq "x$(CXX)" "xg++"
CXXFLAGS=-Wno-unused-local-typedefs
else
CXXFLAGS=
endif

CXXFLAGS+=-std=c++11 -fopenmp
CXXFLAGS+=-I ../../src/exprmgr/ -I ../../src/extern/boost-local/boost_install/include
CXXFLAGS+=-I ../../src/extern/z3-4.3.1/src/api -fopenmp -Wall -pedantic -fopenmp

LDFLAGS=-L ../../src/extern/z3-4.3.1/install/
LDFLAGS+=-L ../../src/extern/boost-local/boost_install/lib/ 
LDFLAGS+=-L ../../lib/debug/ 

LDFLAGS+=-Wl,-rpath $(realpath ../../lib/debug)
LDFLAGS+=-Wl,-rpath $(realpath ../../src/extern/boost-local/boost_install/lib)
LDFLAGS+=-Wl,-rpath $(realpath ../../src/extern/z3-4.3.1/install/)

LINKFLAGS=$(LDFLAGS)
LINKFLAGS+=-lesmc -lz3 

CXXFLAGS+=$(OPTFLAGS)

TESTS=\
	ExprTests \


default:	all

all: $(TESTS)

ExprTests: ExprTests.cpp
	$(CXX) $(CXXFLAGS) ExprTests.cpp -o ExprTests $(LINKFLAGS) 

clean:
	rm -rf $(TESTS) *.o

