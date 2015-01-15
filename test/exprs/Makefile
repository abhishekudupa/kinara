OPTFLAGS=-ggdb3 -O0 -fno-inline
# OPTFLAGS=-O3 -flto

CXX=g++

ifeq "x$(CXX)" "xg++"
CXXFLAGS=-Wno-unused-local-typedefs
else
CXXFLAGS=
endif

CXXFLAGS+=-std=c++11 -fopenmp
CXXFLAGS+=-I ../../thirdparty/boost-local/boost_install/include
CXXFLAGS+=-I ../../thirdparty/z3/src/api -fopenmp -Wall -pedantic -fopenmp

LDFLAGS=-L ../../thirdparty/z3/install/
LDFLAGS+=-L ../../thirdparty/boost-local/boost_install/lib/
LDFLAGS+=-L ../../lib/debug

LDFLAGS+=-Wl,-rpath $(realpath ../../lib/debug)
LDFLAGS+=-Wl,-rpath $(realpath ../../thirdparty/boost-local/boost_install/lib)
LDFLAGS+=-Wl,-rpath $(realpath ../../thirdparty/z3/install/)

LINKFLAGS=$(LDFLAGS)
LINKFLAGS+=-lesmc -lz3

CXXFLAGS+=$(OPTFLAGS)

TESTS=\
	SimpTests \


default:	all

all: $(TESTS)

ExprTests: ExprTests.cpp
	$(CXX) $(CXXFLAGS) ExprTests.cpp -o ExprTests $(LINKFLAGS)

SimpTests: SimpTests.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(LINKFLAGS)

clean:
	rm -rf $(TESTS) *.o
