PROJECT_NAME=esmc
PROJECT_ROOT=$(realpath .)
include $(PROJECT_ROOT)/Makefile.inc

SRC_DIR=$(PROJECT_ROOT)/src
OBJ_DIR=$(PROJECT_ROOT)/obj/$(BUILD_SUFFIX)

PROJECT_MODULES= \
	exprmgr \
	common \
	containers \
	ast \
	utils \


PROJECT_SOURCES= \
	Z3Semanticizer.cpp \


VPATH=$(addsuffix /:, $(addprefix $(SRC_DIR)/, $(PROJECT_MODULES)))

PROJECT_OBJS=$(addprefix $(PROJECT_ROOT)/obj/$(BUILD_SUFFIX)/, $(PROJECT_SOURCES:.cpp=.o))
PROJECT_DEPS=$(addprefix $(PROJECT_ROOT)/obj/$(BUILD_SUFFIX)/, $(PROJECT_SOURCES:.cpp=.d))


default:			debug
debug:				all
opt:				all
optlto:				all
prof:				all
proflto:			all
eprof:				all
eproflto:			all
eopt:				all
eoptlto:			all
fullto:				all
fullltoprof:		all

all:				$(PROJECT_OBJS)

$(PROJECT_OBJS): $(PROJECT_DEPS)

# esolverlib:			$(ESOLVER_LIB_STATIC) $(ESOLVER_LIB_DYNAMIC) $(SYNTHLIB2_PARSER_LIB_PH)

# esolver-synthlib:   $(ESOLVER_MAIN)

# $(ESOLVER_MAIN):    $(ESOLVER_MAIN_DEPS) $(ESOLVER_MAIN_OBJS) \
# 					$(ESOLVER_LIB_STATIC) $(ESOLVER_LIB_DYNAMIC) \
# 					$(SYNTHLIB2_PARSER_LIB_PH)  $(Z3_BUILD_PH)
# ifeq "x$(VERBOSE_BUILD)" "x"
# 	@echo "$(LDPRINTNAME) `basename $@`"; \
# 	$(LD) $(OPTFLAGS) $(ESOLVER_MAIN_OBJS) $(LINKFLAGS) -Wl,-Bstatic -lsynthlib2parser \
# 	-Wl,-Bdynamic -lboost_program_options -o $@
# else
# 	$(LD) $(OPTFLAGS) $(ESOLVER_MAIN_OBJS) $(LINKFLAGS) -Wl,-Bstatic -lsynthlib2parser \
# 	-Wl,-Bdynamic -lboost_program_options -o $@
# endif

# $(ESOLVER_LIB_DYNAMIC):		$(ESOLVER_LIB_DEPS) $(ESOLVER_LIB_OBJS)
# ifeq "x$(VERBOSE_BUILD)" "x"
# 	@echo "$(LDPRINTNAME) `basename $@`"; \
# 	$(LD) $(OPTFLAGS) -shared -o $@ $(ESOLVER_LIB_OBJS)
# else
# 	$(LD) $(OPTFLAGS) -shared -o $@ $(ESOLVER_LIB_OBJS)
# endif

# $(ESOLVER_LIB_STATIC):		$(ESOLVER_LIB_DEPS) $(ESOLVER_LIB_OBJS)
# ifeq "x$(VERBOSE_BUILD)" "x"
# 	@echo "$(ARPRINTNAME) `basename $@`"; \
# 	$(AR) $(ARFLAGS) $@ $(ESOLVER_LIB_OBJS)
# else 
# 	$(AR) $(ARFLAGS) $@ $(ESOLVER_LIB_OBJS)
# endif

$(OBJ_DIR)/%.d:		%.cpp
ifeq "x$(VERBOSE_BUILD)" "x"
	@set -e; rm -f $@; \
	echo "$(DEPPRINTNAME) `basename $<`"; \
	$(CXX) -MM $(CXXFLAGS) $< > $@.$$$$; \
    sed 's,\($*\)\.o[ :]*,$(OBJ_DIR)/\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$
else 
	@set -e; rm -f $@; \
	echo "Calculating dependencies for `basename $<`..."; \
	$(CXX) -MM $(CXXFLAGS) $< > $@.$$$$; \
    sed 's,\($*\)\.o[ :]*,$(OBJ_DIR)/\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$
endif

$(OBJ_DIR)/%.o:		%.cpp
ifeq "x$(VERBOSE_BUILD)" "x"
	@echo "$(CXXPRINTNAME) `basename $<` --> `basename $@`"; \
	$(CXX) $(CXXFLAGS) $(OPTFLAGS) -c $< -o $@
else
	$(CXX) $(CXXFLAGS) $(OPTFLAGS) -c $< -o $@
endif

# $(SYNTHLIB2_PARSER_LIB_PH):
# 	$(MAKE) $(MAKECMDGOALS) -C $(ESOLVER_ROOT)/src/synthlib2parser
# 	cp $(ESOLVER_ROOT)/src/synthlib2parser/lib/$(BUILD_SUFFIX)/libsynthlib2parser.so \
# 	   $(ESOLVER_ROOT)/src/synthlib2parser/lib/$(BUILD_SUFFIX)/libsynthlib2parser.a \
# 	   $(ESOLVER_ROOT)/lib/$(BUILD_SUFFIX)
# 	touch $(SYNTHLIB2_PARSER_LIB_PH)

# $(Z3_BUILD_PH):
# 	cd $(ESOLVER_ROOT)/src/z3-4.3.1; \
# 	autoconf; \
# 	./configure; \
# 	python scripts/mk_make.py; \
# 	cd build; \
# 	$(MAKE); \
# 	touch $@

.PHONY:	clean distclean

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

distclean:
	# rm -rf obj/debug/*
	# rm -rf obj/opt/*
	# rm -rf obj/prof/*
	# rm -rf lib/debug/*
	# rm -rf lib/opt/*
	# rm -rf lib/prof/*
	# rm -rf bin/debug/*
	# rm -rf bin/opt/*
	# rm -rf bin/prof/*
	# $(MAKE) $(MAKECMDGOALS) -C $(ESOLVER_ROOT)/src/synthlib2parser
	# cd $(Z3DIR); ./clean.sh

ifneq ($(MAKECMDGOALS), clean)
ifneq ($(MAKECMDGOALS), distclean)
-include $(PROJECT_DEPS)
endif
endif
