###
#  Copyright (C) 2018 - Innovusion Inc.
#
#  All Rights Reserved.
#
#  $Id$
##

STATIC_LIB = libinnolidar.a
DYNA_LIB = libinnolidar.so
LIB_DIR = ../lib
STATIC_LIB_DIR = ../lib/static
TARGETS = example1_static example1_dynamic example2_static example2_dynamic

CPPLINT = ../build/cpplint.py
CC ?= gcc
CXX ?= g++
CFLAGS ?= -fPIC -O2 -g2 -Wall -Werror -fpermissive
LINKFLAGS ?= -pthread -ldl #-Wl,-Bstatic -static
DYNA_LINKFLAGS ?= -pthread -ldl
INC_DIR = -I../src

.PHONY: build
build: lint $(TARGETS)

ifeq ($(CUSTOMER_RELEASE),OFF)
example1_static: example1.cpp $(STATIC_LIB_DIR)/$(STATIC_LIB)
	$(CXX) $(INC_DIR) $(CFLAGS) -o $@ $< -L $(STATIC_LIB_DIR) -Bstatic -linnolidar -Bdynamic $(LINKFLAGS)
else
.PHONY: example1_static
example1_static:
endif

example1_dynamic: example1.cpp $(LIB_DIR)/$(DYNA_LIB)
	$(CXX) $(INC_DIR) $(CFLAGS) -o $@ $< -L $(LIB_DIR) -linnolidar $(DYNA_LINKFLAGS)

ifeq ($(CUSTOMER_RELEASE),OFF)
example2_static: example2.cpp $(STATIC_LIB_DIR)/$(STATIC_LIB)
	$(CXX) $(INC_DIR) $(CFLAGS) -o $@ $< -L $(STATIC_LIB_DIR) -Bstatic -linnolidar -Bdynamic $(LINKFLAGS)
else
.PHONY: example2_static
example1_static:
endif

example2_dynamic: example2.cpp $(LIB_DIR)/$(DYNA_LIB)
	$(CXX) $(INC_DIR) $(CFLAGS) -o $@ $< -L $(LIB_DIR) -linnolidar $(DYNA_LINKFLAGS)

.PHONY: clean
clean:
	rm -f Makefile.bak $(TARGETS)

.PHONY: rebuild
rebuild: clean build

.PHONY: lint
lint:
	$(CPPLINT) --root=.. `find ./ -type f \( -iname \*.cpp -o -iname \*.cc -o -iname \*.h \)`
