###
#  Copyright (C) 2018 - Innovusion Inc.
#
#  All Rights Reserved.
#
#  $Id$
##

STATIC_LIB = libinnolidar.a
DYNA_LIB = libinnolidar.so
LIB_DIR = ../../lib
STATIC_LIB_DIR = ../../lib/static
TARGETS = pointcloud_client pointcloud_server
TARGETS_STATIC = pointcloud_server_static

CPPLINT = ../../build/cpplint.py
CXX ?= g++
CFLAGS ?= -fPIC -O2 -g2 -Wall -Werror -fpermissive -std=c++11
LINKFLAGS ?= -pthread -ldl #-Wl,-Bstatic -static
DYNA_LINKFLAGS ?= -pthread -ldl
INC_DIR_COMMON = -I../.. -I/usr/include/eigen3/

INC_DIR = $(INC_DIR_COMMON)

.PHONY: build
build: lint $(TARGETS)

.PHONY: all
all: build $(TARGETS_STATIC)

pointcloud_client: pointcloud_client.cpp
ifeq ($(origin QNX_TOOLCHAIN_PREFIX), undefined)
	$(CXX) $(INC_DIR) $(CFLAGS) -o $@ $<
else
	$(CXX) $(INC_DIR) $(CFLAGS) -o $@ $< -lsocket
endif

pointcloud_server: pointcloud_server.cpp $(LIB_DIR)/$(DYNA_LIB)
	$(CXX) $(INC_DIR) $(CFLAGS) -o $@ $< -L $(LIB_DIR) -linnolidar $(DYNA_LINKFLAGS)

pointcloud_server_static: pointcloud_server.cpp $(STATIC_LIB_DIR)/$(STATIC_LIB)
	$(CXX) $(INC_DIR) $(CFLAGS) -o $@ $< -L $(STATIC_LIB_DIR) -Bstatic -linnolidar -Bdynamic $(LINKFLAGS)

.PHONY: clean
clean:
	rm -f Makefile.bak $(TARGETS) $(TARGETS_STATIC)

.PHONY: rebuild
rebuild: clean build

.PHONY: lint
lint:
	$(CPPLINT) --root=.. --filter=-readability/fn_size `find ./ -type f \( -iname \*.cpp -o -iname \*.cc -o -iname \*.h \)`

.PHONY: test
test:   build
	./test_pointcloud_client.bash

.PHONY: test_md5
test_md5:   build
	./test_pointcloud_server_md5.bash
