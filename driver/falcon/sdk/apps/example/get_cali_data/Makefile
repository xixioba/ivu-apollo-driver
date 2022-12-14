###
#  Copyright (C) 2021 - Innovusion Inc.
#
#  All Rights Reserved.
#
#  $Id$
##

STATIC_LIB = libinnolidarsdkclient.a libinnolidarsdkcommon.a libinnolidarutils.a
DYNA_LIB = libinnolidarsdkclient.so libinnolidarsdkcommon.so libinnolidarutils.so
SRC_DIR = ../../../apps/example/get_cali_data
OBJ_DIR = ../../../obj/example/get_cali_data
DEP_DIR = ../../../dep/example/get_cali_data
LIB_DIR = ../../../lib
ifdef MINGW64
TARGETS_STATIC = get_cali_data.exe
else
TARGETS_STATIC = get_cali_data
TARGETS = get_cali_data_dynamic
endif

CPPLINT = ../../../build/cpplint.py
CC ?= gcc
CXX ?= g++
STRIP ?= strip
CFLAGS ?= -D_GLIBCXX_USE_CXX11_ABI=0 -O2 -g2 -Wall -Werror -fpermissive -std=gnu++11 $(INC_DIR)
LINKFLAGS ?= -Wl,--whole-archive -lpthread -Wl,--no-whole-archive -Wl,-Bstatic -static
DYNA_LINKFLAGS ?= -pthread
INC_DIR = -I../../ -I../../../ -I../../../src/
INNO_CLIENT_LIBS = -linnolidarsdkclient -linnolidarsdkcommon -linnolidarutils
ifneq (,$(filter $(ARCH_TAG), -mingw64 -qnx))
	OTHER_LIBS = -lstdc++ -lm
else
	OTHER_LIBS = -ldl -lstdc++ -lm
endif

INC += -I./ $(INC_DIR)
OTHER_CFLAGS ?= -fPIC
CFLAGS += $(INC) $(OTHER_CFLAGS)
OTHER_LIB_BUILD_CFLAG ?=

ifeq ($(ARCH_TAG), -arm)
	CFLAGS += -march=armv8-a+crc -mtune=cortex-a53 -DARCH_ARM64
endif

ifeq ($(ARCH_TAG), -macos)
	CFLAGS += -Wno-error=unused-private-field -Wno-error=format
	LINKFLAGS = -pthread ../../../lib/libinnolidarsdkclient.a ../../../lib/libinnolidarsdkcommon.a ../../../lib/libinnolidarutils.a
	DYNA_LINKFLAGS = -pthread -linnolidarsdkclient -linnolidarsdkcommon -linnolidarutils
	INNO_CLIENT_LIBS =
endif

SRCS := $(wildcard $(SRC_DIR)/*.cpp)
OBJS := $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SRCS))
DEPS := $(patsubst $(SRC_DIR)/%.cpp, $(DEP_DIR)/%.d, $(SRCS))
STATIC_LIB_FILES := $(patsubst %, $(LIB_DIR)/%, $(STATIC_LIB))
DYNA_LIB_FILES := $(patsubst %, $(LIB_DIR)/%, $(DYNA_LIB))

.PHONY: build
build: lint $(TARGETS_STATIC) $(TARGETS)

.PHONY: all
all: build

$(OBJ_DIR) :
	mkdir -p $(OBJ_DIR)

$(OUT_DIR) :
	mkdir -p $(OUT_DIR)

$(DEP_DIR) :
	mkdir -p $(DEP_DIR)

-include $(DEPS)

$(OBJ_DIR)/%.o: %.cpp | $(OBJ_DIR) $(DEP_DIR)
	$(CC) -c $(CFLAGS) $*.cpp -o $(OBJ_DIR)/$*.o
	$(CC) -MM $(CFLAGS) -MT"$@" $*.cpp > $(DEP_DIR)/$*.d

$(TARGETS): $(OBJ_DIR)/get_cali_data.o $(DYNA_LIB_FILES)
	$(CC) $(CFLAGS) -o $@ $< -L $(LIB_DIR) $(INNO_CLIENT_LIBS) $(DYNA_LINKFLAGS) $(OTHER_LIBS) $(OTHER_LIB_BUILD_CFLAG)

$(TARGETS_STATIC): $(OBJ_DIR)/get_cali_data.o $(STATIC_LIB_FILES)
	$(CC) $(CFLAGS) -o $@ $< -L $(LIB_DIR) $(INNO_CLIENT_LIBS) $(LINKFLAGS) $(OTHER_LIBS) $(OTHER_LIB_BUILD_CFLAG)

.PHONY: clean
clean:
	rm -rf  $(OBJ_DIR) $(DEP_DIR) $(TARGETS) $(TARGETS_STATIC) lint_checked

.PHONY: lint
lint: lint_checked

lint_checked: $(wildcard *.h) $(wildcard *.cpp)
	$(CPPLINT) --root=.. $?
	touch lint_checked
