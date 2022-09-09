###
#  Copyright (C) 2020 - Innovusion Inc.
#
#  All Rights Reserved.
#
#  $Id$
##

include ../src/version_gen.mk

DYNA_LIB = libinnodupfilter.so

OBJ_DIR = ../obj
OUT_DIR = ../lib

CPPLINT = ../build/cpplint.py
CC ?= gcc
CXX ?= g++
CFLAGS ?= -fPIC  -D_GLIBCXX_USE_CXX11_ABI=0 -O2 -g2 -Wall -Werror -fpermissive
LINKFLAGS ?= -pthread -Wl,-Bstatic -static
LINKFLAGS_DYNA ?= -pthread
AR ?= ar
OBJCOPY ?= objcopy
INC = -I../Import -I../

.PHONY: build
build: lint $(OUT_DIR)/$(DYNA_LIB)

$(OUT_DIR)/$(DYNA_LIB): $(patsubst %.cpp,$(OBJ_DIR)/%.o,$(wildcard *.cpp))
	@mkdir -p $(OUT_DIR)
	$(CC) -shared -Wl,-soname,${DYNA_LIB}.${DYNA_LIB_MAJ} -o $@- $^
	$(OBJCOPY) $(OBJCOPY_OPT) $(OBJCOPY_OPT_DYNA) $@- $@.${DYNA_LIB_MAJ}.${DYNA_LIB_MIN}.${DYNA_LIB_BUILD}
	rm -fr $@-
	rm -f $(OUT_DIR)/${DYNA_LIB}.${DYNA_LIB_MAJ} $(OUT_DIR)/${DYNA_LIB}
	ln -s ${DYNA_LIB}.${DYNA_LIB_MAJ}.${DYNA_LIB_MIN}.${DYNA_LIB_BUILD} $(OUT_DIR)/${DYNA_LIB}.${DYNA_LIB_MAJ}
	ln -s ${DYNA_LIB}.${DYNA_LIB_MAJ} $(OUT_DIR)/${DYNA_LIB}
	cd -

# Compiling every *.cpp to *.o
$(OBJ_DIR)/%.o: %.cpp
	@mkdir -p $(OBJ_DIR)
	$(CXX) -c $(INC) $(CFLAGS) -o $@  $<

.PHONY: clean
clean:
	rm -f $(OBJ_DIR)/*.o $(TARGETS)
	rm -fr $(OUT_DIR)

.PHONY: rebuild
rebuild: clean build

.PHONY: lint
lint:
	$(CPPLINT) --root=.. `find ./ -maxdepth 1 -type f \( \( -iname \*.cpp -o -iname \*.h \) \) `
