#! /usr/bin/env bash
set -e
make
LD_LIBRARY_PATH=../lib:${LD_LIBRARY_PATH} ./example1_dynamic -f example1.dat -y example1.yaml --model 'rev_e' --duration 10 --instance 2 --speed 15 --keepalive -r 20
