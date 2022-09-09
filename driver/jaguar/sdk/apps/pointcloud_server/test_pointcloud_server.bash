#! /usr/bin/env bash
set -e

TEST_FILE=/dev/null
(sleep 1 && socat -u TCP:127.0.0.1:10001 OPEN:$TEST_FILE,creat) &
./pointcloud_server_static -f ../../examples/example1.dat -y ../../examples/example1.yaml --model 'rev_e' --duration 4 --speed 40 --repeat 10 --server-port 10001
