#! /usr/bin/env bash
set -e

collect_output_and_test() {
    rm -f $1.data $1.csv $1-net.csv
    (sleep 0.1 && socat -u TCP:127.0.0.1:10001 OPEN:$1.data,creat) &
    LD_LIBRARY_PATH=../../lib:${LD_LIBRARY_PATH} ./pointcloud_server -f ../../examples/example1.dat -y ../../examples/example1.yaml --model 'rev_e' --duration 10 --speed 0 --repeat 1 --server-port 10001 $2
    sleep 10
    ./pointcloud_client -f $1.data --output-csv $1.csv
    (sleep 0.1 && ./pointcloud_client -n 127.0.0.1 -p 10001 --output-csv $1-net.csv && ssdiff $1-net.csv $1.csv) &
    LD_LIBRARY_PATH=../../lib:${LD_LIBRARY_PATH} ./pointcloud_server -f ../../examples/example1.dat -y ../../examples/example1.yaml --model 'rev_e' --duration 10 --speed 0 --repeat 1 --server-port 10001 $2
    sleep 10
}

collect_output_and_test "PS32C" "--compact"
collect_output_and_test "PS32F" "--ps32full"

echo pass
