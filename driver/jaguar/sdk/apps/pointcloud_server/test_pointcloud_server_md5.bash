#! /usr/bin/env bash
set -e
ARCH=`uname -m`

echo ${ARCH}
TEST_FILE=test-data
./pointcloud_server_static -f ../../examples/example1.dat -y ../../examples/example1.yaml --model 'rev_e' --duration 3 --speed 200 --repeat 2

MD5_INTERNAL=d0f4d6ee8d102da2c973f7c42251d8f8

command -v socat || (echo please do 'sudo apt install socat' && exit 1)

# ./pointcloud_server_static -f ../../examples/example1.dat -y ../../examples/example1.yaml --model 'rev_e' --duration 7 --speed 5 --repeat 2 --server-port 10001 &
# (sleep 0.1 && rm -f $TEST_FILE; socat -u TCP:127.0.0.1:10001 OPEN:$TEST_FILE,creat; ./pointcloud_server_static --version; md5sum $TEST_FILE; (md5sum -c <<< "${MD5_INTERNAL} $TEST_FILE" ) && rm -f $TEST_FILE)

#./pointcloud_server_static -f ../../examples/example1.dat -y ../../examples/example1.yaml --model 'rev_e' --duration 6 --speed 0 --repeat 2 --server-port 10001 &
# (sleep 0.1 && rm -f $TEST_FILE; socat -u TCP:127.0.0.1:10001 OPEN:$TEST_FILE,creat; ./pointcloud_server_static --version; md5sum $TEST_FILE; (if [ ${ARCH} == "x86_64" ]; then md5sum -c <<< "${MD5_INTERNAL} $TEST_FILE" ; else echo skip MD5 check for ARCH ${ARCH}; fi) && rm -f $TEST_FILE)
