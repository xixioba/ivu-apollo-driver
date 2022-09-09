#! /usr/bin/env bash
set -e
make

SCRIPT=`basename "$0"`
RUN_DT=`date +%Y%m%d#%H-%M`
LIDAR_IP=172.168.1.10

function script_usage {
    printf "%s takes one or two parameters:\n" $SCRIPT
    printf "\t%s <YAML_FILE> [IP_ADDRESS]\n\n" $SCRIPT
    printf "<YAML_FILE> -- name of YAML file for attached sensor (required)\n"
    printf "[IP_ADDRESS] -- IP address for attached sensor (optional)\n"
    printf "                if IP address not given, it defaults to %s\n\n" \
           $LIDAR_IP
}

if [[ $# -ne 1 && $# -ne 2 ]]; then
    script_usage
    exit 1
fi

YAML=$1
if [[ $# -eq 2 ]]; then
    LIDAR_IP=$2
fi

LD_LIBRARY_PATH=../lib:${LD_LIBRARY_PATH} ./example1_dynamic \
    -n $LIDAR_IP \
    -w frames_$RUN_DT.csv \
    -y $YAML \
    -d 600 \
    2>&1 | tee example_console_$RUN_DT.log
