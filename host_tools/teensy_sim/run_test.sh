#!/bin/bash
# Host simulation of the Teensy S17 sequencing. Run from anywhere:
#   host_tools/teensy_sim/run_test.sh
# Stages the REAL ESCPID.ino / ESCPID.h / AWPID.* next to the mock ESCCMD.h,
# DSHOT.h and Arduino.h (quoted includes resolve in the including file's dir).
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
SRC="$HERE/../../teensy_escpid/ESCPID"
STAGE=$(mktemp -d)
cp "$SRC"/ESCPID.ino "$SRC"/ESCPID.h "$SRC"/AWPID.cpp "$SRC"/AWPID.h "$STAGE"/
cp "$HERE"/Arduino.h "$HERE"/ESCCMD.h "$HERE"/DSHOT.h "$HERE"/test_teensy_s17.cpp "$STAGE"/
g++ -std=c++17 -O1 -Wall -Wno-unused-variable -Wno-unused-function -DESCPID_OL_THROTTLE=1935 \
    -I "$STAGE" "$STAGE"/test_teensy_s17.cpp "$STAGE"/AWPID.cpp -o "$STAGE"/ts17
"$STAGE"/ts17; rc=$?
rm -rf "$STAGE"; exit $rc
