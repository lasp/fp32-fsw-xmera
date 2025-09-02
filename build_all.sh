#!/usr/bin/env bash

set -x
set -e

./build_a.sh linux debug
./build_a.sh linux release
./build_a.sh riscv32 debug
./build_a.sh riscv32 release
