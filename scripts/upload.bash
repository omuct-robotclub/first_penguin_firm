#!/bin/bash
set -euxo pipefail
cd "${BASH_SOURCE[0]%/*}"/..

make all
cp build/first_penguin_firm.bin /media/$USER/STLINK_V3M
