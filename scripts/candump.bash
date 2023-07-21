#!/bin/bash
set -euxo pipefail
cd "${BASH_SOURCE[0]%/*}"/../logs

sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
candump -celtA -s 0 can0

# canbusload can0@1000000
