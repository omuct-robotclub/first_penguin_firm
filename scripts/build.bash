#!/bin/bash
set -euxo pipefail
cd "${BASH_SOURCE[0]%/*}"/..

make clean

mkdir -p artifacts
for id in {30..39}; do
  sed -i -e "/int can_id/s/[0-9]\+/${id}/" Core/Src/main.cpp
  make all
  mv "build/first_penguin_firm.bin" "artifacts/first_penguin_firm_${id}.bin"
done
