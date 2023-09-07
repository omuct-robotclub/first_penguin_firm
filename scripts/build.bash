#!/bin/bash
set -euxo pipefail
cd "${BASH_SOURCE[0]%/*}"/..

make clean
rm -rf artifacts

mkdir -p artifacts
for id in {45,40,35,30}; do
  sed -i -e "/int can_id/s/[0-9]\+/${id}/" Core/Src/main.cpp
  for num in {3..0}; do
    sed -i -e "/uint8_t spnum/s/[0-9]\+/${num}/2g" Core/Src/main.cpp
    make all
    mv "build/first_penguin_firm.bin" "artifacts/first_penguin_firm_${id}_${num}.bin"
  done
done

tar -zcvf artifacts.tar.gz artifacts
zip -r artifacts.zip artifacts
mv artifacts.* artifacts
