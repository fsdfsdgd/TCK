#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
echo "Configuring and building thirdparty/fast ..."

cd thirdparty/fast
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2

cd ../../Sophus

echo "Configuring and building thirdparty/Sophus ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2

cd ../../g2o

echo "Configuring and building thirdparty/g2o ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2

cd ../../../

echo "Configuring and building VILO ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j2
