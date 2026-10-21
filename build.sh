#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
vilo_build_jobs="${VILO_BUILD_JOBS:-2}"
echo "Configuring and building thirdparty/fast ..."

cd thirdparty/fast
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j"$vilo_build_jobs"

cd ../../Sophus

echo "Configuring and building thirdparty/Sophus ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j"$vilo_build_jobs"

cd ../../g2o

echo "Configuring and building thirdparty/g2o ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j"$vilo_build_jobs"

cd ../../../

echo "Configuring and building VILO ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo "$@"
make -j"$vilo_build_jobs"
