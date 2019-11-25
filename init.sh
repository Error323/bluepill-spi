#!/usr/bin/env bash

STM32CUBE_DIR=$HOME/dev/stm/tools/STM32Cube_FW_F1_V1.8.0

set -e

case $1 in
  Debug|Release|MinSizeRel)
    BUILDTYPE=$1
    shift
    ;;
  *)
    BUILDTYPE=MinSizeRel
    ;;
esac

mkdir -p build/${BUILDTYPE}
pushd build/${BUILDTYPE}

rm -rf *

cmake \
  -DSTM32CUBE=${STM32CUBE_DIR} \
  -DCMAKE_BUILD_TYPE=${BUILDTYPE} \
  -GNinja ../..

ninja

popd
