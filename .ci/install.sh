#!/bin/sh
set -e

mkdir $HOME/git
export CXXFLAGS="-Wno-unused-command-line-argument"

# Install YCM
cd $HOME/git
git clone --depth 1 -b $DEPS_BRANCH https://github.com/robotology/ycm.git
cd ycm
mkdir build && cd build
cmake .. \
    -G"$TRAVIS_CMAKE_GENERATOR" \
    -DCMAKE_INSTALL_PREFIX=$DEPS_CACHE
cmake --build . --target install

# Install Yarp
cd $HOME/git
#git clone --depth 1 -b $DEPS_BRANCH https://github.com/robotology/yarp.git
echo "Checking out devel. Use master as soon as it will be fixed."
git clone --depth 1 -b devel https://github.com/robotology/yarp.git
cd yarp
mkdir build && cd build
cmake .. \
    -G"$TRAVIS_CMAKE_GENERATOR" \
    -DCMAKE_BUILD_TYPE=$TRAVIS_BUILD_TYPE \
    -DCMAKE_INSTALL_PREFIX=$DEPS_CACHE \
    -DCREATE_LIB_MATH=ON
cmake --build . --config $TRAVIS_BUILD_TYPE --target install

# Install icub-main
cd $HOME/git
echo "Checking out devel. Use master as soon as it will be fixed."
git clone --depth 1 -b devel https://github.com/robotology/icub-main.git
# git clone --depth 1 -b $DEPS_BRANCH https://github.com/robotology/icub-main.git
cd icub-main
mkdir build && cd build
cmake .. \
    -G"$TRAVIS_CMAKE_GENERATOR" \
    -DCMAKE_BUILD_TYPE=$TRAVIS_BUILD_TYPE \
    -DCMAKE_INSTALL_PREFIX=$DEPS_CACHE
cmake --build . --config $TRAVIS_BUILD_TYPE --target install $CMAKE_BUILD_OPTIONS

# Install iDynTree
cd $HOME/git
git clone --depth 1 -b $DEPS_BRANCH https://github.com/robotology/idyntree.git
cd idyntree
mkdir build && cd build
cmake .. \
    -G"$TRAVIS_CMAKE_GENERATOR" \
    -DCMAKE_BUILD_TYPE=$TRAVIS_BUILD_TYPE \
    -DCMAKE_INSTALL_PREFIX=$DEPS_CACHE
cmake --build . --config $TRAVIS_BUILD_TYPE --target install $CMAKE_BUILD_OPTIONS

# Install BlockFactory
cd $HOME/git
git clone --depth 1 https://github.com/robotology/blockfactory
cd blockfactory
mkdir build && cd build
cmake .. \
    -G"$TRAVIS_CMAKE_GENERATOR" \
    -DCMAKE_BUILD_TYPE=$TRAVIS_BUILD_TYPE \
    -DCMAKE_INSTALL_PREFIX=$DEPS_CACHE
cmake --build . --config $TRAVIS_BUILD_TYPE --target install $CMAKE_BUILD_OPTIONS
