#! /bin/bash
TMP_SETUP_DIR=${PWD}/"tmp_setup"
THIRD_PARTY_DIR=${PWD}/"third-party"
rm -rf ${TMP_SETUP_DIR} 2>&1
rm -rf ${THIRD_PARTY_DIR} 2>&1
mkdir -p ${TMP_SETUP_DIR}
mkdir -p ${THIRD_PARTY_DIR}
pushd ${TMP_SETUP_DIR}

echo "========================================================================="
echo "Installing Prerequisite Libraries"
echo "========================================================================="
sudo apt install build-essential libeigen3-dev clang

echo "========================================================================="
echo "Installing Boost C++ Library"
echo "========================================================================="
BOOST_VERSION=1.74.0 
BOOST_TOOLSET="clang-8.0"
BOOST_CFLAGS="-fPIC -std=c++14 -DBOOST_ERROR_CODE_HEADER_ONLY"
BOOST_BASENAME="boost-${BOOST_VERSION}-${CXX_TAG}"
BOOST_PACKAGE_BASENAME=boost_${BOOST_VERSION//./_}
BOOST_DOWNLOAD="https://dl.bintray.com/boostorg/release/${BOOST_VERSION}/source/${BOOST_PACKAGE_BASENAME}.tar.gz"

PY_VERSION="3"
PYTHON_BINARY="/usr/bin/env python${PY_VERSION}"
PYTHON_INCLUDE_DIR=$(python${PY_VERSION}-config --includes | cut -c 3- | awk {'print $1'})
export CPLUS_INCLUDE_PATH="$CPLUS_INCLUDE_PATH:${PYTHON_INCLUDE_DIR}"

echo "Retrieving Boost..."
wget ${BOOST_DOWNLOAD} || true

echo "Extracting Boost..."
tar -xzf ${BOOST_PACKAGE_BASENAME}.tar.gz
mv ${BOOST_PACKAGE_BASENAME} ${BOOST_BASENAME}-source

pushd ${BOOST_BASENAME}-source >/dev/null

# Patch the header warnings for geometry libraries
echo "Patching Boost library headers..."
patch -p2 < ../../util/boost_header_warnings.patch 

./bootstrap.sh \
    --with-toolset=clang \
    --prefix=${THIRD_PARTY_DIR}/boost-install

./b2 --without-mpi --without-graph_parallel toolset="${BOOST_TOOLSET}" link=static cxxflags="${BOOST_CFLAGS}" --prefix="${BOOST_INSTALL_DIR}" -j `nproc --all` stage release
./b2 --without-mpi --without-graph_parallel toolset="${BOOST_TOOLSET}" link=static cxxflags="${BOOST_CFLAGS}" --prefix="${BOOST_INSTALL_DIR}" -j `nproc --all` install
popd >/dev/null

echo "========================================================================="
echo "Setup Complete."
echo "========================================================================="
popd >/dev/null

rm -rf ${TMP_SETUP_DIR}
