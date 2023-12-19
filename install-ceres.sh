#!/bin/bash

sudo apt-get update &&\
sudo apt-get install -y wget &&\
git clone -b 2.1.0 https://ceres-solver.googlesource.com/ceres-solver &&\
sudo apt-get install -y cmake &&\
sudo apt-get install -y libgoogle-glog-dev libgflags-dev &&\
sudo apt-get install -y libatlas-base-dev &&\
sudo apt-get install -y libeigen3-dev &&\
sudo apt-get install -y libsuitesparse-dev &&\
mkdir ceres-bin &&\
cd ceres-bin &&\
cmake ../ceres-solver &&\
make -j3 &&\
make test &&\
make install
