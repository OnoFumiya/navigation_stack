#!/bin/bash

sudo apt update

cd ~/
cd ../..
cd usr/include/

echo -e "\e[34m\n Git Clone matplotlib-cpp \e[m"
sudo git clone https://github.com/lava/matplotlib-cpp.git
sudo apt-get install ros-noetic-eigen-conversions