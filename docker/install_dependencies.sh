#!/bin/bash
set -e 
sudo apt-get update
# Install 
apt install -y libnanoflann-dev
# Set the working directory
cd /root/c1t_ws/src/
git clone https://github.com/usdot-fhwa-stol/carma-msgs
git clone -b nav2_route_server_humble https://github.com/usdot-fhwa-stol/navigation2
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=humble -y -r
