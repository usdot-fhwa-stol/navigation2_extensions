#!/bin/bash
set -e 
sudo apt-get update
# Install 
apt install -y libnanoflann-dev
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=humble -y -r
