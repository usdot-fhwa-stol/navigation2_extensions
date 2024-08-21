#!/bin/bash
set -e 

sudo apt-get update
# Install 
apt install -y libnanoflann-dev
rosdep update
rosdep install --from-paths /__w/navigation2_extensions/navigation2_extensions/src --ignore-src --rosdistro=humble -y -r
