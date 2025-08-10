#!/bin/bash
set -e

# Update apt cache
sudo apt-get update

# Install git + OMPL + catkin tools
sudo apt-get install -y git libompl-dev python3-catkin-tools libsdl1.2-dev libsdl-image1.2-dev


# kr_param_map
if [ ! -d "kr_param_map" ]; then
    git clone https://github.com/KumarRobotics/kr_param_map.git
fi

# GCOPTER
if [ ! -d "GCOPTER" ]; then
    git clone https://github.com/yuwei-wu/GCOPTER.git
fi
