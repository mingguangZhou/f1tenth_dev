#!/bin/bash
source /opt/ros/foxy/setup.bash
apt-get update
rosdep install -q -y -r --from-paths src --ignore-src
pip3 install cython
source /opt/ros/foxy/setup.bash
cd src/range_libc/pywrapper/
python3 setup.py install
cd ../../..
colcon build --packages-select slam_toolbox particle_filter
source install/local_setup.bash