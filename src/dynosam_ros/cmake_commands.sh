#!/usr/bin/env bash

export AMENT_PREFIX_PATH=/home/bonnie/DynOSAM/install/dynosam:/home/bonnie/DynOSAM/install/dynamic_slam_interfaces:${AMENT_PREFIX_PATH}
export CMAKE_PREFIX_PATH=/home/bonnie/DynOSAM/install/dynosam:/home/bonnie/DynOSAM/install/dynamic_slam_interfaces:/opt/ros/humble
export LD_LIBRARY_PATH=/home/bonnie/DynOSAM/install/dynosam/lib:/home/bonnie/DynOSAM/install/dynamic_slam_interfaces/lib:${LD_LIBRARY_PATH}
export PYTHONPATH=/home/bonnie/DynOSAM/install/dynamic_slam_interfaces/local/lib/python3.10/dist-packages:${PYTHONPATH}

cmake /home/bonnie/DynOSAM/src/dynosam_ros -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebinfo -G Ninja -DCMAKE_INSTALL_PREFIX=/home/bonnie/DynOSAM/install/dynosam_ros
cmake --build /home/bonnie/DynOSAM/build/dynosam_ros -- -j16 -l16
cmake --install /home/bonnie/DynOSAM/build/dynosam_ros
