#!/usr/bin/env bash

export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/opt/ros/humble
cmake /home/bonnie/DynOSAM/src/dynosam -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebinfo -G Ninja -DCMAKE_INSTALL_PREFIX=/home/bonnie/DynOSAM/install/dynosam
cmake --build /home/bonnie/DynOSAM/build/dynosam -- -j16 -l16
cmake --install /home/bonnie/DynOSAM/build/dynosam
