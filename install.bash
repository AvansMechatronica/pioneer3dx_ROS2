#!/bin/bash
set -e
sudo rosdep init
rosdep update

rosdep install --from-paths src --ignore-src -y