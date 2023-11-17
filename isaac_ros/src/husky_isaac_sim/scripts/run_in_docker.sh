#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

bold=`tput bold`
red=`tput setaf 1`
green=`tput setaf 2`
yellow=`tput setaf 3`
reset=`tput sgr0`

LOCAL_PATH="/workspaces/isaac_ros-dev"
ISAAC_DEMO_PKG_PATH="$LOCAL_PATH/src/husky_isaac_sim"

run_desktop()
{
    local LIBWEBSOCKETPP_PKG=$(dpkg -l 2>/dev/null | grep -m1 "libwebsocketpp")
    if [ -z "$LIBWEBSOCKETPP_PKG" ] ; then
        echo " - ${green}Install dependencies foxglove websocket${reset}"
        sudo apt-get update
        sudo apt-get install -y libwebsocketpp-dev
        sudo rm -rf /var/lib/apt/lists/*
        sudo apt-get clean
    fi

    if [ ! -d $LOCAL_PATH/install ] ; then
        echo " - ${green}Build Isaac ROS${reset}"
        colcon build --symlink-install --merge-install --packages-skip husky_base husky_bringup husky_robot husky_msgs husky_desktop || { echo "${red}ROS build failure!${reset}"; exit 1; }
    fi
    
    echo " - ${green}Run rviz2 and husky push on Isaac SIM${reset}"
    # source workspace
    source install/setup.bash
    # Run demo
    ros2 launch husky_isaac_sim allinone.launch.py
}

run_jetson()
{
    local LIBWEBSOCKETPP_PKG=$(dpkg -l 2>/dev/null | grep -m1 "libwebsocketpp")
    if [ -z "$LIBWEBSOCKETPP_PKG" ] ; then
        echo " - ${green}Install dependencies foxglove websocket${reset}"
        sudo apt-get update
        sudo apt-get install -y libwebsocketpp-dev
        sudo rm -rf /var/lib/apt/lists/*
        sudo apt-get clean
    fi

    if [ ! -d $LOCAL_PATH/install ] ; then
        echo " - ${green}Build Isaac ROS${reset}"
        colcon build --symlink-install --merge-install
    fi

    # source workspace
    source install/setup.bash
    # Run demo
    ros2 launch husky_isaac_sim husky.launch.py
}

main()
{
    local PLATFORM="$(uname -m)"

    # Decode all information from startup
    while [ -n "$1" ]; do
        case "$1" in
            --foxglove)
                FOXGLOVE_RUN=true
                ;;
            *)
                echo "[ERROR] Unknown option: $1" >&2
                exit 1
                ;;
        esac
            shift 1
    done

    if [ -d $HOME/.ros/ ] ; then
        if [ ! -f $HOME/.ros/fastdds.xml ] ; then
            echo " - ${green}Copy Fast DDS configuration on .ros folder${reset}"
            cp $ISAAC_DEMO_PKG_PATH/scripts/fastdds.xml $HOME/.ros/fastdds.xml
        fi
    fi
    
    

    # Run a different installation depend of the architecture
    if [[ $PLATFORM != "aarch64" ]]; then
        run_desktop
    else
        run_jetson
    fi
}


main $@
# EOF
