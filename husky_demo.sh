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

HIL_DEMO=false
FOXGLOVE_RUN=false
# Requested version to install this set of demo on Jetson
ISAAC_DEMO_ROS_L4T="35.3" # 35.1 = Jetpack 5.0.2
ISAAC_SIM_VERSION="isaac_sim-2023.1.1"  # Isaac SIM version

# DO NOT EDIT

PROJECT_PATH=$(pwd)
ISAAC_ROS_PATH="$PROJECT_PATH/isaac_ros"
ISAAC_ROS_SRC_PATH="$ISAAC_ROS_PATH/src"
ISAAC_DEMO_LOCAL_PATH="$ISAAC_ROS_SRC_PATH/husky_isaac_sim"
ISAAC_SIM_PATH="$HOME/.local/share/ov/pkg/$ISAAC_SIM_VERSION"
ISAAC_SIM_ROS_PATH="$ISAAC_SIM_PATH/ros2_workspace"
ISAAC_SIM_ROS_SRC_PATH="$ISAAC_SIM_ROS_PATH/src"

ISAAC_DEMO_SIMULATION_PATH="$ISAAC_DEMO_LOCAL_PATH/scripts/husky_isaac_sim.py"

usage()
{
    if [ "$1" != "" ]; then
        echo "${red}$1${reset}" >&2
    fi
    
    local name=$(basename ${0})
    echo "This script install isaac_demo on your desktop or your NVIDIA Jetson" >&2
    echo "$name [options]" >&2
    echo "${bold}options:${reset}" >&2
    echo "   -y                   | Run this script silent" >&2
    echo "   --HIL                | Run Isaac ROS from Jetson Orin series" >&2
    echo "   --reset              | Reset all demo" >&2
    echo "   --rviz               | Run rviz2 on desktop (default)" >&2
    echo "   --foxglove           | Run foxglove on desktop" >&2
    echo "   --skip-install       | Skip installing and just run the demo" >&2
    echo "   -h|--help            | This help" >&2
}


pull_isaac_ros_packages()
{
    local path=$1
    if [ ! -d $ISAAC_ROS_SRC_PATH ] ; then
        echo " - ${green}Make folder $ISAAC_ROS_SRC_PATH${reset}"
        mkdir -p $ISAAC_ROS_SRC_PATH
    fi

    echo " - ${green}Pull or update all Isaac ROS packages${reset}"
    cd $ISAAC_ROS_PATH
    # Recursive import
    # https://github.com/dirk-thomas/vcstool/issues/93
    vcs import src < $path --recursive
    vcs pull src

    cd $PROJECT_PATH
}


workstation_install()
{
    if [ -z ${SKIP_INSTALL+x} ] ; then
        if [ ! -d $ISAAC_SIM_PATH ] ; then
            echo "${bold}${red}Install Isaac SIM $ISAAC_SIM_VERSION on your workstation${reset}"
            exit 1
        fi

        echo "${green}${bold}Update/Install on desktop${reset}"

        if [ -d $HOME/.ros/ ] ; then
            if [ ! -f $HOME/.ros/fastdds.xml ] ; then
                echo " - ${green}Copy Fast DDS configuration on .ros folder${reset}"
                cp $ISAAC_DEMO_LOCAL_PATH/scripts/fastdds.xml $HOME/.ros/fastdds.xml
            fi
        else
            echo "${bold}${red}ROS not installed${reset}"
        fi

        pull_isaac_ros_packages $ISAAC_DEMO_LOCAL_PATH/rosinstall/husky_workstation.rosinstall
        if ! $HIL_DEMO ; then
            pull_isaac_ros_packages $ISAAC_DEMO_LOCAL_PATH/rosinstall/husky_robot.rosinstall
        fi
    fi

    unset LD_LIBRARY_PATH

    if ! $HIL_DEMO ; then
        # Run everything from workstation
        # Load host path (Only for Docker)
        # Load host path, this is used to share the same path between host and container for Isaac SIM
        # this is required to load the same urdf meshes
        echo "$(pwd)" > $ISAAC_ROS_SRC_PATH/host_path
        echo " - ${green}Run Isaac ROS and Husky${reset}"
        cd $ISAAC_ROS_SRC_PATH/isaac_ros_common
        gnome-terminal  --title="Isaac ROS terminal" -- sh -c "bash -c \"scripts/run_dev.sh $ISAAC_ROS_PATH; exec bash\""
        # https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ISAAC_SIM_PATH/exts/omni.isaac.ros2_bridge/humble/lib
        # Pass reference to Docker
        echo ">>> $ISAAC_SIM_PATH/python.sh $ISAAC_DEMO_SIMULATION_PATH"
    else
        # Run just rviz and robot description on workstation
        source /opt/ros/humble/setup.bash
        # Build packages
        if [ ! -d $ISAAC_ROS_PATH/install ] ; then
            echo " - ${green}Build Husky demo packages ${reset}"
            cd $ISAAC_ROS_PATH
            colcon build --symlink-install --merge-install --packages-up-to nvblox_rviz_plugin husky_isaac_sim husky_description xacro || { echo "${red}ROS build failure!${reset}"; exit 1; }
            cd $PROJECT_PATH
        fi
        gnome-terminal -- bash -c "bash -c \"source $ISAAC_ROS_PATH/install/setup.bash;echo 'When Isaac SIM is running execute:';echo 'ros2 launch husky_isaac_sim robot_display.launch.py'; exec bash\""
    fi

    # https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox/blob/main/docs/tutorial-isaac-sim.md
    # Run Isaac ROS with Carter in a Warehouse
    echo " - ${green}Start Isaac SIM ${bold}$ISAAC_SIM_VERSION${reset}"
    echo "   ${green}Path:${reset} $ISAAC_DEMO_SIMULATION_PATH"
    $ISAAC_SIM_PATH/python.sh $ISAAC_DEMO_SIMULATION_PATH

}


jetson_l4t_check()
{
    # Read version
    local JETSON_L4T_STRING=$(dpkg-query --showformat='${Version}' --show nvidia-l4t-core)
    # extract version
    local JETSON_L4T_ARRAY=$(echo $JETSON_L4T_STRING | cut -f 1 -d '-')
    # Load release and revision
    local JETSON_L4T_RELEASE=$(echo $JETSON_L4T_ARRAY | cut -f1,2 -d '.')
    local JETSON_L4T_REVISION=${JETSON_L4T_ARRAY#"$JETSON_L4T_RELEASE."}
    echo "$JETSON_L4T_RELEASE"
}


jetson_install()
{
    if [ -z ${SKIP_INSTALL+x} ] ; then
        local JETSON_L4T=$(jetson_l4t_check)
        if [[ "$(printf '%s\n' "$JETSON_L4T" "$ISAAC_DEMO_ROS_L4T" | sort -V | head -n1)" != "$JETSON_L4T" ]]; then
            echo "${bold}${red}You cannot install isaac_demo on this Jetpack with L4T $JETSON_L4T need L4T $ISAAC_DEMO_ROS_L4T${reset}"
            exit 1
        fi

        echo "${green}${bold}Install on NVIDIA Jetson L4T $JETSON_L4T${reset}"

        pull_isaac_ros_packages $ISAAC_DEMO_LOCAL_PATH/rosinstall/husky_robot.rosinstall
    fi

    if [ ! -f $ISAAC_ROS_SRC_PATH/isaac_ros_common/scripts/.isaac_ros_common-config  ] ; then
        echo " - ${green}Setup Isaac ROS docker image${reset}"
        cd $ISAAC_ROS_SRC_PATH/isaac_ros_common/scripts
        touch .isaac_ros_common-config 
        echo CONFIG_IMAGE_KEY=ros2_humble.realsense > .isaac_ros_common-config
    fi

    echo " - ${green}Move to Isaac ROS common and run image${reset}"
    cd $ISAAC_ROS_SRC_PATH/isaac_ros_common
    bash scripts/run_dev.sh $ISAAC_ROS_PATH
}

main()
{
    local PLATFORM="$(uname -m)"
    local SILENT=false
    local RESET=false

    # Decode all information from startup
    while [ -n "$1" ]; do
        case "$1" in
            -h|--help) # Load help
                usage
                exit 0
                ;;
            --reset)
                RESET=true
                ;;
            --rviz)
                RVIZ_RUN=true
                ;;
            --HIL|--hil)
                HIL_DEMO=true
                ;;
            --skip-install)
                SKIP_INSTALL=true
                ;;
            -y)
                SILENT=true
                ;;
            *)
                usage "[ERROR] Unknown option: $1" >&2
                exit 1
                ;;
        esac
            shift 1
    done

    if $RESET ; then
        echo "${bold}${red}Reset all demo${reset}"
        sudo rm -rf $ISAAC_ROS_PATH/build $ISAAC_ROS_PATH/install $ISAAC_ROS_PATH/log
        sudo rm -rf $ISAAC_ROS_SRC_PATH/isaac_ros_* $ISAAC_ROS_SRC_PATH/joint_state_publisher 
        sudo rm -rf $ISAAC_ROS_SRC_PATH/robot_state_publisher $ISAAC_ROS_SRC_PATH/realsense-ros
        sudo rm -rf $ISAAC_ROS_SRC_PATH/xacro $ISAAC_ROS_SRC_PATH/ros-foxglove-bridge
        sudo rm -rf $ISAAC_ROS_SRC_PATH/host_path
        exit 0
    fi

    # Recap installatation
    echo "------ Configuration ------"
    echo " - ${bold}Install on:${reset} ${green}$PLATFORM${reset}"
    echo " - ${bold}User:${reset} ${green}$USER${reset} - ${bold}Hostname:${reset} ${green}$HOSTNAME${reset}"
    if [ $PLATFORM = "aarch64" ] ; then
        local JETSON_L4T=$(jetson_l4t_check)
        echo " - ${bold}Jetson L4T:${reset} ${green}$JETSON_L4T${reset}"
    fi
    echo "---------------------------"

    if ! command -v vcs &> /dev/null ; then
        echo " - ${green}Install required packages${reset}"
        sudo apt install -y git-lfs python3-vcstools python3-pip
        sudo pip3 install -U vcstool
    fi

    # Run a different installation depend of the architecture
    if [[ $PLATFORM != "aarch64" ]]; then
        workstation_install
    else
        jetson_install
    fi

}

main $@
# EOF
