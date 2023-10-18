# Husky Demo

## Hardware required

Workstation:

1. Internet connection
2. x86/64 machine
3. Install Ubuntu 22.04
4. [Install ROS2 Humble](#install-ros2-humble)
5. NVIDIA Graphic card with RTX
6. Display
7. Keyboard and Mouse

NVIDIA Jetson:

1. NVIDIA Jetson AGX Orin
2. Jetpack 5.1.2

Tools:

1. Router
2. eth cables

## Install ROS2 Humble

Follow the Isaac SIM 2023 official documentation and check or install ROS2 Humble on your desktop [Running native ROS](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html?highlight=native%20ros#running-native-ros).

In quick steps:

1. Download ROS 2 following the instructions on the official website: [ROS 2 Humble Ubuntu 22.04](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
2. Source the ROS environment in the terminal. You must perform this step each time before using any ROS commands.

```console
source /opt/ros/humble/setup.bash
Install vision_msgs_package
```

3. Install vision msgs package

```console
sudo apt install ros-humble-vision-msgs
```

## Install Husky Demo

Clone this repository and move to repository folder

```console
git clone https://github.com/NVIDIA-AI-IOT/husky_demo.git
cd husky_demo
```

Add docker group to your user

```console
sudo usermod -aG docker $USER && newgrp docker
```

Set the default nvidia runtime

You're going to be building containers, you need to set Docker's `default-runtime` to `nvidia`, so that the NVCC compiler and GPU are available during `docker build` operations.  Add `"default-runtime": "nvidia"` to your `/etc/docker/daemon.json` configuration file before attempting to build the containers:

``` json
{
    "default-runtime": "nvidia",
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    }
}
```

Then restart the Docker service, or reboot your system before proceeding:

```console
sudo systemctl restart docker
```

Run the installer

```console
./husky_demo.sh
```

## Run demo

```console
bash src/husky_isaac_sim/scripts/run_in_docker.sh
```
