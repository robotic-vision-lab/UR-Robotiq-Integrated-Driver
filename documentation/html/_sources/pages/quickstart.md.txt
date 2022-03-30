# Quickstart

## Universal Robots Setup

### Setup RS485 URCap and `socat`

Follow the [step-by-step
instructions](https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap/blob/master/doc/install_urcap.md)
provided by Universal Robots to allow for tool communication through a socket on 
port 54321.

The control box filesystem can be accessed via `ssh root@[ROBOT-IP]` with the
default password `easybot`. You should see the internal UR prompt, i.e., 

```console
Universal Robots A/S Linux image

Production image
root@ur-[serial]:~#
```

`socat` can then be installed either from source or using a package manager.
You may want to use the attached `socat-robotiq-gripper.service` so that
`socat` starts up every time the system is powered on.

To setup the service file, just place it in `/etc/systemd/system/` and run

```console
$ sudo systemctl enable socat-robotiq-gripper.service
```

which will start the `socat` service on boot.

## Docker Environment Setup

### Installing Docker

```console
$ sudo apt-get install docker.io
```

or follow the WSL 2 backend installation guide at [Get Docker](https://docs.docker.com/get-docker/).

:::{note} 
If you are using a Linux distribution, be sure to follow [Docker Post-Installation Steps](https://docs.docker.com/engine/install/linux-postinstall/).
:::

### Clone the Source Code Repository

```console
$ git clone --recurse-submodules https://github.com/robotic-vision-lab/UR-Robotiq-Integrated-Driver
```

### Build or Pull the Docker Image

You can build the image using the Dockerfile already included in the repository

```console
$ cd UR-Robotiq-Integrated-Driver/docker
$ docker build -t rvl-ur-robotiq-driver -f Dockerfile.noetic .
```
or you can pull it directly from Docker Hub with 

```console
$ docker pull mqt0029/rvl-ur-robotiq-driver:latest` 
```

If you pulled from Docker Hub, then be sure to retag your image to
`rvl-ur-robotiq-driver:latest`

```console
$ docker tag mqt0029/rvl-ur-robotiq-driver:latest rvl-ur-robotiq-driver:latest
```

or update the image name in the launch script accordingly.

### Creating the ROS Container

```{note}

Be sure to check your image name before running the script.
```

The included `[windows_]launch_docker_container.sh` is a useful script for
creating and accessing docker containers created from the built/pulled image.
Simply run

```console
$ sh launch_docker_container.sh
```

The first run will create the container if it does not already exist.
Subsequent runs will attach the terminal to the docker container via `bash`.
At this point, you should see the following prompt

```console
[...]
root ~/catkin_ws
> 
```

## Inside the Container

With our setup using a UR5e and Robotiq 2F-85 gripper, we have customized the
accompanying `xacro` (which also includes the generated `urdf`) to match our
specific machine description. You may want to modify the launch file to use the
correct files for your own setup.

While the base path is commonly `[...]/catkin_ws/src`, this is not always the
case. Paths will be relative to where you place your ROS packages.

### Robot Xacro and URDF

The included ROS package `rvl_robot_description` contains an example of our
robot setup. The file `[...]/rvl_robot_description/xacro/rvl_ur5e.xacro` can be
a good place to start and components should be fairly straightforward to swap
in and move around.

### Launching the UR Driver 

The `ur_robot_driver` package launch file has been modified to work with the
tool port gripper and our particular setup. You can use the launch file
`[...]/rvl_ur_remote_dashboard/launch/rvl_ur5e_bringup.launch` as a reference,
and modify it to fit your setup. The important sections are highlighted below.

```XML
<arg name="robot_ip" default="192.168.1.147" doc="IP address by which ..."/>
<arg name="robot_description_file" default="$(find rvl_ur_remote_dashboard)/launch/load_rvl_ur5e.launch" doc="Robot description launch file."/>
<arg name="kinematics_config" default="$(find rvl_ur_remote_dashboard)/configs/ur5e_calibration.yaml" doc="Kinematics config file used ..."/>

<!-- always start the UR side socat -->
<arg name="use_tool_communication" default="true" doc="On e-Series robots tool communication can be enabled with this argument"/>
```

```{warning}

Double check that your calibration file is correct or [retrieve it
accordingly](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_calibration/README.md).
```

```{warning}

Make sure your robot description is correct! This is *crucial* to generating
your own URDF and SRDF that MoveIt uses to plan all subsequent motions.
```

### Building and Running ROS

A few aliases have been provided to quickly setup ROS dependencies and paths.
`bash` will also automatically source `devel/setup.bash`.

```console
$ run_rosdep
[...] installing dependencies

$ rebuild_catkin
[...] building catkin workspace

$ roslaunch <your_stuff>
```

## Debugging

Since this driver is built inside a Docker container, the most common issue is
networking. A good place to start is to ensure that all the ports (or the
network interface) are properly exposed, the robot IP is correct, and there is
nothing interfering with ROS communications both ways. Otherwise, the host OS
should not be a problem. However, should any issues arise, please [open an
issue on our GitHub repository](https://github.com/robotic-vision-lab/UR-Robotiq-Integrated-Driver/issues). 
