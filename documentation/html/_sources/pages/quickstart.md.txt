# Quickstart

## Universal Robot Setup

### Setup RS485 URCap and `socat`

Follow the [step by step instructions](https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap/blob/master/doc/install_urcap.md) 
provided by Universal Robots to allow tool communication via a socket on port 54321.

The control box file system can be accessing via `ssh root@[ROBOT-IP]` with the default password
`easybot`. You should see the internal UR prompt, something along the line of

```console
Universal Robots A/S Linux image

Production image
root@ur-[serial]:~#
```

`socat` can be installed from there from source or using a package manager. You may
want to use the attached `socat-robotiq-gripper.service` so `socat` starts every time on system
power on.

To setup the service file, just put it in `/ect/systemd/system/` and run

```console
$ sudo systemctl enable socat-robotiq-gripper.service
```

which will starts the `socat` service on boot.

## Docker Environment Setup

### Installing Docker

```console
$ sudo apt-get install docker.io
```

or follow Docker on WSL 2 backend installation guide at [Get Docker](https://docs.docker.com/get-docker/).

:::{note} 
If you are on Linux, be sure to follow [Docker Post-Installation Steps](https://docs.docker.com/engine/install/linux-postinstall/).
:::

### Clone the Repository

```console
$ git clone [repo_url]
```

### Build or Pull the Docker Image

You can build the image using the Dockerfile already included in the repository

```console
$ cd UR-Robotiq-Integrated-Driver/scripts
$ docker build -t rvl_driver -f name.dockerfile .
```

or you can pull it directly from Docker Hub with `TBA`.

### Creating the ROS Container

The included `[windows_]launch_docker_container.sh` is a useful script in creating and accessing
docker containers created from the built/pulled image. Simply run

```console
$ sh launch_docker_container.sh
```

First run will create the container if it is not already existed. Subsequent run will attached the terminal
to the docker container via `bash`. At this point, you should see the following prompt:

```console
[...]
root ~/catkin_ws
> 
```

## Inside the container

With our specific setup of running UR5e and Robotiq 2F-85 gripper, we have customized the accompanying `xacro` (hence, also includes generated
`urdf`) to match our specific machine description. You may want to modify the launch file to use the correct files for your setup.

[insert what can be changed to accommodate different configurations]