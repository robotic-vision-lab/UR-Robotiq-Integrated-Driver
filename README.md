# UR Robotiq Integrated Driver

This repository provides Python 3 packages designed for [ROS 2
Humble](https://index.ros.org/doc/ros2/Releases/Release-Humble-Hawksbill/) to control the [Robotiq 2F
gripper](https://robotiq.com/products/2f85-140-adaptive-robot-gripper) attached to Universal Robot (UR) robot arms using
the toolport available on the robot arms' wrist.

## System Requirements

You must complete the following steps before using this repository:

1. Remove any Robotiq related URCap present on the robot controller via the teach pendant.
2. Complete the robot side setup as instructed by [`ur_robot_driver`
   documentation](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/setup_tool_communication.html).

## Quick Start

1. Clone the repository

```shell
user@host:~$ git clone https://github.com/robotic-vision-lab/UR-Robotiq-Integrated-Driver.git
```

2. Build the Docker image

```shell
user@host:~$ cd UR-Robotiq-Integrated-Driver/docker
user@host:~/UR-Robotiq-Integrated-Driver/docker$ docker build -t rvl-ur-robotiq-driver:latest
# [...]
user@host:~/UR-Robotiq-Integrated-Driver/docker$ cd ..
user@host:~/UR-Robotiq-Integrated-Driver$
```

3. Run the Docker container

```console
user@host:~/UR-Robotiq-Integrated-Driver$ docker run \
--rm \
--tty \
--interactive \
--network host \
--privileged \
--name rvl-ur-robotiq-container \
--env DISPLAY=$DISPLAY \
--volume /tmp/.X11-unix:/tmp/.X11-unix \
--volume $(pwd)/colcon_ws:/root/colcon_ws \
rvl-ur-robotiq-driver:latest \
bash

root@host:/#
```

4. Build the ROS 2 workspace inside the Docker container

```console
root@host:/# cd /root/colcon_ws
root@host:~/colcon_ws# source /opt/ros/${ROS_DISTRO}/setup.bash
root@host:~/colcon_ws# apt-get update
root@host:~/colcon_ws# rosdep update
root@host:~/colcon_ws# rosdep install --from-paths src --ignore-src -r -y
root@host:~/colcon_ws# colcon build --symlink-install
root@host:~/colcon_ws# source install/setup.bash
```

5. Start `ur_robot_driver` with tool communication enabled

```console
root@host:~/colcon_ws# ros2 launch ur_robot_driver ur_control.launch.py ur_type:=UR_TYPE robot_ip:=ROBOT_IP use_tool_communication:=true
```

Replace `UR_TYPE` with the type of the UR robot arm you are using, e.g. `ur5e`. Replace `ROBOT_IP` with the IP address
of the robot arm.

6. Start `rvl_ur_robotiq_driver` controller node

In a ***new terminal***, run the following commands:

```console
user@host:~$ docker exec -it rvl-ur-robotiq-container bash
root@host:/# cd /root/colcon_ws
root@host:~/colcon_ws# source install/setup.bash
root@host:~/colcon_ws# ros2 run rvl_robotiq_driver robotiq_controller
```

> :spiral_notepad: NOTE: Verify that the gripper is powered and the gripper status LED is on and is solid blue. This means that
> the driver has successfully established communication with the gripper. If the LED is not on or is red, please
> see the [Troubleshooting](#troubleshooting) section.

7. Activate the gripper via provided service

In a ***new terminal***, run the following commands:

```console
user@host:~$ docker exec -it rvl-ur-robotiq-container bash
root@host:/# cd /root/colcon_ws
root@host:~/colcon_ws# source install/setup.bash
root@host:~/colcon_ws# ros2 service call /robotiq/activate std_srvs/srv/Trigger
```
At this point, you should see the gripper cycle through its activation sequence by fully closing then fully opening its
fingers. The gripper is now ready to receive commands.

> ⚠️ WARNING: This is an example usage container, and the run command includes the `--rm` flag, which means that the
> container will be deleted after exiting the container. If you want to keep the container, remove the `--rm` flag from
> the run command.

## Provided Services and Topics

### Topics

```console
/robotiq/joint_states
/robotiq/status
```

### Services

```
/robotiq/activate
/robotiq/auto_close/fragile
/robotiq/auto_close/medium
/robotiq/auto_close/soft
/robotiq/auto_close/strong
/robotiq/auto_open/fragile
/robotiq/auto_open/medium
/robotiq/auto_open/soft
/robotiq/auto_open/strong
/robotiq/reactivate
/robotiq/request_status
/robotiq/set_opening
/robotiq/set_position
```

You can use `ros2 topic info` and `ros2 service info` to get more information about each topic and service.
Alternatively, `rqt` have plugins that can be used to inspect topics and call services, though this would require you to
enable GUI from the Docker container, which is not in the scope of this repository. This feature can still be accessible
if you have ROS 2 Humble on your host machine.

## Citation

If you find this code useful, then please consider citing our work.

You can use the "Cite this repository" feature under About section for automatic generation of APA and BibTex references.

## Troubleshooting

### 1. Gripper is not powered (No status LED)

- Check physical connection between the gripper and the toolport on the robot arm.
- Check that the robot is not in E-Stop or Power Off state.

### 2. Status LED is RED

- Make sure that the RS-485 URCap is installed and enabled on the robot controller.
- Make sure that the `ur_robot_driver` launch command has `use_tool_communication:=true` argument and correct robot IP
  address.
- Make sure that the gripper controller node is running and is not reporting any errors.

### 3. Status LED is BLUE but gripper is not responding to commands

- Restart the gripper controller node.

## References

- [ROS Industrial Robotiq](https://github.com/ros-industrial/robotiq)
- [Universal Robot ROS 2 driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
