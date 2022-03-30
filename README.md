# UR-Robotiq Integrated Driver

## Overview

The UR-Robotiq Integrated Driver is a wrapper for existing ROS services and
topics provided by the Universal Robots (UR) ROS driver [1]. It has support for
a Robotiq 2F gripper directly attached to the UR tool port [2]. The driver also
provides functionality for actuating the gripper in various modes (e.g., grasp
on open/close, specific jaw gap, etc.) as well as integrating the MoveIt [3]
motion planning framework which tracks the entire setup, UR robot, and Robotiq
gripper.

## Citation

If you find this code useful, then please consider citing our work.

You can use the "Cite this repository" feature under About for automatic
generation of APA and BibTex references. ***CURRENTLY DISABLED!***

Otherwise, the static BibTex entry below can be used.

```TeX
@software{tram2022ur,
  author={Tram, Minh},
  title={{UR-Robotiq Integrated Driver}},
  url={https://github.com/robotic-vision-lab/UR-Robotiq-Integrated-Driver}
  version={0.0.1-alpha},
  year={2022}
}
```

## Documentation

The documentation is generated using Sphinx and is available in
[HTML](documentation/html/_modules/index.html) and
[PDF](documentation/rvl_driver_documentation.pdf) formats. For instructions on 
how to regenerate documentation based on updated versions of the software,
please refer to 
[Sphinx Docgen Instructions](catkin_ws/src/rvl_ur_robotiq/sphinx-docgen/README.md).

## References

[1] [Official Universal Robot ROS 1 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)  
[2] [ROS Industrial Robotiq](https://wiki.ros.org/robotiq)  
[3] [MoveIt](https://moveit.ros.org)
