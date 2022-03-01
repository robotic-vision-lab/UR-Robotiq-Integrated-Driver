# RVL UR-Robotiq API

Due to how ROS structured Python source code and workspace setup, the module/package
terminology usage might be confusing here.

## Imports

```python
from rvl_robotiq_controller.RobotiqController import Robotiq2FController
from rvl_ur_remote_dashboard.URRemoteDashboard import URRemoteDashboard
from rvl_ur_motion_planner.URMoveitCommander import URCommander
```

These are the primary modules and classes that covers most of the use case. The supporting
functions might provide additional insights and is accessible as well.

`Robotiq2FController` is a wrapper to control Robotiq 2F grippers.

`URRemoteDashboard` is a wrapper for handling services and status monitoring as described on 
[ROS Interface page in UR Driver Repository](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/c3378599d5fa73a261328b326392e847f312ab6b/ur_robot_driver/doc/ROS_INTERFACE.md).

`URCommander` is a wrapper for `MoveIt!` with some function built-in (pose goal, joint goal, etc.). Additionals features are planned in future releases.

## Modules Quick Access

These are links to quickly access modules mentioned under Import section. Otherwise, the full
documentation of the entire driver is in the next section. Documentation will continue to be updated
and covers more modules as the driver is being developed.

```{eval-rst}

:py:class:`rvl_robotiq_controller.RobotiqController.Robotiq2FController`

:py:class:`rvl_ur_remote_dashboard.URRemoteDashboard.URRemoteDashboard`

:py:class:`rvl_ur_motion_planner.URMoveitCommander.URCommander`
```

## Full Modules Documentation

### Gripper Control

```{toctree}
---
maxdepth: 2
---

../_modules/robotiq/modules.rst
```

### UR Dashboard Control

```{toctree}
---
maxdepth: 2
---

../_modules/universal_robots/modules.rst
```

### Utilities

```{toctree}
---
maxdepth: 2
---

../_modules/rvl_utils/modules.rst
```