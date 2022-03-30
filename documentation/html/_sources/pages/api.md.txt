# UR-Robotiq Integrated Driver API

Due to the way ROS structures Python source code and the workspace setup, the
module/package usage terminology may be confusing here.

## Imports

```python
from rvl_robotiq_controller.RobotiqController import Robotiq2FController
from rvl_ur_remote_dashboard.URRemoteDashboard import URRemoteDashboard
from rvl_ur_motion_planner.URMoveitCommander import URCommander
```

These are the primary modules and classes that cover most of the use cases. The
supporting functions might provide additional insights and are accessible as
well.

`Robotiq2FController` is a wrapper to control the Robotiq 2F grippers.

`URRemoteDashboard` is a wrapper for handling services and status monitoring as
described on the [ROS Interface page in the UR Driver Repository](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/c3378599d5fa73a261328b326392e847f312ab6b/ur_robot_driver/doc/ROS_INTERFACE.md).

`URCommander` is a wrapper for `MoveIt` with some functions built-in (e.g.,
pose goal, joint goal, etc.). Additional features are planned for future
releases.

## Modules Quick Access

The links below provide access to the modules mentioned under the Import
section. Full documentation of the entire driver is described in the next
section. This documentation will continue to be updated and cover more modules
as the driver is developed.

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
