from enum import Enum

# UR Messages and Services
from ur_msgs.srv import *
from ur_msgs.msg import *
from ur_dashboard_msgs.msg import *
from ur_dashboard_msgs.srv import *

# Standard ROS Types
from geometry_msgs.msg import Vector3
from rosgraph_msgs.msg import Log
from std_msgs.msg import String
from std_srvs.srv import Trigger

from numpy import clip

class RobotModeMapping(Enum):
    NO_CONTROLLER=-1
    DISCONNECTED=0
    CONFIRM_SAFETY=1
    BOOTING=2
    POWER_OFF=3
    POWER_ON=4
    IDLE=5
    BACKDRIVE=6
    RUNNING=7
    UPDATING_FIRMWARE=8

class SafetyModeMapping(Enum):
    NORMAL=1
    REDUCED=2
    PROTECTIVE_STOP=3
    RECOVERY=4
    SAFEGUARD_STOP=5
    SYSTEM_EMERGENCY_STOP=6
    ROBOT_EMERGENCY_STOP=7
    VIOLATION=8
    FAULT=9
    VALIDATE_JOINT_ID=10
    UNDEFINED_SAFETY_MODE=11
    AUTOMATIC_MODE_SAFEGUARD_STOP=12
    SYSTEM_THREE_POSITION_ENABLING_STOP=13

class SetIOFunctionMapping(Enum):
    # FUN_SET_DIGITAL_OUT = 1
    # FUN_SET_FLAG = 2
    # FUN_SET_ANALOG_OUT = 3
    # FUN_SET_TOOL_VOLTAGE = 4

    SET_DIGITAL_OUT = 1
    SET_FLAG = 2
    SET_ANALOG_OUT = 3
    SET_TOOL_VOLTAGE = 4

class SetIOPinState(Enum):
    OFF = 0
    ON = 1

class SetIOToolState(Enum):
    TOOL_VOLTAGE_0V = 0
    TOOL_VOLTAGE_12V = 12
    TOOL_VOLTAGE_24V = 24

class SetIOPinMapping(Enum):
    pass