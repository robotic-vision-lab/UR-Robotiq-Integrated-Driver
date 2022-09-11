#!/usr/bin/env python3

import rospy
import sys

from rvl_ur_remote_dashboard.URRemoteDashboard import URRemoteDashboard
from rvl_ur_motion_planner.URMoveitCommander import URCommander
from rvl_robotiq_controller.RobotiqController import Robotiq2FController

# init node
import moveit_commander
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur_remote_dashboard_node', anonymous=True)

# connect to UR dashboard.
# dashboard = URRemoteDashboard()
# dashboard.connect_dashboard()

# # load installation and start program
# dashboard.load_program('jerry_remote_windows.installation', ptype='installation')
# dashboard.load_program('jerry_ext.urp', ptype='program')

# # release arm brakes
# # if arm is not powered, this will power the arm.
# dashboard.release_brakes()

# wait for user to press enter
# _ = input('Press any key to continue...')

# start program
# dashboard.start_loaded_program()

# wait for user to press enter
# _ = input('Press any key to continue...')

# commander example
# commander = URCommander()

# wait for user to press enter
# _ = input('Press any key to continue...')

# move to pose
# commander.go_to_preset_location('fancy_home')

# wait for user to press enter
# _ = input('Press any key to continue...')

# connect to gripper
gripper = Robotiq2FController(stroke=85, initialize=True, startup_reset=True)

# wait for user to press enter
# you may want to place a solid object between the jaws of the gripper now
_ = input('Press any key to continue...')

# auto-close until fully closed or obstructed
gripper.auto_close(alt_force=1)

# wait for user to press enter
# you may want to place a solid object between the jaws of the gripper now
_ = input('Press any key to continue...')

# auto-close until fully closed or obstructed
gripper.auto_open()