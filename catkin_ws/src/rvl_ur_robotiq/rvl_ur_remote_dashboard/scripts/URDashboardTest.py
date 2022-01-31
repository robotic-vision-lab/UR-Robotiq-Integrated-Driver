import rospy
import sys

from rvl_ur_remote_dashboard.URRemoteDashboard import URRemoteDashboard
from rvl_ur_motion_planner.URMoveitCommander import URCommander

import moveit_commander

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur_remote_dashboard_node', anonymous=True)

u = URRemoteDashboard()
# u.power_off_arm()

# u.load_program('jerry_remote.installation', ptype='i')
# u.load_program('jerry_ext.urp', ptype='p')
# u.cold_boot()
# u.connect_dashboard()
# u.start_loaded_program()
u.stop_loaded_program()

# c = URCommander()
# c.home()
# c.go_to_preset_location('fancy_home')

# u = URRemoteDashboard()
# u.get_safety_mode()
# u.release_brakes()
# u.power_off_arm()