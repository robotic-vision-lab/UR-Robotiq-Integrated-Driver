import rospy

from rvl_utilities.CustomLogger import ColorLogger

# MoveIt!
import moveit_commander as mc
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from moveit_commander import MoveItCommanderException
from moveit_commander.conversions import pose_to_list, list_to_pose
from moveit_msgs.msg import DisplayTrajectory

# ROS standard classes
import geometry_msgs.msg

# Math
from math import pi, tau, dist, fabs, cos
from numpy import clip

class URCommander:
    def __init__(self, group_name = 'arm', speed = 0.1, accel = 0.1):
        self.logger = ColorLogger('UR MoveIt!')
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()

        # MoveGroup planning and execution class
        self.planner = MoveGroupCommander(group_name)
        self.planner.set_planning_time(10.0)
        self.planner.set_num_planning_attempts(10)
        self.planner.set_goal_position_tolerance(0.01)
        self.planner.set_goal_orientation_tolerance(0.05)
        self.planner.allow_looking(True)
        self.planner.allow_replanning(True)
        self.planner.set_max_velocity_scaling_factor(clip(speed, 0.0, 1.0))
        self.planner.set_max_acceleration_scaling_factor(clip(accel, 0.0, 1.0))

        # define locations
        self.preset_locations = self.define_preset_locations()

    def report(self):
        self.logger.log_warn(f'Status Report')
        self.logger.log_info(f'planning frame = {self.planner.get_planning_frame()}', indent = 1)
        self.logger.log_info(f'end-effector link = {self.planner.get_end_effector_link()}', indent = 1)
        self.logger.log_info(f'group names = {self.robot.get_group_names()}', indent = 1)
        self.logger.log_info(f'SRDF defined poses = {self.planner.get_named_targets()}', indent = 1)
        self.logger.log_info(f'memorized poses = {self.planner.get_remembered_joint_values()}', indent = 1)

    def home(self):
        joint_goal = self.planner.get_current_joint_values()
        joint_goal[0] = 0.0
        joint_goal[1] = -pi/2.0
        joint_goal[2] = 0.0
        joint_goal[3] = -pi/2.0
        joint_goal[4] = 0.0
        joint_goal[5] = 0.0
        self.planner.go(joint_goal, wait=True)
        self.planner.stop()

    def go_to_preset_location(self, name):
        if name not in self.preset_locations:
            self.logger.log_error(f'location {name} is undefined')
        else:
            joint_goal = self.planner.get_current_joint_values()
            joint_goal[:] = self.preset_locations[name][:]
            self.planner.go(joint_goal, wait=True)
            self.planner.stop()

    # https://github.com/ros-planning/moveit_tutorials/blob/d366a8cc78fa582b68de407e969e970ff70122c0/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py#L72
    def all_close(self, goal, actual, tolerance):
        """
        Convenience method for testing if the values in two lists are within a tolerance of each other.
        For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
        between the identical orientations q and -q is calculated correctly).
        @param: goal       A list of floats, a Pose or a PoseStamped
        @param: actual     A list of floats, a Pose or a PoseStamped
        @param: tolerance  A float
        @returns: bool
        """
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    return False

        elif type(goal) is geometry_msgs.msg.PoseStamped:
            return self.all_close(goal.pose, actual.pose, tolerance)

        elif type(goal) is geometry_msgs.msg.Pose:
            x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
            x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
            # Euclidean distance
            d = dist((x1, y1, z1), (x0, y0, z0))
            # phi = angle between orientations
            cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
            return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

        return True

    def define_preset_locations(self):
        locations = {}

        name = 'fancy_home'
        joints = [0.0,           # shoulder pan
                  -pi / 4.0,     # shoulder lift
                  -pi * 3.0/4.0, # elbow
                  -pi / 2.0,     # wrist 1
                  0.0,           # wrist 2
                  pi / 2.0]      # wrist 3
        locations[name] = tuple(joints)

        return locations
