#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from pymodbus.exceptions import ModbusIOException

from rvl_robotiq_controller.msg import Robotiq2FCommand, Robotiq2FStatus
from rvl_robotiq_controller.Robotiq2FSupport import generate_binary_command_from_2f_msg, generate_2f_status_from_binary, raw_to_rad_2f85
from rvl_utilities.CustomLogger import ColorLogger
from rvl_robotiq_modbus_server.RobotiqModbusServer import RobotiqRTUClient

from ur_dashboard_msgs.msg import RobotMode

ur_mode = -1

logger = ColorLogger('R2F85 Node')


def publish_gripper_joint_state(status_msg, publisher):
    gripper_joint_state = JointState()
    gripper_joint_state.header = Header()
    gripper_joint_state.header.stamp = rospy.Time.now()
    gripper_joint_state.name = ['finger_joint']
    gripper_joint_state.position = [raw_to_rad_2f85(status_msg.current_position)]
    gripper_joint_state.effort = [0.0]
    gripper_joint_state.velocity = [0.0]

    publisher.publish(gripper_joint_state)

def send_command_to_gripper(command_msg, communicator:RobotiqRTUClient):
    communicator.send_command(generate_binary_command_from_2f_msg(command_msg))

def update_robot_mode(msg):
    global ur_mode
    ur_mode = msg.mode

def start_robotiq_backend(device, refresh_rate = 200, delayed_start = 5):
    global logger
    gripper = RobotiqRTUClient()
    gripper.connect(device)
    limiter = rospy.Rate(hz = refresh_rate)

    # publish joint state for MoveIt!
    joint_state_publisher = rospy.Publisher('joint_states', JointState, queue_size=1)

    # status handler
    status_publisher = rospy.Publisher('/Robotiq2F/gripper_status', Robotiq2FStatus, queue_size=200)
    status_monitor = rospy.Subscriber('/Robotiq2F/gripper_status', Robotiq2FStatus, callback=lambda msg:publish_gripper_joint_state(msg, joint_state_publisher))

    # command handler
    command_publisher = rospy.Publisher('/Robotiq2F/command_interface', Robotiq2FCommand, queue_size=10)
    command_monitor = rospy.Subscriber('/Robotiq2F/command_interface', Robotiq2FCommand, callback=lambda msg: send_command_to_gripper(msg, gripper))

    # UR power monitor
    power_monitor = rospy.Subscriber('/ur_hardware_interface/robot_mode', RobotMode, callback=update_robot_mode)

    logger.log_warn(f'Waiting for {delayed_start} seconds so publishers and subscribers register correctly')
    rospy.sleep(delayed_start)

    # first boot
    if ur_mode < 5:
        logger.log_warn(f'Waiting for UR Arm to power on...')
        while ur_mode < 5 and not rospy.is_shutdown():
            rospy.sleep(1)
        logger.log_warn('UR Arm powered, updating and accepting command')
    else:
        logger.log_warn('UR Arm already powered, updating and accepting command')
    rospy.sleep(3)
    logger.log_success('Gripper interface node is now running')

    # UR power dependent
    while not rospy.is_shutdown():
        if ur_mode < 5:
            logger.log_warn('UR Arm power lost, sleeping...')
            while ur_mode < 5 and not rospy.is_shutdown():
                limiter.sleep()
            # wait a little bit before updating again
            logger.log_success('UR Arm repowered, updating and accepting command')
            rospy.sleep(3)
        else:
            gripper_status = generate_2f_status_from_binary(gripper.request_status())
            status_publisher.publish(gripper_status)
        limiter.sleep()

    # # doesn't check UR or gripper power state
    # # hence, noisy when gripper is not powered or connected
    # while not rospy.is_shutdown():
    #     try:
    #         gripper_status = generate_2f_status_from_binary(gripper.request_status())
    #     except ModbusIOException as e:
    #         rospy.sleep(1)
    #         continue

    #     # only publish when gripper actually replies
    #     status_publisher.publish(gripper_status)
    #     limiter.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('robotiq_interface_node', anonymous=True)
        start_robotiq_backend(device=rospy.get_param('/ur_tool_communication_bridge/device_name', default='/tmp/ttyUR'))
    except Exception as e:
        rospy.logerr(e)