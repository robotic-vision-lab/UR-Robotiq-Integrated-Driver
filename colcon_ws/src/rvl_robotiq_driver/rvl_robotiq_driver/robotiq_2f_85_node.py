import rclpy

import time

from rclpy.node import Node

from rvl_robotiq_driver.robotiq_modbus_server import RobotiqRTUClient
from rvl_robotiq_msgs.msg import Robotiq2FStatus, Robotiq2FCommand
from rvl_robotiq_msgs.srv import RobotiqRequestStatus, RobotiqRequestPosition, RobotiqRequestOpening

from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState

class Robotiq2F85ControllerNode(Node):
    def __init__(self):
        super().__init__('robotiq_2f_85_controller_node')
        self.communicator:RobotiqRTUClient = RobotiqRTUClient()
        self.activated:bool = False
        self.status:list = []

        # attempt to connect to robotiq gripper via RS485
        self.get_logger().warn('Waiting for robotiq connection...')
        while self.communicator.connect('/tmp/ttyUR') is False:
            self.get_logger().info('Waiting for connection...')
            time.sleep(1)

        # should be connected now, wait a bit
        time.sleep(1)
        self.get_logger().info('Connected to robotiq gripper!')

        # publishers
        self.status_timer = self.create_timer(1.0/500.0, callback=self.status_timer_callback)
        self.status_publisher = self.create_publisher(Robotiq2FStatus, '/robotiq/status', 10)
        self.joint_states_publisher = self.create_publisher(JointState, '/robotiq/joint_states', 10)

        # services
        self.activation_service = self.create_service(Trigger, '/robotiq/activate', self.activate_gripper_callback)
        self.reactivate_service = self.create_service(Trigger, '/robotiq/reactivate', self.reactivate_gripper_callback)
        self.status_service     = self.create_service(RobotiqRequestStatus, '/robotiq/request_status', self.request_status_callback)
        self.request_position_manual_service = self.create_service(RobotiqRequestPosition, '/robotiq/set_position', self.request_position_manual_callback)
        self.request_opening_manual_service  = self.create_service(RobotiqRequestOpening,  '/robotiq/set_opening',  self.request_opening_manual_callback)
        self.auto_open_fragile_service  = self.create_service(Trigger, '/robotiq/auto_open/fragile', self.auto_open_fragile_callback)
        self.auto_open_soft_service     = self.create_service(Trigger, '/robotiq/auto_open/soft', self.auto_open_soft_callback)
        self.auto_open_medium_service   = self.create_service(Trigger, '/robotiq/auto_open/medium', self.auto_open_medium_callback)
        self.auto_open_hard_service     = self.create_service(Trigger, '/robotiq/auto_open/strong', self.auto_open_hard_callback)
        self.auto_close_fragile_service = self.create_service(Trigger, '/robotiq/auto_close/fragile', self.auto_close_fragile_callback)
        self.auto_close_soft_service    = self.create_service(Trigger, '/robotiq/auto_close/soft', self.auto_close_soft_callback)
        self.auto_close_medium_service  = self.create_service(Trigger, '/robotiq/auto_close/medium', self.auto_close_medium_callback)
        self.auto_close_hard_service    = self.create_service(Trigger, '/robotiq/auto_close/strong', self.auto_close_hard_callback)

    def request_position_manual_callback(self, request:RobotiqRequestPosition.Request, response:RobotiqRequestPosition.Response):
        # check if gripper is activated
        if self.activated is False:
            response.success = False
            response.message = 'Gripper is not activated! Command is skipped.'
            return response
        # generate command message
        command = Robotiq2FCommand()
        command.activate = 1
        command.goto_action = 1
        command.trigger_autorelease = 0
        command.autorelease_direction = 0
        # clip all given values to valid range (0-255)
        # check if position value is valid
        if request.position > 255: command.position = 255
        elif request.position < 0: command.position = 0
        else: command.position = request.position
        # check if speed value is valid
        if request.speed > 255: command.speed = 255
        elif request.speed < 0: command.speed = 0
        else: command.speed = request.speed
        # check if force value is valid
        if request.force > 255: command.force = 255
        elif request.force < 0: command.force = 0
        else: command.force = request.force
        # convert command to binary
        command_binary = self.generate_binary_command_from_2f_msg(command)
        # build the reponse
        response = RobotiqRequestPosition.Response()
        response.success = self.communicator.send_command(command_binary)
        if response.success:
            response.message = 'Position request successful!'
            self.get_logger().info(response.message)
        else:
            response.message = 'Position request failed!'
            self.get_logger().error(response.message)
        return response

    def request_opening_manual_callback(self, request:RobotiqRequestOpening.Request,
                                              response:RobotiqRequestOpening.Response,
                                              measured_error:float=1.9):
        # check if gripper is activated
        if self.activated is False:
            response.success = False
            response.message = 'Gripper is not activated! Command is skipped.'
            return response
        # generate command message
        command = Robotiq2FCommand()
        command.activate = 1
        command.goto_action = 1
        command.trigger_autorelease = 0
        command.autorelease_direction = 0
        # convert opening in mm to positions per user manual
        # 1 step = 0.4mm (or 0.3333333)?
        command.position = self.map_opening_to_position(request.opening + measured_error)
        # clip all given values to valid range (0-255)
        # check if speed value is valid
        if request.speed > 255: command.speed = 255
        elif request.speed < 0: command.speed = 0
        else: command.speed = request.speed
        # check if force value is valid
        if request.force > 255: command.force = 255
        elif request.force < 0: command.force = 0
        else: command.force = request.force
        # convert command to binary
        command_binary = self.generate_binary_command_from_2f_msg(command)
        # build the reponse
        response = RobotiqRequestOpening.Response()
        response.success = self.communicator.send_command(command_binary)
        if response.success:
            response.message = 'Opening request successful!'
            self.get_logger().info(response.message)
        else:
            response.message = 'Opening request failed!'
            self.get_logger().error(response.message)
        return response

    def auto_open_fragile_callback(self, _, response:Trigger.Response):
        # check if gripper is activated
        if self.activated is False:
            response.success = False
            response.message = 'Gripper is not activated! Command is skipped.'
            return response
        # generate command message
        command = Robotiq2FCommand()
        command.activate = 1
        command.goto_action = 1
        command.trigger_autorelease = 0
        command.autorelease_direction = 0
        # setting for fragile requires force 0, slow speed is more of an opinion
        command.position = 0
        command.speed = 50
        command.force = 0
        # convert command to binary
        command_binary = self.generate_binary_command_from_2f_msg(command)
        # build the reponse
        response = Trigger.Response()
        response.success = self.communicator.send_command(command_binary)
        if response.success:
            response.message = 'FRAGILE auto-open request successful!'
            self.get_logger().info(response.message)
        else:
            response.message = 'FRAGILE auto-open request failed!'
            self.get_logger().error(response.message)
        return response

    def auto_open_soft_callback(self, _, response:Trigger.Response):
        # check if gripper is activated
        if self.activated is False:
            response.success = False
            response.message = 'Gripper is not activated! Command is skipped.'
            return response
        # generate command message
        command = Robotiq2FCommand()
        command.activate = 1
        command.goto_action = 1
        command.trigger_autorelease = 0
        command.autorelease_direction = 0
        # setting for soft requires force 1, slow speed is more of an opinion
        command.position = 0
        command.speed = 50
        command.force = 1
        # convert command to binary
        command_binary = self.generate_binary_command_from_2f_msg(command)
        # build the reponse
        response = Trigger.Response()
        response.success = self.communicator.send_command(command_binary)
        if response.success:
            response.message = 'SOFT auto-open request successful!'
            self.get_logger().info(response.message)
        else:
            response.message = 'SOFT auto-open request failed!'
            self.get_logger().error(response.message)
        return response

    def auto_open_medium_callback(self, _, response:Trigger.Response):
        # check if gripper is activated
        if self.activated is False:
            response.success = False
            response.message = 'Gripper is not activated! Command is skipped.'
            return response
        # generate command message
        command = Robotiq2FCommand()
        command.activate = 1
        command.goto_action = 1
        command.trigger_autorelease = 0
        command.autorelease_direction = 0
        # setting for medium requires force ~100, medium speed is more of an opinion
        command.position = 0
        command.speed = 127
        command.force = 100
        # convert command to binary
        command_binary = self.generate_binary_command_from_2f_msg(command)
        # build the reponse
        response = Trigger.Response()
        response.success = self.communicator.send_command(command_binary)
        if response.success:
            response.message = 'MEDIUM auto-open request successful!'
            self.get_logger().info(response.message)
        else:
            response.message = 'MEDIUM auto-open request failed!'
            self.get_logger().error(response.message)
        return response

    def auto_open_hard_callback(self, _, response:Trigger.Response):
        # check if gripper is activated
        if self.activated is False:
            response.success = False
            response.message = 'Gripper is not activated! Command is skipped.'
            return response
        # generate command message
        command = Robotiq2FCommand()
        command.activate = 1
        command.goto_action = 1
        command.trigger_autorelease = 0
        command.autorelease_direction = 0
        # setting for strong requires maximum, maximum speed is more of an opinion
        command.position = 0
        command.speed = 255
        command.force = 255
        # convert command to binary
        command_binary = self.generate_binary_command_from_2f_msg(command)
        # build the reponse
        response = Trigger.Response()
        response.success = self.communicator.send_command(command_binary)
        if response.success:
            response.message = 'STRONG auto-open request successful!'
            self.get_logger().info(response.message)
        else:
            response.message = 'STRONG auto-open request failed!'
            self.get_logger().error(response.message)
        return response

    def auto_close_fragile_callback(self, _, response:Trigger.Response):
        # check if gripper is activated
        if self.activated is False:
            response.success = False
            response.message = 'Gripper is not activated! Command is skipped.'
            return response
        # generate command message
        command = Robotiq2FCommand()
        command.activate = 1
        command.goto_action = 1
        command.trigger_autorelease = 0
        command.autorelease_direction = 0
        # setting for fragile requires force 0, slow speed is more of an opinion
        command.position = 255
        command.speed = 50
        command.force = 0
        # convert command to binary
        command_binary = self.generate_binary_command_from_2f_msg(command)
        # build the reponse
        response = Trigger.Response()
        response.success = self.communicator.send_command(command_binary)
        if response.success:
            response.message = 'FRAGILE auto-close request successful!'
            self.get_logger().info(response.message)
        else:
            response.message = 'FRAGILE auto-close request failed!'
            self.get_logger().error(response.message)
        return response

    def auto_close_soft_callback(self, _, response:Trigger.Response):
        # check if gripper is activated
        if self.activated is False:
            response.success = False
            response.message = 'Gripper is not activated! Command is skipped.'
            return response
        # generate command message
        command = Robotiq2FCommand()
        command.activate = 1
        command.goto_action = 1
        command.trigger_autorelease = 0
        command.autorelease_direction = 0
        # setting for soft requires force 1, slow speed is more of an opinion
        command.position = 255
        command.speed = 50
        command.force = 1
        # convert command to binary
        command_binary = self.generate_binary_command_from_2f_msg(command)
        # build the reponse
        response = Trigger.Response()
        response.success = self.communicator.send_command(command_binary)
        if response.success:
            response.message = 'SOFT auto-close request successful!'
            self.get_logger().info(response.message)
        else:
            response.message = 'SOFT auto-close request failed!'
            self.get_logger().error(response.message)
        return response

    def auto_close_medium_callback(self, _, response:Trigger.Response):
        # check if gripper is activated
        if self.activated is False:
            response.success = False
            response.message = 'Gripper is not activated! Command is skipped.'
            return response
        # generate command message
        command = Robotiq2FCommand()
        command.activate = 1
        command.goto_action = 1
        command.trigger_autorelease = 0
        command.autorelease_direction = 0
        # setting for medium requires force ~100, medium speed is more of an opinion
        command.position = 255
        command.speed = 127
        command.force = 100
        # convert command to binary
        command_binary = self.generate_binary_command_from_2f_msg(command)
        # build the reponse
        response = Trigger.Response()
        response.success = self.communicator.send_command(command_binary)
        if response.success:
            response.message = 'MEDIUM auto-close request successful!'
            self.get_logger().info(response.message)
        else:
            response.message = 'MEDIUM auto-close request failed!'
            self.get_logger().error(response.message)
        return response

    def auto_close_hard_callback(self, _, response:Trigger.Response):
        # check if gripper is activated
        if self.activated is False:
            response.success = False
            response.message = 'Gripper is not activated! Command is skipped.'
            return response
        # generate command message
        command = Robotiq2FCommand()
        command.activate = 1
        command.goto_action = 1
        command.trigger_autorelease = 0
        command.autorelease_direction = 0
        # setting for strong requires maximum, maximum speed is more of an opinion
        command.position = 255
        command.speed = 255
        command.force = 255
        # convert command to binary
        command_binary = self.generate_binary_command_from_2f_msg(command)
        # build the reponse
        response = Trigger.Response()
        response.success = self.communicator.send_command(command_binary)
        if response.success:
            response.message = 'STRONG auto-close request successful!'
            self.get_logger().info(response.message)
        else:
            response.message = 'STRONG auto-close request failed!'
            self.get_logger().error(response.message)
        return response

    def request_status_callback(self, _, response):
        response = RobotiqRequestStatus.Response()
        if self.status is not None:
            response.status = self.generate_2f_status_from_binary(self.status)
            response.success = True
            response.message = 'Status request successful!'
            return response
        else:
            response.status = None
            response.success = False
            response.message = 'Status request failed!'
            return response

    def activate_gripper_callback(self, _, response):
        response = Trigger.Response()
        command = Robotiq2FCommand()
        command.activate = 0
        command_binary = self.generate_binary_command_from_2f_msg(command)
        response.success = self.communicator.send_command(command_binary)
        time.sleep(0.5)
        command.activate = 1
        command_binary = self.generate_binary_command_from_2f_msg(command)
        self.communicator.send_command(command_binary)
        time.sleep(1.0)
        response.success &= self.communicator.send_command(command_binary)
        if response.success:
            self.activated = True
            response.message = 'Gripper activated!'
            self.get_logger().info(response.message)
        else:
            self.activated = False
            response.message = 'Gripper activation failed!'
            self.get_logger().error(response.message)
        return response

    def reactivate_gripper_callback(self, _, response):
        response = Trigger.Response()
        command = Robotiq2FCommand()
        command.activate = 0
        command_binary = self.generate_binary_command_from_2f_msg(command)
        response.success = self.communicator.send_command(command_binary)
        time.sleep(0.5)
        command.activate = 1
        command_binary = self.generate_binary_command_from_2f_msg(command)
        self.communicator.send_command(command_binary)
        time.sleep(1.0)
        response.success &= self.communicator.send_command(command_binary)
        if response.success:
            self.activated = True
            response.message = 'Gripper reactivated!'
            self.get_logger().info(response.message)
        else:
            self.activated = False
            response.message = 'Gripper reactivation failed!'
            self.get_logger().error(response.message)
        return response

    def status_timer_callback(self):
        self.status = self.communicator.request_status()
        if self.status is not None:
            # convert raw status to 2F status message
            current_status = self.generate_2f_status_from_binary(self.status)

            # publish status message as whole Robotiq 2F status message
            self.status_publisher.publish(current_status)

            # calculate joint_states based on current position from raw status
            # multiply by mimic factor as defined in URDF
            joint_states = JointState()
            joint_states.header.stamp = self.get_clock().now().to_msg()
            joint_states.name = ['left_knuckle_joint',
                                 'left_inner_knuckle_joint',
                                 'left_finger_tip_joint',
                                 'right_knuckle_joint',
                                 'right_inner_knuckle_joint',
                                 'right_finger_tip_joint']
            joint_position = self.map_position_to_joint_angle(current_status.current_position)
            joint_states.position = [-joint_position,
                                      joint_position,
                                     -joint_position,
                                      joint_position,
                                      joint_position,
                                      joint_position]
            joint_states.velocity = [0.0 for _ in range(6)]
            joint_states.effort   = [0.0 for _ in range(6)]
            self.joint_states_publisher.publish(joint_states)

    def generate_2f_status_from_binary(self, binary:list):
        status = Robotiq2FStatus()
        status.activated         = (binary[0] >> 0) & 0x01
        status.action_status     = (binary[0] >> 3) & 0x01
        status.gripper_status    = (binary[0] >> 4) & 0x03
        status.object_status     = (binary[0] >> 6) & 0x03
        status.fault_status      =  binary[2]
        status.position_request  =  binary[3]
        status.current_position  =  binary[4]
        status.motor_current     =  binary[5]
        return status

    def generate_binary_command_from_2f_msg(self, message: Robotiq2FCommand):
        command = []
        # byte 0: action request
        command.append(message.activate + (message.goto_action << 3) + (message.trigger_autorelease << 4) + (message.autorelease_direction << 5))
        # byte 1 and 2 (reserved)
        command.append(0)
        command.append(0)
        # byte 3: position request
        command.append(message.position)
        # byte 4: speed
        command.append(message.speed)
        # byte 5: force
        command.append(message.force)
        return command

    def linear_map(self, value, from_range, to_range):
        (a, b) = from_range
        (c, d) = to_range
        return ((value - a) / (b - a)) * (d - c) + c

    def map_opening_to_position(self, opening:float, calibrated_low:int = 3, calibrated_high:int = 228):
        if opening > 85.0: opening = 85.0
        if opening <  0.0: opening = 0.0
        return int(self.linear_map(opening, (0.0, 85.0), (calibrated_high, calibrated_low)))

    def map_position_to_opening(self, position:int, calibrated_low:int = 3, calibrated_high:int = 228):
        if position > 255: position = 255
        if position <   0: position = 0
        return self.linear_map(position, (calibrated_high, calibrated_low), (0.0, 85.0))

    def map_position_to_joint_angle(self, position:int, calibrated_low:int = 3, calibrated_high:int = 228):
        if position > 255: position = 255
        if position <   0: position = 0
        return self.linear_map(position, (calibrated_low, calibrated_high), (0.0, 0.8))

def main(args=None):
    rclpy.init(args=args)
    controller_node = Robotiq2F85ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
