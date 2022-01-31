import rospy
from rospy import ROSException

from numpy import clip, polyfit, poly1d
from typing import Union, Tuple
from math import isclose

from rvl_robotiq_controller.msg import Robotiq2FCommand, Robotiq2FStatus
from Robotiq2FSupport import fault_mapping
from rvl_utilities.CustomLogger import ColorLogger

from ur_dashboard_msgs.msg import RobotMode

class Robotiq2FController:
    def __init__(self, stroke: int, default_force: int = 100, default_speed: int = 100, initialize: bool = False, startup_reset: bool = False, calibrate: bool = False, bypass_power: bool = False) -> None:
        """Controller for basic Robotiq 2F Gripper operation.

        Args:
            stroke (int): Stroke of the gripper. 2F grippers must use either 85mm or 140mm.
            default_force (int, optional): Default gripping force. Defaults to 100.
            default_speed (int, optional): Default gripping speed. Defaults to 100.
            initialize (bool, optional): Register ROS publisher and subscriber on object instantiation. Defaults to False.
            startup_reset (bool, optional): Invoke internal reset on instantiation. Defaults to False.
            calibrate (bool, optional): Adjust binary limits of gripper. Defaults to False.
            bypass_power (bool, optional): Ignoring power state monitor. Useful when gripper not attached to UR tool port. Defaults to False.

        Raises:
            ValueError: Invalid stroke width, must be 85mm or 140mm
        """
        
        # check definition
        if stroke != 85 and stroke != 140:
            raise ValueError('2F Gripper stroke must be 85 or 140 mm')
        else:
            self.stroke = stroke
            self.logger = ColorLogger(label = 'R2F' + str(self.stroke) + ' Controller')

        # factory preset (page 25 of user manual)
        self.force = int(clip(default_force, 0, 255))
        self.speed = int(clip(default_speed, 0, 255))

        # NBR Finger (page 121 of user manual)
        # self.finger_thickness = finger_thickness # mm

        # mechanical specification (page 123 of user manual)
        if self.stroke == 85:
            self.force_range = (20.0, 235.0) # Newton
            self.speed_range = (20.0, 150.0) # mm/s
        else:
            self.force_range = (10.0, 125.0) # Newton
            self.speed_range = (30.0, 250.0) # mm/s

        # recorded limit of gripper binary range
        self.binary_range = tuple([int(rospy.get_param('/robotiq_2f_85_calibration/lower_binary', default =   0)),
                                   int(rospy.get_param('/robotiq_2f_85_calibration/upper_binary', default = 255))])

        # status keeping
        self.status = None
        self.power_status = -1
        self.power_bypass = bypass_power

        if initialize:
            self.register()
            if startup_reset:
                self.reset()
            if calibrate:
                self.binary_range = self.calibrate()

        # conversion ratio for force, speed, and distance (metrics to binary)
        self.f_ratio = poly1d(polyfit(self.force_range, (0, 255), 1))
        self.s_ratio = poly1d(polyfit(self.speed_range, (0, 255), 1))
        self.d_ratio = poly1d(polyfit((self.stroke, 0), self.binary_range, 1))
        self.inv_f   = poly1d(polyfit((0, 255), self.force_range, 1))
        self.inv_s   = poly1d(polyfit((0, 255), self.speed_range, 1))
        self.inv_d   = poly1d(polyfit(self.binary_range, (self.stroke, 0), 1))
        
    # ---------------------------------------------------------------------------- #
    #                               GRIPPER ACTUATION                              #
    # ---------------------------------------------------------------------------- #

    def auto_close(self, alt_speed: int = None, alt_force: int = None, blocking: bool = True) -> None:
        """Fully close the gripper or until obstructed.

        Args:
            alt_speed (int, optional): Override internal speed settings. Defaults to None.
            alt_force (int, optional): Override internal force settings. Defaults to None.
            blocking (bool, optional): Wait until gripper motion is completed. Defaults to True.
        """
        command = Robotiq2FCommand()
        command.activate = 1
        command.goto = 1
        command.position = 255
        command.speed = self.speed if alt_speed is None else int(clip(alt_speed, 0, 255))
        command.force = self.force if alt_force is None else int(clip(alt_force, 0, 255))
        self.compensated_publish(command)
        if blocking:
            self.block()

    def auto_open(self, alt_speed: int = None, alt_force: int = None, blocking: bool = True) -> None:
        """Fully open the gripper or until obstructed.

        Args:
            alt_speed (int, optional): Override internal speed settings. Defaults to None.
            alt_force (int, optional): Override internal force settings. Defaults to None.
            blocking (bool, optional): Wait until gripper motion is completed. Defaults to True.
        """
        command = Robotiq2FCommand()
        command.activate = 1
        command.goto = 1
        command.position = 255
        command.speed = self.speed if alt_speed is None else int(clip(alt_speed, 0, 255))
        command.force = self.force if alt_force is None else int(clip(alt_force, 0, 255))
        self.compensated_publish(command)
        if blocking:
            self.block()

    def grasp_soft(self, opening: bool = False, alt_speed: int = None, blocking: bool = True) -> None:
        """Soft grasp preset, grasping with force set to 1 (approximately 20 N).

        Args:
            opening (bool, optional): Grasp in the opening direction i.e. internal grasp. Defaults to False.
            alt_speed (int, optional): Override internal speed settings. Defaults to None.
            blocking (bool, optional): Wait until gripper motion is completed. Defaults to True.
        """
        command = Robotiq2FCommand()
        command.activate = 1
        command.goto = 1
        command.position = 0
        command.speed = self.speed if alt_speed is None else int(clip(alt_speed, 0, 255))
        command.force = 0
        self.compensated_publish(command)
        if blocking:
            self.block()
    
    def grasp_soft_regrasp(self, opening: bool = False, alt_speed: int = None, blocking: bool = True) -> None:
        """Soft grasp preset, grasping with force set to 1 (approximately 20 N) with Re-Grasp enabled.

        Args:
            opening (bool, optional): Grasp in the opening direction i.e. internal grasp. Defaults to False.
            alt_speed (int, optional): Override internal speed settings. Defaults to None.
            blocking (bool, optional): Wait until gripper motion is completed. Defaults to True.
        """
        command = Robotiq2FCommand()
        command.activate = 1
        command.goto = 1
        command.position = 0 if opening else 255
        command.speed = self.speed if alt_speed is None else int(clip(alt_speed, 0, 255))
        command.force = 1
        self.compensated_publish(command)
        if blocking:
            self.block()

    def grasp_medium(self, opening: bool = False, alt_speed: int = None, blocking: bool = True) -> None:
        """Medium grasp preset, grasping with force set to 128 (approximately 128 N).

        Args:
            opening (bool, optional): Grasp in the opening direction i.e. internal grasp. Defaults to False.
            alt_speed (int, optional): Override internal speed settings. Defaults to None.
            blocking (bool, optional): Wait until gripper motion is completed. Defaults to True.
        """
        command = Robotiq2FCommand()
        command.activate = 1
        command.goto = 1
        command.position = 0 if opening else 255
        command.speed = self.speed if alt_speed is None else int(clip(alt_speed, 0, 255))
        command.force = 128
        self.compensated_publish(command)
        if blocking:
            self.block()

    def grasp_hard(self, opening: bool = False, alt_speed: int = None, blocking: bool = True) -> None:
        """Hard grasp preset, grasping with force set to maximum or 255 (approximately 235 N).

        Args:
            opening (bool, optional): Grasp in the opening direction i.e. internal grasp. Defaults to False.
            alt_speed (int, optional): Override internal speed settings. Defaults to None.
            blocking (bool, optional): Wait until gripper motion is completed. Defaults to True.
        """
        command = Robotiq2FCommand()
        command.activate = 1
        command.goto = 1
        command.position = 0 if opening else 255
        command.speed = self.speed if alt_speed is None else int(clip(alt_speed, 0, 255))
        command.force = 255
        self.compensated_publish(command)
        if blocking:
            self.block()

    def open_gripper(self, value: Union[int, float], alt_speed: int = None, alt_force: int = None, unit: str = 'mm', blocking: bool = True) -> None:
        """Open/Close the gripper to specified gap between the gripper pads.

        Args:
            value (Union[int, float]): Width of the jaw opening.
            alt_speed (int, optional): Override internal speed settings. Defaults to None.
            alt_force (int, optional): Override internal force settings. Defaults to None.
            unit (str, optional): Unit of measurement. Defaults to 'mm'.
            blocking (bool, optional): Wait until gripper motion is completed. Defaults to True.
        """
        acceptable_units = ['mm', 'in', 'raw']
        if unit not in acceptable_units:
            self.logger.log_error(f'{unit} is invalid unit')
            self.logger.log_error(f'Expecting {acceptable_units}')
        else:
            command = Robotiq2FCommand()
            command.activate = 1
            command.goto = 1
            command.speed = self.speed if alt_speed is None else int(clip(alt_speed, 0, 255))
            command.force = self.force if alt_force is None else int(clip(alt_force, 0, 255))

            if unit == 'raw':
                command.position = int(clip(value, *self.binary_range))
            elif unit == 'mm':
                command.position = self.open_mm_to_raw(value)
            else:
                command.position = self.open_in_to_raw(value)

            self.compensated_publish(command)
            if blocking:
                self.block()

    def set_gripper_speed(self, value: Union[int, float], unit: str = 'mm/s') -> None:
        """Set the internal (default) speed setting of the gripper. 

        Args:
            value (Union[int, float]): New speed value.
            unit (str, optional): Unit of measurement. Defaults to 'mm/s'.
        """
        acceptable_units = ['raw', 'mm/s', 'in/s']
        if unit not in acceptable_units:
            self.logger.log_error(f'{unit} is invalid unit')
            self.logger.log_error(f'Expecting {acceptable_units}')
        else:
            if unit == 'raw':
                self.speed = int(clip(value, 0, 255))
            elif unit == 'mm/s':
                self.speed = self.mmps_to_raw_speed(value)
            else:
                self.speed = self.inps_to_raw_speed(value)

    def set_gripper_force(self, value: Union[int, float], unit: str = 'N') -> None:
        """Set the internal (default) force setting of the gripper. 

        Args:
            value (Union[int, float]): New force value.
            unit (str, optional): Unit of measurement. Defaults to 'N'.
        """
        acceptable_units = ['raw', 'newton', 'N', 'pound-force', 'lbf']
        if unit not in acceptable_units:
            self.logger.log_error(f'{unit} is invalid unit')
            self.logger.log_error(f'Expecting {acceptable_units}')
        else:
            if unit == 'raw':
                self.force = int(clip(value, 0, 255))
            elif unit == 'newton':
                self.force = self.newton_to_raw_force(value)
            else:
                self.force = self.lbf_to_raw_force(value)
    
    def send_raw_position_command(self, position: int, speed: int, force: int, blocking: bool = True) -> None:
        """Send a position request command to the gripper ignoring internal settings.

        Args:
            position (int): Raw position value [0-255].
            speed (int): Raw speed value [0-255].
            force (int): Raw force value [0-255].
            blocking (bool, optional): Wait until gripper motion is completed. Defaults to True.
        """
        command = Robotiq2FCommand()
        command.activate = 1
        command.goto = 1
        command.position = int(clip(position, 0, 255))
        command.speed = int(clip(speed, 0, 255))
        command.force = int(clip(force, 0, 255))
        self.compensated_publish(command)
        if blocking:
            self.block()
        
    # ---------------------------------------------------------------------------- #
    #                            GRIPPING/MOVING STATUS                            #
    # ---------------------------------------------------------------------------- #

    def is_moving(self) -> bool:
        """Returns true when the gripper is in motion"""
        return (self.status.object_status == 0)

    def is_holding(self) -> bool:
        """Returns true when the gripper is holding an object"""
        status = self.status.object_status
        if status == 1:
            self.logger.log_info('Object detected/holding while opening')
        elif status == 2:
            self.logger.log_info('Object detected/holding while closing')
        else:
            self.logger.log_info('No object detected')
        return (self.status.object_status == 1 or self.status.object_status == 2)
    
    # ---------------------------------------------------------------------------- #
    #                              ACTIVATION CONTROL                              #
    # ---------------------------------------------------------------------------- #

    def reset(self) -> None:
        """Reset the gripper to default state (may or may not be obstructed)."""
        self.logger.log_warn(f'Gripper reset')
        self.deactivate()
        self.activate()
        rospy.sleep(3)

    def deactivate(self) -> None:
        """Deactivate the gripper. Can also be used to clear the reset bit."""
        command = Robotiq2FCommand()
        command.activate = 0
        self.compensated_publish(command)
        rospy.sleep(1)

    def activate(self) -> None:
        """[summary]"""
        command = Robotiq2FCommand()
        command.activate = 1
        self.compensated_publish(command)
        rospy.sleep(1)
            
    # ---------------------------------------------------------------------------- #
    #                             SUPPORTING FUNCTIONS                             #
    # ---------------------------------------------------------------------------- #

    def report_status(self, verbose: bool = False) -> None:
        """Output the current state of the gripper.

        Args:
            verbose (bool, optional): Display additional raw status message. Defaults to False.
        """
        pos = self.status.current_position
        pos_mm = self.raw_to_open_mm(pos)
        pos_in = self.raw_to_open_in(pos)
        speed_mm = self.raw_speed_to_mmps(self.speed)
        speed_in = self.raw_speed_to_inps(self.speed)
        force_N = self.raw_force_to_newton(self.force)
        force_lbf = self.raw_force_to_lbf(self.force)
        fault_status = self.status.fault_status
        self.logger.log_info(f'==== Current status ====')
        self.logger.log_info(f'Opening  = {pos} [{pos_mm} mm ~ {pos_in} in]')
        self.logger.log_info(f'Speed    = {self.speed} [{speed_mm} mm/s ~ {speed_in} in/s]')
        self.logger.log_info(f'Force    = {self.force} [{force_N} N ~ {force_lbf} lbf]')
        self.logger.log_info(f'Fault    = {fault_mapping[fault_status] if fault_status in fault_mapping else "N/A"}')
        if verbose:
            self.logger.log_info('Raw Robotiq2FStatus message:')
            for s in str(self.status).split('\n'):
                self.logger.log_info(s)

    def register(self, timeout: int = 10) -> None:
        """Register necessary ROS publishers and subscribers.

        Args:
            timeout (int, optional): Wait time for topics to start publishing. Defaults to 10.
        """
        try:
            self.logger.log_warn(f'Registering publisher and subscriber')
            rospy.wait_for_message('/Robotiq2F/gripper_status', Robotiq2FStatus, timeout = timeout)
            self.status_monitor = rospy.Subscriber('/Robotiq2F/gripper_status', Robotiq2FStatus, callback = self.status_monitor_callback)
            self.power_monitor = rospy.Subscriber('/ur_hardware_interface/robot_mode', RobotMode, callback = self.power_monitor_callback)
            self.command_publisher = rospy.Publisher('/Robotiq2F/command_interface', Robotiq2FCommand, queue_size = 1)
            self.logger.log_warn(f'Waiting for {5} seconds so publishers and subscribers register correctly')
            rospy.sleep(5)
            self.logger.log_success(f'Controller registered successfully and ready to send commands')
        except (ROSException, KeyboardInterrupt) as error:
            self.logger.log_error('Unable to assign controller to gripper. Did you start Robotiq Node and powered the arm?')
            self.logger.log_error(error)
            raise ROSException

    def calibrate(self) -> Tuple[int, int]:
        """Fully open and closes the gripper to record internal binary. Must be executed without any obstructions as this can dramatically influence gripper operations!

        Returns:
            (int, int): lower and upper binary limit (default 0x00 to 0xFF or 0 to 255)
        """
        self.logger.log_warn('Calibrating binary limits')
        self.auto_open()
        rospy.sleep(3)
        upper = self.status.current_position
        rospy.sleep(1)
        self.auto_close()
        rospy.sleep(3)
        lower = self.status.current_position
        rospy.sleep(1)
        self.auto_open()
        self.logger.log_success('Calibration completed')
        self.logger.log_success(f'Position binary range limit is {(lower, upper)}')
        return tuple(lower, upper)
    
    def compensated_publish(self, command: Robotiq2FCommand, lag: float = 0.1):
        """Publish with a small wait between message for ROS topics to cope with the refresh rate of the gripper."""
        self.command_publisher.publish(command)
        rospy.sleep(lag)

    def status_monitor_callback(self, msg):
        """Callback for status subsciber"""
        self.status = msg

    def power_monitor_callback(self, msg):
        """Callback for power subsciber"""
        self.power_status = msg.mode

    def block(self) -> None:
        """Convenient snippet for idle looping until gripper finishes motion."""
        while self.is_moving():
            rospy.sleep(0.1)
            
    # ---------------------------------------------------------------------------- #
    #                              CONVERSION HELPERS                              #
    # ---------------------------------------------------------------------------- #

    def raw_speed_to_mmps(self, raw: int) -> float:
        """Convert raw value [0-255] to mm/s"""
        return round(self.inv_s(clip(raw, 0, 255)), 3)
    
    def raw_speed_to_inps(self, raw: int) -> float:
        """Convert raw value [0-255] to in/s"""
        return round(self.raw_speed_to_mmps(raw) / 25.4, 3)
    
    def mmps_to_raw_speed(self, mmps: Union[int, float]) -> int:
        """Convert mm/s to raw binary value [0-255]"""
        return int(round(self.s_ratio(clip(mmps, *self.speed_range))))
    
    def inps_to_raw_speed(self, inps: Union[int, float]) -> int:
        """Convert inches/s to raw binary value [0-255]"""
        return int(round(self.s_ratio(clip(inps * 25.4, *self.speed_range))))
    
    def raw_force_to_newton(self, raw: int) -> float:
        """Convert raw value [0-255] to N"""
        return round(self.inv_f(clip(raw, 0, 255)), 3)
    
    def raw_force_to_lbf(self, raw: int) -> float:
        """Convert raw value [0-255] to lbf"""
        return round(self.raw_force_to_newton(raw) * 0.224809, 3)

    def newton_to_raw_force(self, newton: Union[int, float]) -> int:
        """Convert N to raw binary value [0-255]"""
        return int(round(self.f_ratio(clip(newton, *self.force_range))))
    
    def lbf_to_raw_force(self, lbf: Union[int, float]) -> int:
        """Convert lbf to raw binary value [0-255]"""
        return int(round(self.f_ratio(clip(lbf * 4.44822, *self.force_range))))
    
    def raw_to_open_mm(self, raw: int) -> float:
        """Convert raw value [0-255] to mm of opening"""
        return round(self.inv_d(clip(raw, 0, 255)), 3)
    
    def raw_to_open_in(self, raw: int) -> float:
        """Convert raw value [0-255] to in of opening"""
        return round(self.raw_to_open_mm(raw) / 25.4, 3)
    
    def open_mm_to_raw(self, mm: Union[int, float]) -> int:
        """Convert opening in mm to raw binary value [0-255]"""
        return int(round(self.d_ratio(clip(mm, 0, self.stroke))))
    
    def open_in_to_raw(self, inches: Union[int, float]) -> int:
        """Convert opening in inches to raw binary value [0-255]"""
        return int(round(self.d_ratio(clip(inches * 25.4, 0, self.stroke))))
        
class Robotiq3FController:
    def __init__(self):
        raise NotImplementedError