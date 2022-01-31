# rospy
import rospy
import rosservice
from rospy import ServiceException, ROSException
from rospy.exceptions import ROSInterruptException

# typing and type hinting
from typing import Union

# pretty logging
from rvl_utilities.CustomLogger import ColorLogger
from pprint import pprint

# Additional UR mappings
# from ur_dashboard_msgs.msg import ProgramState
from rvl_ur_remote_dashboard.URInterfaceMapping import *

# Robotiq driver
from rvl_robotiq_controller.RobotiqController import Robotiq2FController

class URRemoteDashboard:
    def __init__(self, name: str = 'UR5e', using_gripper: bool = False, using_urscript: bool = False, service_timeout: int = 5) -> None:
        """The UR Remote Dashboard class. This is the primary extension overlaying the existing
        Universal Robot Driver code base to access mapped services.

        Args:
            name (str, optional): Readable name to identify controller. Defaults to 'UR5e'.
            using_gripper (bool, optional): Initialized the attached Robotiq gripper. Defaults to False.
            using_urscript (bool, optional): Register appropriate publisher to send UR Script. Defaults to False.
            service_timeout (int, optional): Wait time for services to come on. Defaults to 5.
        """

        # custom logger
        self.logger = ColorLogger(name + ' Remote Dashboard')

        # default service timeout
        self.service_timeout = service_timeout

        # robot status tracking
        self.robot_mode = None
        self.safety_mode = None
        self.last_known_io_states = None
        self.loaded_program = None
        self.using_urscript = False

        # services
        self.services = self.define_services()

        # publisher/subscriber
        self.register_robot_status()
        
        # extended features
        if using_gripper:
            raise NotImplementedError
        
        if using_urscript:
            raise NotImplementedError

    # ---------------------------------------------------------------------------- #
    #                                 POWER CONTROL                                #
    # ---------------------------------------------------------------------------- #

    def power_on_arm(self, timeout: int = 30) -> bool:
        """Power on the arm to idle state (brakes engaged)."""
        success = self.trigger_service('power_on')
        if success:
            self.logger.log_warn(f'Waiting for arm to power on')
            try:
                elapsed = 0
                while self.robot_mode < 5 and elapsed < timeout and not rospy.is_shutdown():
                    rospy.sleep(1)
                    elapsed += 1
                    if elapsed > timeout:
                        raise ROSException
                self.logger.log_success('Arm powered on (brakes engaged)')
                return True
            except ROSException as e:
                self.logger.log_error(f'Wait time exceeded {timeout} seconds power on time. Aborted.')
                return False
        else:
            self.logger.log_error('Unable to power on robot arm')
            return False

    def power_off_arm(self, timeout: int = 30) -> bool:
        """Power off the arm."""
        success = self.trigger_service('power_off')
        if success:
            self.logger.log_warn(f'Waiting for arm to power off')
            try:
                elapsed = 0
                while self.robot_mode > 3 and elapsed < timeout and not rospy.is_shutdown():
                    rospy.sleep(1)
                    elapsed += 1
                    if elapsed > timeout:
                        raise ROSException
                self.logger.log_success('Arm powered off')
                return True
            except ROSException as e:
                self.logger.log_error(f'Wait time exceeded {timeout} seconds power off time. Aborted.')
                return False
        else:
            self.logger.log_error('Unable to power off robot arm')
            return False

    def system_shutdown(self) -> None:
        """Fully power down the robot (including control box)."""
        _ = self.trigger_service('shutdown')
        # rospy.signal_shutdown('UR System shutdown requested. Shutting everything down.')
        self.logger.log_success('Goodbye!')

    def cold_boot(self) -> bool:
        """Go directly to operational state (power on, brakes released). See release_brakes()."""
        return self.release_brakes()

    # ---------------------------------------------------------------------------- #
    #                                SAFETY CONTROL                                #
    # ---------------------------------------------------------------------------- #

    def release_brakes(self, timeout: int = 30) -> bool:
        """Fully power on the robot with brakes released."""
        success = self.trigger_service('brake_release')
        if success:
            self.logger.log_warn(f'Waiting for arm to power on and release brakes')
            try:
                elapsed = 0
                while self.robot_mode < 7 and elapsed < timeout and not rospy.is_shutdown():
                    rospy.sleep(1)
                    elapsed += 1
                    if elapsed > timeout:
                        raise ROSException
                self.logger.log_success('Arm powered and ready for planning!')
                return True
            except ROSException as _:
                self.logger.log_error(f'Wait time exceeded {timeout} seconds full powered on time. Aborted.')
                return False
        else:
            self.logger.log_error('Unable to fully initialized robot')
            return False

    def restart_safety(self) -> bool:
        """Clear a safety fault or violation. Arm will be powered off."""
        success = self.trigger_service('restart_safety')
        if success:
            self.logger.log_warn(f'Safety fault/violation cleared')
            self.logger.log_warn(f'Robot is now powered off')
            self.logger.log_error(f'Check log for additional information before restarting!')
            return True
        else:
            self.logger.log_error('Unable to clear safety violation')
            return False

    def clear_protective_stop(self, timeout: int = 30) -> bool:
        """Clear a protective stop."""
        success = self.trigger_service('unlock_protective_stop')
        if success:
            self.logger.log_warn(f'Waiting for protective stop to clear')
            try:
                elapsed = 0
                while self.safety_mode != 1 and elapsed < timeout and not rospy.is_shutdown():
                    rospy.sleep(1)
                    elapsed += 1
                    if elapsed > timeout:
                        raise ROSException
                self.logger.log_success('Protective stop cleared')
                return True
            except ROSException as _:
                self.logger.log_error(f'Wait time exceeded {timeout} seconds clearing time. Aborted.')
                return False
        else:
            self.logger.log_error('Unable to clear protective stop')
            return False

    def clear_operational_mode(self) -> bool:
        """Allow PolyScope to change operational mode. User password will be enabled."""
        success = self.trigger_service('clear_operational_mode')
        if success:
            self.logger.log_warn(f'Operational mode cleared')
            return True
        else:
            self.logger.log_error('Unable to clear operational mode')
            return False

    # ---------------------------------------------------------------------------- #
    #                                 POPUP CONTROL                                #
    # ---------------------------------------------------------------------------- #

    def close_popup(self, safety: bool = False) -> bool:
        """Close a popup on the Teach Pendant or PolyScope.

        Args:
            safety (bool, optional): Set to True if popup is a safety popup. Defaults to False.

        Returns:
            bool: True if the targeted popup is closed.
        """
        if safety:
            success = self.trigger_service('close_safety_popup')
        else:
            success = self.trigger_service('close_popup')

        if success:
            self.logger.log_success(f'{"Safety popup" if safety else "Popup"} closed')
            return True
        else:
            self.logger.log_error('Unable to close popup')
            return False

    def send_popup(self, message: str) -> bool:
        """Send a message as a popup to Teach Pendant or PolyScope."""
        request = PopupRequest()
        request.message = message
        try:
            serv = self.services['popup']
            rospy.wait_for_message(serv, timeout = self.service_timeout)
            response = rospy.ServiceProxy(serv, Popup)(request)
            if not response.success: raise ServiceException('response.success returned False')
            self.logger.log_success('Popup sent to Teach Pendant')
        except (ROSException, ServiceException, KeyError) as e:
            self.logger.log_error('Unable to send popup')
            self.logger.log_error(e)
            return False

    # ---------------------------------------------------------------------------- #
    #                           DASHBOARD SERVER CONTROL                           #
    # ---------------------------------------------------------------------------- #

    def connect_dashboard(self, quiet: bool = False) -> bool:
        """Connect to the dashboard server. Need to be done before calling other services."""
        success = self.trigger_service('connect')
        if success:
            self.logger.log_success('Connection to dashboard established')
            return True
        else:
            if not quiet:
                self.logger.log_error('Unable to disconnect from dashboard server')
            return False

    def disconnect_dashboard(self) -> bool:
        """Disconnect from the dashboard server."""
        success = self.trigger_service('quit')
        if success:
            self.logger.log_success('Connection to dashboard terminated')
            return True
        else:
            self.logger.log_error('Unable to disconnect from dashboard server')
            return False

    def spam_connect(self, attempts: int = 10) -> bool:
        """Repeatedly calling connect() due to error prone and asynchronous status of the server.
        
        Args:
            attempts (int, optional): Number of times connect() is called internally. Defaults to 10.
        """
        for i in range(attempts):
            self.logger.log_warn(f'Reconnecting attempted ({attempts - i} remaining)', indent = 1)
            if self.connect_dashboard(quiet = True):
                return True
            rospy.sleep(1)
        self.logger.log_error('Reconnection attempts to dashboard server unsuccessful')
        return False

    # ---------------------------------------------------------------------------- #
    #                    POLYSCOPE PROGRAMS/INSTALLATION CONTROL                   #
    # ---------------------------------------------------------------------------- #

    def start_loaded_program(self) -> bool:
        """Start execution of default or loaded program."""
        success = self.trigger_service('play')
        if success:
            self.logger.log_success(f'Program {self.loaded_program} is running')
            return True
        else:
            self.logger.log_error(f'Unable to start {self.loaded_program}')
            return False

    def pause_loaded_program(self) -> bool:
        """Pause PolyScope program execution."""
        success = self.trigger_service('pause')
        if success:
            self.logger.log_success(f'Program {self.loaded_program} is paused')
            return True
        else:
            self.logger.log_error(f'Unable to pause {self.loaded_program}')

            return False

    def stop_loaded_program(self) -> bool:
        """Stop PolyScope program execution."""
        success = self.trigger_service('stop')
        if success:
            self.logger.log_success(f'Program {self.loaded_program} is stopped')
            return True
        else:
            self.logger.log_error(f'Unable to stop {self.loaded_program}')
            return False

    def is_program_running(self) -> bool:
        """Returns true if the default or loaded program is running."""
        try:
            response = rospy.ServiceProxy(self.services['program_running'], IsProgramRunning)()
            if not response.success: raise ServiceException('response.success returned False')
            self.logger.log_success(f'Program "{self.loaded_program}" is {"running" if response.program_running else "not running"}')
            return response.program_running
        except (ROSException, ServiceException, KeyError) as e:
            self.logger.log_error('Unable to query if program is running')
            self.logger.log_error(e)
            return False

    def is_program_saved(self) -> bool:
        """Returns true if the default or loaded program is saved."""
        try:
            response = rospy.ServiceProxy(self.services['program_saved'], IsProgramSaved)()
            self.logger.log_success(f'Program "{response.program_name}" is {"saved" if response.program_saved else "not saved"}')
            if not response.success: raise ServiceException('response.success returned False')
            return response.program_saved
        except (ROSException, ServiceException, KeyError) as e:
            self.logger.log_error('Unable to query if program is saved')
            self.logger.log_error(e)
            return False

    def query_program_state(self) -> None:
        """Display the name and execution state of the current PolyScope program."""
        try:
            response = rospy.ServiceProxy(self.services['program_state'], GetProgramState)()
            if not response.success: raise ServiceException('response.success returned False')
            self.logger.log_success(f'Program "{response.program_name}" state is {response.state.state}')
        except (ROSException, ServiceException, KeyError) as e:
            self.logger.log_error('Unable to query if program is saved')
            self.logger.log_error(e)

    def terminate_external_control(self) -> bool:
        """Make the external_control node on PolyScope returns."""
        success = self.trigger_service('hand_back_control')
        if success:
            self.logger.log_success('"External Control" program node terminated')
            return True
        else:
            self.logger.log_error('Unable to terminate external control node')
            return False

    def get_loaded_program(self) -> Union[str, None]:
        """Returns the name of the loaded program."""
        try:
            serv = self.services['get_loaded_program']
            response = rospy.ServiceProxy(serv, GetLoadedProgram)()
            if not response.success: raise ServiceException('response.success returned False')
            self.loaded_program = response.program_name
            return response.program_name
        except Exception as e:
            self.logger.log_error('Unable to request loaded program name')
            self.logger.log_error(str(e))

    def load_program(self, filename: str, ptype: str, wait: int = 10, attempts: int = 10) -> None:
        """Load a program or installation file.

        Args:
            filename (str): Name of file with extension e.g., program.urp
            ptype (str): Type of program. Accepting ['prog', 'p', 'program', 'urp'] or ['inst', 'i', 'installation'].
            wait (int, optional): Wait time to handle known disconnection issue. Defaults to 10.
            attempts (int, optional): Number of reconnection attempts. Defaults to 10.
        """
        request = LoadRequest()
        request.filename = filename
        try:
            if ptype in ['prog', 'p', 'program', 'urp']:
                serv = self.services['load_program']
            elif ptype in ['inst', 'i', 'installation']:
                serv = self.services['load_installation']
            else:
                self.logger.log_error(f'{ptype} is invalid')
                self.logger.log_error(f'Expecting program [p, prog, program, urp] or installation [inst, i, installation]')
                return None
            rospy.wait_for_service(serv, timeout=self.service_timeout)
            response = rospy.ServiceProxy(serv, Load)(request)
            if not response.success: raise ServiceException('response.success returned False')
        except (ServiceException, ROSException) as _:
            self.logger.log_warn('Known dashboard server disconnection occured')
            self.logger.log_warn(f'Waiting for {wait} seconds for program/installation to load correctly')
            rospy.sleep(wait)
            self.logger.log_warn(f'Attempting to reconnect to dashboard server ({attempts} attempts)')
            self.spam_connect(attempts = attempts)
            self.close_popup()
            self.cold_boot()
        except KeyError as e:
            self.logger.log_error(str(e))
            return None
        self.last_known_installation = filename
        rospy.sleep(1)

    # ---------------------------------------------------------------------------- #
    #                                 ROBOT STATUS                                 #
    # ---------------------------------------------------------------------------- #

    def robot_status_callback(self, msg):
        self.robot_mode = msg.mode

    def robot_safety_callback(self, msg):
        self.safety_mode = msg.mode

    def robot_iostate_callback(self, msg):
        self.last_known_io_states = msg

    def get_robot_mode(self) -> Union[int, None]:
        """Returns the current robot mode."""
        try:
            serv = self.services['get_robot_mode']
            rospy.wait_for_service(serv, timeout = self.service_timeout)
            response = rospy.ServiceProxy(serv, GetRobotMode)()
            if not response.success: raise ServiceException('response.success returned False')
            mode = response.robot_mode.mode
            self.logger.log_success(f'Robot mode = {mode} ({RobotModeMapping(mode).name})')
            return mode
        except (ROSException, ServiceException, KeyError) as e:
            self.logger.log_error('Unable to request robot mode')
            self.logger.log_error(e)
        
    def get_safety_mode(self) -> Union[int, None]:
        """Returns the current safety mode."""
        try:
            serv = self.services['get_safety_mode']
            rospy.wait_for_service(serv, timeout = self.service_timeout)
            response = rospy.ServiceProxy(serv, GetSafetyMode)()
            if not response.success: raise ServiceException('response.success returned False')
            mode = response.safety_mode.mode
            self.logger.log_success(f'Robot safety mode = {mode} ({SafetyModeMapping(mode).name})')
            return mode
        except (ROSException, ServiceException, KeyError) as e:
            self.logger.log_error('Unable to request robot safety mode')
            self.logger.log_error(e)

    # ---------------------------------------------------------------------------- #
    #                               ADVANCED FEATURES                              #
    # ---------------------------------------------------------------------------- #

    def zero_force_torque_sensor(self) -> bool:
        """Zero the ft-sensor. Only work on e-Series in remote-control mode."""
        success = self.trigger_service('zero_ftsensor')
        if success:
            self.logger.log_success('Force/Torque sensor zero-ed')
            return True
        else:
            self.logger.log_error('Unable to zero force/torque sensor')
            return False

    def log_to_pendant(self, message: str) -> None:
        """Log a message to PolyScope logs."""
        request = AddToLogRequest()
        request.message = message
        try:
            serv = self.services['add_to_log']
            rospy.wait_for_service(serv, timeout = self.service_timeout)
            response = rospy.ServiceProxy(serv, AddToLog)(request)
            if not response.success: raise ServiceException('response.success returned False')
        except (ROSException, ServiceException, KeyError) as e:
            self.logger.log_error('Unable to log message to Teach Pendant')
            self.logger.log_error(e)

    def raw_request(self, query):
        """Send any arbitrary message or request to the dashboard server."""
        request = RawRequestRequest()
        request.query = query
        try:
            serv = self.services['raw_request']
            rospy.wait_for_service(serv, timeout = self.service_timeout)
            response = rospy.ServiceProxy(serv, RawRequest)(request)
            if not response.success: raise ServiceException('response.success returned False')
        except (ROSException, ServiceException, KeyError) as e:
            self.logger.log_error('Unable to log message to Teach Pendant')
            self.logger.log_error(e)

    def set_io(self, function: int, pin: int, state: float) -> None:
        """Set specific IO port on the robot. Currently not supporting specific domains (current/voltage).

        Args:
            function (int): See SetIOFunctionMapping.
            pin (int): Which pin to execute the function on.
            state (float): 0/1 for digital IOs and value for analog IO.
        """
        request = SetIORequest()
        request.fun = function
        request.pin = pin
        request.state = state
        try:
            serv = self.services['set_io']
            rospy.wait_for_service(serv, timeout = self.service_timeout)
            response = rospy.ServiceProxy(serv, SetIO)(request)
            if not response.success: raise ServiceException('response.success returned False')
        except (ROSException, ServiceException, KeyError) as e:
            self.logger.log_error(f'Unable to set pin {pin} to {state} using {SetIOFunctionMapping(function).name}')
            self.logger.log_error(e)

    def set_payload(self, mass: float, cx: float, cy: float, cz: float) -> None:
        """Set the payload mass and center of gravity.

        Args:
            mass (float): Mass of the payload in kg.
            cx, cy, cz (float): Center of gravity of the payload.
        """
        request = SetPayloadRequest()
        request.center_of_gravity = Vector3()
        request.center_of_gravity.x = cx
        request.center_of_gravity.y = cy
        request.center_of_gravity.z = cz
        request.mass = mass
        try:
            serv = self.services['set_payload']
            rospy.wait_for_service(serv, timeout = self.service_timeout)
            response = rospy.ServiceProxy(serv, SetPayload)(request)
            if not response.success: raise ServiceException('response.success returned False')
            self.logger.log_success(f'Setting payload successfully')
        except (ROSException, ServiceException, KeyError) as e:
            self.logger.log_error('Unable to set payload')
            self.logger.log_error(e)

    def set_speed_slider(self, fraction: float) -> None:
        """Set robot execution speed as a fraction. Only set less than 1 on scaled controllers.

        Args:
            fraction (float): 0 to 1 if using scaled (default) controllers.
        """
        request = SetSpeedSliderFractionRequest()
        request.speed_slider_fraction = clip(fraction, 0.0, 1.0)
        try:
            serv = self.services['set_speed_slider']
            rospy.wait_for_service(serv, timeout = self.service_timeout)
            response = rospy.ServiceProxy(serv, SetSpeedSliderFraction)(request)
            if not response.success: raise ServiceException('response.success returned False')
            self.logger.log_success(f'Setting speed slider successfully')
        except (ROSException, ServiceException, KeyError) as e:
            self.logger.log_error('Unable to set speed slider')
            self.logger.log_error(e)

    # def set_robot_mode(self):
    #     raise NotImplementedError

    # ---------------------------------------------------------------------------- #
    #                             SUPPORTING FUNCTIONS                             #
    # ---------------------------------------------------------------------------- #

    def trigger_service(self, serv_alias):
        """Internal trigger service handling with exceptions"""
        if serv_alias in self.services:
            serv_name = self.services[serv_alias]
            try:
                rospy.wait_for_service(serv_name, timeout = self.service_timeout)
                response = rospy.ServiceProxy(serv_name, Trigger)()
                if not response.success:
                    self.logger.log_error(response.message)
                    raise ServiceException('response.success returned False')
                return response.success
            except (ROSException, ROSInterruptException, KeyboardInterrupt, ServiceException) as e:
                self.logger.log_error(f'Unable to trigger {serv_name}')
                self.logger.log_error(e)
        else:
            self.logger.log_error(f'{serv_alias} is unknown/unsupported. Aborted.')
            return False

    def define_services(self):
        available = rosservice.get_service_list()
        filterted = [s for s in available if 'ur_hardware_interface' in s and 'logger' not in s]
        services = {s[s.rfind('/') + 1:] : s for s in filterted}
        return dict(sorted(services.items()))

    def register_robot_status(self) -> None:
        """Register necessary subscribers and callbacks to monitor robot operational status."""
        try:
            # wait for topics to show up
            self.robot_mode = rospy.wait_for_message('/ur_hardware_interface/robot_mode', RobotMode, timeout = self.service_timeout).mode
            self.safety_mode = rospy.wait_for_message('/ur_hardware_interface/safety_mode', SafetyMode, timeout = self.service_timeout).mode
            self.last_known_io_states = rospy.wait_for_message('/ur_hardware_interface/io_states', IOStates, timeout = self.service_timeout)

            # register robot status tracking with callbacks
            self.robot_mode_sub = rospy.Subscriber('/ur_hardware_interface/robot_mode', RobotMode, self.robot_status_callback)
            self.robot_safety_sub = rospy.Subscriber('/ur_hardware_interface/safety', SafetyMode, self.robot_safety_callback)
            self.robot_io_sub = rospy.Subscriber('/ur_hardware_interface/io_states', IOStates, self.robot_iostate_callback)

            # verify if services are available
            self.verify_services()

            # get program name
            self.get_loaded_program()

            self.logger.log_success('Registered robot status subscribers')
        except ROSException as error:
            self.logger.log_error('Unable to register status subscriber')
            self.logger.log_error('Remote Dashboard terminated')
            self.logger.log_error(error)
            exit(-1)

    def verify_services(self):
        self.logger.log_warn('Validating if all services are available')
        for _, serv in self.services.items():
            rospy.wait_for_service(serv, timeout = self.service_timeout)
        self.logger.log_success('All supported services are available')