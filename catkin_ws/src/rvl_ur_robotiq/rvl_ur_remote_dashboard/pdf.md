---
description: |
    API documentation for modules: src, src.rvl_ur_motion_planner, src.rvl_ur_motion_planner.URMoveitCommander, src.rvl_ur_remote_dashboard, src.rvl_ur_remote_dashboard.URInterfaceMapping, src.rvl_ur_remote_dashboard.URRemoteDashboard.

lang: en

classoption: oneside
geometry: margin=1in
papersize: a4

linkcolor: blue
links-as-notes: true
...


    
# Namespace `src` {#id}




    
## Sub-modules

* [src.rvl_ur_motion_planner](#src.rvl_ur_motion_planner)
* [src.rvl_ur_remote_dashboard](#src.rvl_ur_remote_dashboard)






    
# Module `src.rvl_ur_motion_planner` {#id}




    
## Sub-modules

* [src.rvl_ur_motion_planner.URMoveitCommander](#src.rvl_ur_motion_planner.URMoveitCommander)






    
# Module `src.rvl_ur_motion_planner.URMoveitCommander` {#id}







    
## Classes


    
### Class `URCommander` {#id}




>     class URCommander(
>         group_name='arm',
>         speed=0.1,
>         accel=0.1
>     )










    
#### Methods


    
##### Method `all_close` {#id}




>     def all_close(
>         self,
>         goal,
>         actual,
>         tolerance
>     )


Convenience method for testing if the values in two lists are within a tolerance of each other.
For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
between the identical orientations q and -q is calculated correctly).
@param: goal       A list of floats, a Pose or a PoseStamped
@param: actual     A list of floats, a Pose or a PoseStamped
@param: tolerance  A float
@returns: bool

    
##### Method `define_preset_locations` {#id}




>     def define_preset_locations(
>         self
>     )




    
##### Method `go_to_preset_location` {#id}




>     def go_to_preset_location(
>         self,
>         name
>     )




    
##### Method `home` {#id}




>     def home(
>         self
>     )




    
##### Method `report` {#id}




>     def report(
>         self
>     )






    
# Module `src.rvl_ur_remote_dashboard` {#id}




    
## Sub-modules

* [src.rvl_ur_remote_dashboard.URInterfaceMapping](#src.rvl_ur_remote_dashboard.URInterfaceMapping)
* [src.rvl_ur_remote_dashboard.URRemoteDashboard](#src.rvl_ur_remote_dashboard.URRemoteDashboard)






    
# Module `src.rvl_ur_remote_dashboard.URInterfaceMapping` {#id}







    
## Classes


    
### Class `RobotModeMapping` {#id}




>     class RobotModeMapping(
>         value,
>         names=None,
>         *,
>         module=None,
>         qualname=None,
>         type=None,
>         start=1
>     )


An enumeration.


    
#### Ancestors (in MRO)

* [enum.Enum](#enum.Enum)



    
#### Class variables


    
##### Variable `BACKDRIVE` {#id}






    
##### Variable `BOOTING` {#id}






    
##### Variable `CONFIRM_SAFETY` {#id}






    
##### Variable `DISCONNECTED` {#id}






    
##### Variable `IDLE` {#id}






    
##### Variable `NO_CONTROLLER` {#id}






    
##### Variable `POWER_OFF` {#id}






    
##### Variable `POWER_ON` {#id}






    
##### Variable `RUNNING` {#id}






    
##### Variable `UPDATING_FIRMWARE` {#id}









    
### Class `SafetyModeMapping` {#id}




>     class SafetyModeMapping(
>         value,
>         names=None,
>         *,
>         module=None,
>         qualname=None,
>         type=None,
>         start=1
>     )


An enumeration.


    
#### Ancestors (in MRO)

* [enum.Enum](#enum.Enum)



    
#### Class variables


    
##### Variable `AUTOMATIC_MODE_SAFEGUARD_STOP` {#id}






    
##### Variable `FAULT` {#id}






    
##### Variable `NORMAL` {#id}






    
##### Variable `PROTECTIVE_STOP` {#id}






    
##### Variable `RECOVERY` {#id}






    
##### Variable `REDUCED` {#id}






    
##### Variable `ROBOT_EMERGENCY_STOP` {#id}






    
##### Variable `SAFEGUARD_STOP` {#id}






    
##### Variable `SYSTEM_EMERGENCY_STOP` {#id}






    
##### Variable `SYSTEM_THREE_POSITION_ENABLING_STOP` {#id}






    
##### Variable `UNDEFINED_SAFETY_MODE` {#id}






    
##### Variable `VALIDATE_JOINT_ID` {#id}






    
##### Variable `VIOLATION` {#id}









    
### Class `SetIOFunctionMapping` {#id}




>     class SetIOFunctionMapping(
>         value,
>         names=None,
>         *,
>         module=None,
>         qualname=None,
>         type=None,
>         start=1
>     )


An enumeration.


    
#### Ancestors (in MRO)

* [enum.Enum](#enum.Enum)



    
#### Class variables


    
##### Variable `SET_ANALOG_OUT` {#id}






    
##### Variable `SET_DIGITAL_OUT` {#id}






    
##### Variable `SET_FLAG` {#id}






    
##### Variable `SET_TOOL_VOLTAGE` {#id}









    
### Class `SetIOPinMapping` {#id}




>     class SetIOPinMapping(
>         value,
>         names=None,
>         *,
>         module=None,
>         qualname=None,
>         type=None,
>         start=1
>     )


An enumeration.


    
#### Ancestors (in MRO)

* [enum.Enum](#enum.Enum)






    
### Class `SetIOPinState` {#id}




>     class SetIOPinState(
>         value,
>         names=None,
>         *,
>         module=None,
>         qualname=None,
>         type=None,
>         start=1
>     )


An enumeration.


    
#### Ancestors (in MRO)

* [enum.Enum](#enum.Enum)



    
#### Class variables


    
##### Variable `OFF` {#id}






    
##### Variable `ON` {#id}









    
### Class `SetIOToolState` {#id}




>     class SetIOToolState(
>         value,
>         names=None,
>         *,
>         module=None,
>         qualname=None,
>         type=None,
>         start=1
>     )


An enumeration.


    
#### Ancestors (in MRO)

* [enum.Enum](#enum.Enum)



    
#### Class variables


    
##### Variable `TOOL_VOLTAGE_0V` {#id}






    
##### Variable `TOOL_VOLTAGE_12V` {#id}






    
##### Variable `TOOL_VOLTAGE_24V` {#id}











    
# Module `src.rvl_ur_remote_dashboard.URRemoteDashboard` {#id}







    
## Classes


    
### Class `URRemoteDashboard` {#id}




>     class URRemoteDashboard(
>         name: str = 'UR5e',
>         using_gripper: bool = False,
>         using_urscript: bool = False,
>         service_timeout: int = 5
>     )


The UR Remote Dashboard class. This is the primary extension overlaying the existing
Universal Robot Driver code base to access mapped services.


Args
-----=
**```name```** :&ensp;<code>str</code>, optional
:   Readable name to identify controller. Defaults to 'UR5e'.


**```using_gripper```** :&ensp;<code>bool</code>, optional
:   Initialized the attached Robotiq gripper. Defaults to False.


**```using_urscript```** :&ensp;<code>bool</code>, optional
:   Register appropriate publisher to send UR Script. Defaults to False.


**```service_timeout```** :&ensp;<code>int</code>, optional
:   Wait time for services to come on. Defaults to 5.









    
#### Methods


    
##### Method `clear_operational_mode` {#id}




>     def clear_operational_mode(
>         self
>     )




    
##### Method `clear_protective_stop` {#id}




>     def clear_protective_stop(
>         self,
>         timeout=30
>     )




    
##### Method `close_popup` {#id}




>     def close_popup(
>         self,
>         safety=False
>     )




    
##### Method `cold_boot` {#id}




>     def cold_boot(
>         self
>     )




    
##### Method `connect_dashboard` {#id}




>     def connect_dashboard(
>         self,
>         quiet=False
>     )




    
##### Method `define_services` {#id}




>     def define_services(
>         self
>     )




    
##### Method `disconnect_dashboard` {#id}




>     def disconnect_dashboard(
>         self
>     )




    
##### Method `get_loaded_program` {#id}




>     def get_loaded_program(
>         self
>     )




    
##### Method `get_robot_mode` {#id}




>     def get_robot_mode(
>         self
>     )




    
##### Method `get_safety_mode` {#id}




>     def get_safety_mode(
>         self
>     )




    
##### Method `is_program_running` {#id}




>     def is_program_running(
>         self
>     )




    
##### Method `is_program_saved` {#id}




>     def is_program_saved(
>         self
>     )




    
##### Method `load_program` {#id}




>     def load_program(
>         self,
>         filename,
>         ptype,
>         wait=10,
>         attempts=10
>     )




    
##### Method `log_to_pendant` {#id}




>     def log_to_pendant(
>         self,
>         message
>     )




    
##### Method `pause_loaded_program` {#id}




>     def pause_loaded_program(
>         self
>     )




    
##### Method `power_off_arm` {#id}




>     def power_off_arm(
>         self,
>         timeout=30
>     )




    
##### Method `power_on_arm` {#id}




>     def power_on_arm(
>         self,
>         timeout: int = 30
>     ) ‑> bool


Power on the arm to idle state (arm cannot move, brakes engaged).


Args
-----=
**```timeout```** :&ensp;<code>int</code>, optional
:   Wait time for the arm to power on. Defaults to 30 seconds.



Raises
-----=
<code>ROSException</code>
:   Elapsed time exceeded specified wait time, will raise exception but will


not terminate program.

Returns
-----=
<code>bool</code>
:   True if arm is not powered, False otherwise



    
##### Method `query_program_state` {#id}




>     def query_program_state(
>         self
>     )




    
##### Method `raw_request` {#id}




>     def raw_request(
>         self,
>         query
>     )




    
##### Method `register_robot_status` {#id}




>     def register_robot_status(
>         self
>     ) ‑> None


Register necessary subscribers and callbacks to monitor robot operational status.

    
##### Method `release_brakes` {#id}




>     def release_brakes(
>         self,
>         timeout=30
>     )




    
##### Method `restart_safety` {#id}




>     def restart_safety(
>         self
>     )




    
##### Method `robot_iostate_callback` {#id}




>     def robot_iostate_callback(
>         self,
>         msg
>     )




    
##### Method `robot_safety_callback` {#id}




>     def robot_safety_callback(
>         self,
>         msg
>     )




    
##### Method `robot_status_callback` {#id}




>     def robot_status_callback(
>         self,
>         msg
>     )




    
##### Method `send_popup` {#id}




>     def send_popup(
>         self,
>         message
>     )




    
##### Method `set_io` {#id}




>     def set_io(
>         self,
>         function,
>         pin,
>         state
>     )




    
##### Method `set_payload` {#id}




>     def set_payload(
>         self,
>         mass,
>         cx,
>         cy,
>         cz
>     )




    
##### Method `set_robot_mode` {#id}




>     def set_robot_mode(
>         self
>     )




    
##### Method `set_speed_slider` {#id}




>     def set_speed_slider(
>         self,
>         fraction
>     )




    
##### Method `spam_connect` {#id}




>     def spam_connect(
>         self,
>         attempts=10
>     )




    
##### Method `start_loaded_program` {#id}




>     def start_loaded_program(
>         self
>     )




    
##### Method `stop_loaded_program` {#id}




>     def stop_loaded_program(
>         self
>     )




    
##### Method `system_shutdown` {#id}




>     def system_shutdown(
>         self
>     )




    
##### Method `terminate_external_control` {#id}




>     def terminate_external_control(
>         self
>     )




    
##### Method `trigger_service` {#id}




>     def trigger_service(
>         self,
>         serv_alias
>     )




    
##### Method `verify_services` {#id}




>     def verify_services(
>         self
>     )




    
##### Method `zero_force_torque_sensor` {#id}




>     def zero_force_torque_sensor(
>         self
>     )





-----
Generated by *pdoc* 0.10.0 (<https://pdoc3.github.io>).
