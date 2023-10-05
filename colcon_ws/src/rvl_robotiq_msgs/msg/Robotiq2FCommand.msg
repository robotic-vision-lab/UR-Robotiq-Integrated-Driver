# rACT: First action to be made prior to any other actions, rACT bit will activate the Gripper.
# Clear rACT to reset the Gripper and clear fault status.
uint8 activate 0

# rGTO: The "Go To" action moves the Gripper fingers to the requested position using the configuration defined by the other registers,
# rGTO will engage motion while byte 3, 4 and 5 will determine aimed position, force and speed. The only motions performed without the
# rGTO bit are activation and automatic release routines.
#   0x0 - Stop.
#   0x1 - Go to requested position.
uint8 goto_action 0

# rATR: Automatic Release routine action slowly opens the Gripper fingers until all motion axes reach their mechanical limits. After all
# motion is completed, the Gripper sends a fault signal and needs to be reactivated before any other motion is performed. The rATR bit
# overrides all other commands excluding the activation bit (rACT).
#   0x0 - Normal.
#   0x1 - Emergency auto-release.
uint8 trigger_autorelease 0

# rARD: Auto-release direction. When auto-releasing, rARD commands the direction of the movement. The rARD bit should be set prior to
# or at the same time as the rATR bit, as the motion direction is set when the auto-release is initiated.
# Value:
#   0x0 - Closing auto-release
#   0x1 - Opening auto-release
uint8 autorelease_direction 0

# rPR: This register is used to set the target position for the Gripper's fingers. The positions 0x00 and 0xFF correspond respectively to the fully
# opened and fully closed mechanical stops. For detailed finger trajectory, please refer to the Specifications section.
#
# Opening / count: 0.4 mm (for 85 mm stroke) and 0.65 mm (for 140 mm stroke)
#
# Value:
#   0x00 - Open position, with 85 mm or 140 mm opening respectively
#   0xFF - Closed
uint8 position 0

# rSP: This register is used to set the Gripper closing or opening speed in real time, however, setting a speed will not initiate a motion.
# Value range 0x00 to 0xFF
uint8 speed 0

# rFR: The force setting defines the final gripping force for the Gripper. The force will fix the maximum current sent to the motor while in
# motion. If the current limit is exceeded, the fingers stop and trigger an object detection notification. Please refer to the Robot Input
# Registers & Status section for details on force control.
# Value range 0x00 to 0xFF
uint8 force 0
