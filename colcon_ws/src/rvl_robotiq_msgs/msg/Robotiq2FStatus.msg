# gACT: Activation status, echo of the rACT bit (activation bit).
# This bit set to 1 when first power on, must be toggle to activate gripper
# Values:
#   0x0 - Gripper reset.
#   0x1 - Gripper activation.
uint8 activated

# gGTO: Action status, echo of the rGTO bit (go to bit).
# Values:
#   0x0 - Stopped (or performing activation / automatic release).
#   0x1 - Go to Position Request.
uint8 action_status

# gSTA: Gripper status, returns the current status & motion of the Gripper fingers.
# Values:
#   0x00 - Gripper is in reset ( or automatic release ) state. See Fault Status if gripper is activated.
#   0x01 - Activation in progress.
#   0x02 - Not used.
#   0x03 - Activation is completed.
uint8 gripper_status

# gOBJ: Object detection status, is a built-in feature that provides information on possible object pick-up. Ignore if gGTO == 0.
# Values:
#   0x00 - Fingers are in motion towards requested position. No object detected.
#   0x01 - Fingers have stopped due to a contact while opening before requested position. Object detected opening.
#   0x02 - Fingers have stopped due to a contact while closing before requested position. Object detected closing.
#   0x03 - Fingers are at requested position. No object detected or object has been loss / dropped.
uint8 object_status

# gFLT: Fault status returns general error messages that are useful for troubleshooting.
# Fault LED (red) is present on the Gripper chassis, LED can be blue, red or both and be solid or blinking.
# Values:
#   0x00 - No fault (solid blue LED)
#
#   Priority faults (solid blue LED)
#       0x05 - Action delayed, the activation (re-activation) must be completed prior to performing the action.
#       0x07 - The activation bit must be set prior to performing the action.
#
#   Minor faults (solid red LED)
#       0x08 - Maximum operating temperature exceeded (≥ 85 °C internally), let cool down (below 80 °C).
#       0x09 No communication during at least 1 second.
#
#   Major faults (LED blinking red/blue) - Reset is required (rising edge on activation bit (rACT) needed).
#       0x0A - Under minimum operating voltage.
#       0x0B - Automatic release in progress.
#       0x0C - Internal fault; contact support@robotiq.com.
#       0x0D - Activation fault, verify that no interference or other error occurred.
#       0x0E - Overcurrent triggered.
#       0x0F - Automatic release completed.
uint8 fault_status

# gPR: Echo of the requested position for the Gripper, value between 0x00 and 0xFF.
# Values range 0x00 to 0xFF (0-255):
#   0x00 - Full opening.
#   0xFF - Full closing.
uint8 position_request

# gPO: Actual position of the Gripper obtained via the encoders, value between 0x00 and 0xFF.
# Values range 0x00 to 0xFF (0-255):
#   0x00 - Fully opened.
#   0xFF - Fully closed.
uint8 current_position

# gCU: The current is read instantaneously from the motor drive, value between 0x00 and 0xFF.
# Approximate current equivalent is 10 * value read in mA.
uint8 motor_current
