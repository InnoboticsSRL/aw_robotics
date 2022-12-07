# This message contains the status of the AWTool.

std_msgs/Header header

# Status of the vaccum pump
#int16 PUMP_ERROR = -1
#int16 PUMP_STANDBY = 0
#int16 PUMP_GRIPPING = 1
#int16 PUMP_HOLDING_OBJECT = 2
#int16 PUMP_NO_OBJECT = 3
#int16 PUMP_RELEASING = 4

int16 pump_state

# Clamp status
#int16 CLAMP_ERROR = -1  # error of clamping
#int16 CLAMP_STANDBY = 0 # gripper is waiting for instruction
#int16 CLAMP_CLOSING = 1 # closing the gripper
#int16 CLAMP_HOLDING_OBJECT = 2
#int16 CLAMP_NO_OBJECT = 3
#int16 CLAMP_OPENING = 4 # opening the gripper

int16 clamp_state
float32 clamp_width #aperture of the clamp in [m]

# tool_change_state
#int16 TOOL_CHANGE_ERROR = -1
#int16 TOOL_CHANGE_EMPTY = 0           # no gripper loaded

int16 tool_change_state

# tool_type
# NONE = 0
# BOX_GRIPPER = 1
# INTERLAY_GRIPPER = 2

int16 tool_type
