string  name
float64 stamp

uint8   MODE_DISABLED = 0
uint8   MODE_EFFORT = 1
uint8   MODE_VELOCITY = 2
uint8   MODE_POSITION = 3

uint8   mode

float32 desired_position
float32 desired_velocity
float32 desired_effort

float32 actual_position
float32 actual_velocity
float32 actual_effort

float32 calibration_offset

float32 position_lower_limit
float32 position_upper_limit
float32 velocity_limit
float32 effort_limit
