#
# Primary message for users to get
#   information about the robot
#
std_msgs/Header header

# The supply voltage of the robot.
float32 supply_voltage

# The estimated battery level, 0.0 to 1.0
float32 battery_level

# True if the robot is currently charging from the plug or the dock.
bool is_charging

# True if the robot is plugged in.
bool plugged_in

# True if the robot is on the dock.
bool on_dock

# True if the physical runstop on the robot is triggered.
bool run_stopped

# True if the software runstop has been triggered.
bool run_stopped_in_sw

# True if we have a hardware fault
bool hardware_fault

# State of the motor breaker
BreakerState motor_breaker_state
