
# Control whether the setpoint represents a velocity or position command.
# Velocity is the default, but position may be used for fine-grained movements,
# or active braking on an incline.
uint8 MODE_VELOCITY=0
uint8 MODE_POSITION=1
uint8 mode

# Commanded velocity (rad/s) or position (rad) for the motor. Position uses
# the same scale as measured_position in the Feedback message.
float32 setpoint
