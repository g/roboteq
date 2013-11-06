# 50Hz feedback message for controls purposes
# Arrays for motor-specific values to support dual-channel drivers.
Header header

# Current flowing in the motors (A)
float32[] motor_current

# Power delivered by the motors (W)
float32[] motor_power

# Commanded and measured speed of the motors (rad/s)
float32[] commanded_velocity
float32[] measured_velocity

# Electrical power supply to the driver (V, A)
float32 supply_voltage
float32 supply_current

# Logic rails in the driver
float32 internal_voltage
float32 adc_voltage
