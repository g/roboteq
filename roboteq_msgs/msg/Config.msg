#Config message, should only be sent when the motor is at 0RPM as the control loops will not be active.
#Roboteq can have one or two channels.
float32[] motor_amp_limit
float32[] pid_i_cap
float32[] pid_ki
float32[] pid_kd
float32[] pid_kp
float32[] max_acc
float32[] max_dec
uint8 operating_mode
float32[] max_pwr_fwd
float32[] max_pwr_rev
uint16 max_rpm
float32 overvoltage_limit
float32 undervoltage_limit
uint16 encoder_pulse_per_rev
bool serial_echo
float32 serial_watchdog_timeout
string auto_telemetry

