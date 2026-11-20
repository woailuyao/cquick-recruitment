# A complete servo command.
# Two roles:
#   1) message type of /servo/command, so ros2 bag can record and replay it;
#   2) mirrors the Action Goal fields, so replayed motion and live motion are one thing.

uint8 id             # servo ID, 1-254; must be listed in the driver parameter servo_ids
uint16 pwm           # target PWM; 500-2500 is the electrical limit, safe range is pwm_min..pwm_max
uint16 duration_ms   # travel time in ms; 0 means use default_duration_ms
