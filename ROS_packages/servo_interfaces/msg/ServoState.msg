# Message for current servo state
std_msgs/Header header
float32 pan_angle    # Current pan angle in degrees
float32 tilt_angle   # Current tilt angle in degrees
bool pan_moving      # True if pan servo is currently moving
bool tilt_moving     # True if tilt servo is currently moving
