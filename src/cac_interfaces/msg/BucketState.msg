# /bucket/state   platform -> mission, dashboard
std_msgs/Header header
float32 angle_rad
float32 motor_current_a
bool loaded                # true when the load check says we are carrying sand
bool fault                 # stalled, over-current or encoder lost
