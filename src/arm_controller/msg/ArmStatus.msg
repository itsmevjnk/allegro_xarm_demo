# header    Header      header from original xArm telemetry message
# pos       float32[3]  base-centric coordinates of the tool centre point (TCP) (in mm) - X, Y and Z respectively
# orient    float32[3]  tool orientation relative to base (in radians) - roll (RX), pitch (RY) and yaw (RZ) respectively
# joints    float32[7]  joint angles (in radians)
# moving    bool        whether the arm is moving

Header header
float32[3] pos
float32[3] orient
float32[7] joints
bool moving