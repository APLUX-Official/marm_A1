# Hand-eye calibration result message

bool eye_on_hand                          # true for eye-in-hand, false for eye-on-base
string robot_base_frame                   # robot base frame name
string robot_effector_frame               # robot end-effector frame name
string tracking_base_frame                # camera/tracking system frame name
string tracking_marker_frame              # marker frame name
geometry_msgs/TransformStamped transform  # calibration transform result
