# Fused state estimate consumed by the behavior layer.
std_msgs/Header header        # frame_id = field (origin at centre, +x toward the goal we attack)

geometry_msgs/Pose2D pose     # robot pose in the field frame
float32 pose_std_xy           # m
float32 pose_std_theta        # rad
bool localized                # true when the position estimate is tight enough to aim with
uint8 TRACKING=0              # observations agree with the estimate
uint8 UNCERTAIN=1             # estimate spread out or few recent landmarks
uint8 LOST=2                  # observations keep contradicting the estimate: relocalizing
uint8 localization_state

bool ball_known               # a ball estimate exists and is not too old
bool ball_visible             # ball was observed within the last few frames
float32 ball_age              # s since the last ball observation
geometry_msgs/Point ball_field      # m, field frame
geometry_msgs/Vector3 ball_velocity # m/s, field frame
geometry_msgs/Point ball_robot      # m, base_footprint frame (x forward, y left)
float32 ball_std              # m, position uncertainty
