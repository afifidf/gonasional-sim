# Latest message received from one teammate (team_comm).
uint8 player
geometry_msgs/Pose2D pose     # field frame, +x toward the goal we attack
bool localized
bool ball_known
geometry_msgs/Point ball      # field frame (valid when ball_known)
float32 ball_age              # s since that robot saw the ball, at send time
string role                   # goalie | striker | defender | supporter | ''
bool active                   # standing, not penalized
float32 age                   # s since the message was received
