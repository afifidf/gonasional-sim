# One field landmark seen by the robot, in polar form relative to base_footprint.
uint8 GOALPOST=1
uint8 PENALTY_MARK=2
uint8 CENTER_MARK=3
uint8 FIELD_CORNER=4

uint8 type
float32 range        # m, horizontal distance from base_footprint
float32 bearing      # rad, left positive, relative to the robot's forward axis
float32 confidence   # 0..1
