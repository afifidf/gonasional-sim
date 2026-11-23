# All detections of one camera frame.
std_msgs/Header header        # stamp = image capture time (copied from the camera message)
uint16 width                  # frame size the boxes refer to (450x337 for main_vision)
uint16 height
aroc_interfaces/Detection[] detections
