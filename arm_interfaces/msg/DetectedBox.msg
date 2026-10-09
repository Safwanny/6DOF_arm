string name               # box_<colour>, one colour per box
geometry_msgs/Pose top    # centre of the top face in base_link; its yaw points along the length
float64 length            # top face size, length >= width
float64 width
bool upright              # false: the colour is seen but no face points up (box tipped over)
