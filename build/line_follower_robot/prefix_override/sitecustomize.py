import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ronith/ros2_ws/src/line_follower_robot_fixed 4/line_follower_robot_fixed/line_follower_robot/install/line_follower_robot'
