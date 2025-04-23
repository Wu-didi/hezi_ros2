import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nvidia/vcii/hezi_ros2/src/demo1/follow_traj_wd/install/follow_traj_wd'
