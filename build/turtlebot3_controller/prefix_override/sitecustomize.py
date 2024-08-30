import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ta/turtlebot3_ws/src/turtlebot3_controller/install/turtlebot3_controller'
