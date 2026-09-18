import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rudra/custom_flightmode/install/example_mode_with_rclpy_py'
