import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gramisaam/mtre4820-final-project/install/node_example_pkg'
