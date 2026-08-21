import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dryanguasr/proyectos/gazebo-tutorial/gazebo-codex-mobile-manipulator/install/mobile_manipulator'
