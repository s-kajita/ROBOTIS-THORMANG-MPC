[ control info ]
control_cycle = 10   # milliseconds
#control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE | DEFAULT JOINT
/dev/ttyUSB0 | 2000000  | joint4

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME     | BULK READ ITEMS
dynamixel | /dev/ttyUSB0 | 2   | XM540-W270     | 2.0      | joint5       | present_position, present_current
dynamixel | /dev/ttyUSB0 | 3   | XM540-W270     | 2.0      | joint4       | present_position, present_current
dynamixel | /dev/ttyUSB0 | 4   | XM540-W270     | 2.0      | joint4s      | present_position, present_current
