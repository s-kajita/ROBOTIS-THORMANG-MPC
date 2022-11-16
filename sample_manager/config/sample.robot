[ control info ]
control_cycle = 10   # milliseconds
#control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE | DEFAULT JOINT
/dev/ttyUSB0 | 2000000  | joint5
/dev/ttyUSB1 | 2000000  | joint3

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME     | BULK READ ITEMS
dynamixel | /dev/ttyUSB0 | 2   | XM540-W270     | 2.0      | joint5       | present_position, present_current
dynamixel | /dev/ttyUSB0 | 3   | XM540-W270     | 2.0      | joint4       | present_position, present_current
dynamixel | /dev/ttyUSB0 | 4   | XM540-W270     | 2.0      | joint4s      | present_position, present_current
dynamixel | /dev/ttyUSB1 | 5   | XM540-W270     | 2.0      | joint3       | present_position, present_current
dynamixel | /dev/ttyUSB1 | 6   | XM540-W270     | 2.0      | joint3s      | present_position, present_current
dynamixel | /dev/ttyUSB1 | 7   | XM540-W270     | 2.0      | joint2       | present_position, present_current
dynamixel | /dev/ttyUSB1 | 8   | XM540-W270     | 2.0      | joint2s      | present_position, present_current
