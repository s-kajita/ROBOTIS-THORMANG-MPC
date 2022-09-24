[ control info ]
control_cycle = 20   # milliseconds
#control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE | DEFAULT JOINT
/dev/ttyUSB0 | 57600    | joint1

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME     | BULK READ ITEMS
dynamixel | /dev/ttyUSB0 | 1   | XM540-W270     | 2.0      | joint1       | present_position, present_current
dynamixel | /dev/ttyUSB0 | 2   | XM540-W270     | 2.0      | joint2       | present_position, present_current
