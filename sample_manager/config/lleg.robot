[ control info ]
control_cycle = 10   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE | DEFAULT JOINT
/dev/ttyUSB1 | 2000000  | joint7

[ device info ]
# TYPE    | PORT NAME    | ID   | MODEL          | PROTOCOL | DEV NAME    | BULK READ ITEMS
dynamixel | /dev/ttyUSB1 | 14   | XM540-W270     | 2.0      | joint7      | present_position, present_current
dynamixel | /dev/ttyUSB1 | 15   | XM540-W270     | 2.0      | joint7s     | present_position, present_current
dynamixel | /dev/ttyUSB1 | 16   | XM540-W270     | 2.0      | joint8      | present_position, present_current
dynamixel | /dev/ttyUSB1 | 17   | XM540-W270     | 2.0      | joint8s     | present_position, present_current
dynamixel | /dev/ttyUSB1 | 18   | XM540-W270     | 2.0      | joint9      | present_position, present_current
dynamixel | /dev/ttyUSB1 | 19   | XM540-W270     | 2.0      | joint9s     | present_position, present_current
dynamixel | /dev/ttyUSB1 | 20   | XM540-W270     | 2.0      | joint10     | present_position, present_current
dynamixel | /dev/ttyUSB1 | 21   | XM540-W270     | 2.0      | joint10s    | present_position, present_current
dynamixel | /dev/ttyUSB1 | 22   | XM540-W270     | 2.0      | joint11     | present_position, present_current
