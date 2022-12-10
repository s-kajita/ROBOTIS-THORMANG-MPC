[ control info ]
control_cycle = 10   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE | DEFAULT JOINT
/dev/ttyUSB0 | 2000000  | joint7

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME     | BULK READ ITEMS
# --------- right leg ---------- 
dynamixel | /dev/ttyUSB0 | 13   | XM540-W270     | 2.0      | joint7       | present_position, present_current
dynamixel | /dev/ttyUSB0 | 14   | XM540-W270     | 2.0      | joint7s      | present_position, present_current
dynamixel | /dev/ttyUSB0 | 15   | XM540-W270     | 2.0      | joint8       | present_position, present_current
dynamixel | /dev/ttyUSB0 | 16   | XM540-W270     | 2.0      | joint8s      | present_position, present_current
