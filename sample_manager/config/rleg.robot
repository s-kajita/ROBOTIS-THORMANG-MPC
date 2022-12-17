[ control info ]
control_cycle = 10   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE | DEFAULT JOINT
/dev/ttyUSB0 | 2000000  | joint1

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME     | BULK READ ITEMS
# --------- right leg ---------- 
dynamixel | /dev/ttyUSB0 | 3   | XM540-W270     | 2.0      | joint1       | present_position, present_current
dynamixel | /dev/ttyUSB0 | 4   | XM540-W270     | 2.0      | joint1s      | present_position, present_current
dynamixel | /dev/ttyUSB0 | 5   | XM540-W270     | 2.0      | joint2       | present_position, present_current
dynamixel | /dev/ttyUSB0 | 6   | XM540-W270     | 2.0      | joint2s      | present_position, present_current
