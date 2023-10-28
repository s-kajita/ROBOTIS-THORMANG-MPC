[ control info ]
control_cycle = 10   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE | DEFAULT JOINT
/dev/ttyUSB0 | 2000000  | joint1
/dev/ttyUSB1 | 2000000  | joint7

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME     | BULK READ ITEMS
# --------- right leg ---------- 
dynamixel | /dev/ttyUSB0 | 2   | XM540-W270     | 2.0      | joint0       | present_position, present_current
dynamixel | /dev/ttyUSB0 | 3   | XM540-W270     | 2.0      | joint1       | present_position, present_current
dynamixel | /dev/ttyUSB0 | 4   | XM540-W270     | 2.0      | joint1s      | present_position, present_current
dynamixel | /dev/ttyUSB0 | 5   | XM540-W270     | 2.0      | joint2       | present_position, present_current
dynamixel | /dev/ttyUSB0 | 6   | XM540-W270     | 2.0      | joint2s      | present_position, present_current
dynamixel | /dev/ttyUSB0 | 7   | XM540-W270     | 2.0      | joint3       | present_position, present_current
dynamixel | /dev/ttyUSB0 | 8   | XM540-W270     | 2.0      | joint3s      | present_position, present_current
dynamixel | /dev/ttyUSB0 | 9   | XM540-W270     | 2.0      | joint4       | present_position, present_current, external_port_data_1, external_port_data_2
dynamixel | /dev/ttyUSB0 | 10  | XM540-W270     | 2.0      | joint4s      | present_position, present_current
dynamixel | /dev/ttyUSB0 | 11  | XM540-W270     | 2.0      | joint5       | present_position, present_current, external_port_data_1, external_port_data_2

dynamixel | /dev/ttyUSB1 | 13   | XM540-W270     | 2.0      | joint6      | present_position, present_current
dynamixel | /dev/ttyUSB1 | 14   | XM540-W270     | 2.0      | joint7      | present_position, present_current
dynamixel | /dev/ttyUSB1 | 15   | XM540-W270     | 2.0      | joint7s     | present_position, present_current
dynamixel | /dev/ttyUSB1 | 16   | XM540-W270     | 2.0      | joint8      | present_position, present_current
dynamixel | /dev/ttyUSB1 | 17   | XM540-W270     | 2.0      | joint8s     | present_position, present_current
dynamixel | /dev/ttyUSB1 | 18   | XM540-W270     | 2.0      | joint9      | present_position, present_current
dynamixel | /dev/ttyUSB1 | 19   | XM540-W270     | 2.0      | joint9s     | present_position, present_current
dynamixel | /dev/ttyUSB1 | 20   | XM540-W270     | 2.0      | joint10     | present_position, present_current
dynamixel | /dev/ttyUSB1 | 21   | XM540-W270     | 2.0      | joint10s    | present_position, present_current
dynamixel | /dev/ttyUSB1 | 22   | XM540-W270     | 2.0      | joint11     | present_position, present_current
