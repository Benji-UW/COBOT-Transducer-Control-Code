struct_key = {}
'''Dictionary containing the struct format data for 
different data types, as well as the standard number
of bytes each data type requires'''
struct_key['char'] = ('c', 1)
struct_key['signed char'] = ('b',1)
struct_key['unsigned char'] = ('B',1)
struct_key['bool'] = ('?',1)
struct_key['short'] = ('h',2)
struct_key['unsigned short'] = ('H',2)
struct_key['int'] = ('i',4)
struct_key['unsigned int'] = ('I',4)
struct_key['long'] = ('l',4)
struct_key['unsigned long'] = ('L',4)
struct_key['long long'] = ('q',8)
struct_key['unsigned long long'] = ('Q',8)
struct_key['float'] = ('f',4)
struct_key['double'] = ('d',8)



robot_mode_data = []
robot_mode_data.append(("int", "packageSize"))
robot_mode_data.append(("unsigned char", "packageType"))
robot_mode_data.append(("long long", "timestamp"))
robot_mode_data.append(("bool", "isRealRobotConnected"))
robot_mode_data.append(("bool", "isRealRobotEnabled"))
robot_mode_data.append(("bool", "isRobotPowerOn"))
robot_mode_data.append(("bool", "isEmergencyStopped"))
robot_mode_data.append(("bool", "isProtectiveStopped"))
robot_mode_data.append(("bool", "isProgramRunning"))
robot_mode_data.append(("bool", "isProgramPaused"))
robot_mode_data.append(("unsigned char", "robotMode"))
robot_mode_data.append(("unsigned char", "controlMode"))
robot_mode_data.append(("double", "targetSpeedFraction"))
robot_mode_data.append(("double", "speedScaling"))
robot_mode_data.append(("double", "targetSpeedFractionLimit"))
robot_mode_data.append(("unsigned char", "reserved"))

joint_data = []
joint_data.append(("int", "packageSize"))
joint_data.append(("unsigned char", "packageType"))

cartesian_info = []
cartesian_info.append(("int", "packageSize"))
cartesian_info.append(("unsigned char", "packageType"))
cartesian_info.append(("double", "X"))
cartesian_info.append(("double", "Y"))
cartesian_info.append(("double", "Z"))
cartesian_info.append(("double", "Rx"))
cartesian_info.append(("double", "Ry"))
cartesian_info.append(("double", "Rz"))
cartesian_info.append(("double", "TCPOffsetX"))
cartesian_info.append(("double", "TCPOffsetY"))
cartesian_info.append(("double", "TCPOffsetZ"))
cartesian_info.append(("double", "TCPOffsetRx"))
cartesian_info.append(("double", "TCPOffsetRy"))
cartesian_info.append(("double", "TCPOffsetRz"))



sub_package_types = {}
sub_package_types[0] = robot_mode_data
sub_package_types[4] = cartesian_info

