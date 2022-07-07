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

# ROSBAG

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
for i in range(6):
    joint_data.append(("double", f"joint {i+1} q_actual"))
    joint_data.append(("double", f"joint {i+1} q_target"))
    joint_data.append(("double", f"joint {i+1} qd_actual"))
    joint_data.append(("float", f"joint {i+1} i_actual"))
    joint_data.append(("float", f"joint {i+1} V_actual"))
    joint_data.append(("float", f"joint {i+1} T_motor"))

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

kinematics_info = []
kinematics_info.append(("int", "packageSize"))
kinematics_info.append(("unsigned char", "packageType"))
for i in range(6):
    kinematics_info.append(("unsigned int", f"joint {i+1} checksum"))
for i in range(6):
    kinematics_info.append(("double", f"joint {i+1} DHtheta"))
for i in range(6):
    kinematics_info.append(("double", f"joint {i+1} DHa"))
for i in range(6):
    kinematics_info.append(("double", f"joint {i+1} Dhd"))
for i in range(6):
    kinematics_info.append(("double", f"joint {i+1} Dhalpha"))
kinematics_info.append(("unsigned int", "calibration status"))





sub_package_types = {}
sub_package_types[0] = robot_mode_data
sub_package_types[4] = cartesian_info
sub_package_types[1] = joint_data
sub_package_types[5] = kinematics_info


message_type = {}
message_type[20] = "ROBOT_MESSAGE"
message_type[16] = "ROBOT_STATE"
message_type[22] = "HMC_MESSAGE"
message_type[5] = "MODBUS_INFO_MESSAGE"
message_type[-1] = "DISCONNECT"
message_type[23] = "SAFETY_SETUP_BROADCAST_MESSAGE"
message_type[24] = "SAFETY_COMPLIANCE_FUCK"
message_type[25] = "PROGRAM_STATE_MESSAGE"

package_type = {}
package_type[0] = "ROBOT_MODE_DATA"
package_type[1] = "JOINT_DATA"
package_type[2] = "TOOL_DATA"
package_type[3] = "MASTERBOARD_DATA"
package_type[4] = "CARTESIAN_INFO"
package_type[5] = "KINEMATICS_INFO"
package_type[6] = "CONFIGURATION_DATA"
package_type[7] = "FORCE_MODE_DATA"
package_type[8] = "ADDITIONAL_INFO"
package_type[9] = "NEEDED_FOR_CALIB_DATA"
package_type[11] = "TOOL_COMM_INFO"
package_type[12] = "TOOL_MODE_INFO"
package_type[13] = "SINGULARITY_INFO"
package_type[-1] = "fuck"
package_type[255] = "fuck"
package_type[10] = "fuck"
package_type[14] = "fuck"

import json
import os

path = os.path.dirname(__file__)
path = path + '\\UR3e_config.json'
all_data = {}
all_data["package_type"] = package_type
all_data["sub_package_types"] = sub_package_types
all_data["struct_key"] = struct_key
all_data["package_type"] = package_type

with open(path, 'w+') as outfile:
    json.dump(all_data, outfile, indent=2)