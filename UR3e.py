"""
Unit convention: m/kg/s

"""
import numpy as np
import socket
import struct
import time
from math import *
import UR3e_config as uconf
import logging

def _reg_convert(reg, rtype=None):
    coeff = 1.
    if rtype == 'pos':
        coeff = 10000.
    elif rtype == 'angle':
        coeff = 1000.
    elif rtype == 'tcp_offset':
        coeff = 1000.
    if reg == '':
        reg = '0000'
    reg = int(reg, 16)
    if reg <= 32677:
        reg = float(reg) / coeff
    else:
        reg = 65535 - reg
        reg = -float(reg) / coeff
    return reg


class UR3e:
    def __init__(self):
        # Initialize variables
        self.pos:np.ndarray = np.zeros((3, 1))
        self.angle:np.ndarray = np.zeros((3, 1))

        self.joints:np.ndarray = np.zeros((6,1))
        self.initial_joints:np.ndarray = np.zeros((6,1))
        
        # TCP offset and rotation refer to the displacement of the TCP from 
        # the end effector, they are constant throughout the operation of 
        # the robot.
        self.tcp_offset: np.ndarray = np.zeros((3, 1))
        self.tcp_rot: np.ndarray = np.zeros((3, 1))

        self.freedrive_active: bool = False

        self.max_disp = 0.1
        self.initial_pos: np.ndarray = np.zeros((3, 1))
        self.initial_angle: np.ndarray = np.zeros((3, 1))
        self.max_disp_delta = 0.002

        self.acc = 0.1
        self.velocity = 0.025
        self.acc_rot = 0.4
        self.vel_rot = 0.05

        self.robot_socket = None
        self.modbus_socket = None
        self.data_socket = None

        self.base = 'tcp'
        # self.current_relative_target = ((0,0,0),(0,0,0))
        self.current_relative_target: np.ndarray = np.zeros(6)

        self.rotation_matrix = np.zeros((3, 3))
        self.rotation_matrix_inv = np.zeros((3, 3))

        self.sleep_time = 0.001

        self.servo_setpoint = np.zeros((3, 1))
        self.servo_prev_setpoint = np.zeros((3, 1))
        self.servo_error = 0.0
        self.servo_error_nm1 = 0.0
        self.servo_error_int = 0.0
        self.servo_t = 0.0
        self.servo_t_nm1 = 0.0
        self.servo_Kp = 0.01
        self.servo_Ki = 0.0
        self.servo_Kd = 0.0
        self.servo_lag = 0.0
        self.servo_type = None
        self.servo_cmd = np.zeros((3, 4))

        self.logger = logging.getLogger(__name__)
        self.logger.info("Robot fully initialized")
        

    def connect(self,
            robot_ip:str,
            robot_port:int=30020,
            modbus_port:int=502,
            data_ip:str="192.168.0.5",
            data_port:int=508) -> tuple[bool,str]:
        """
        Connect to the robot.

        :param str robot_ip: IP address of the robot
        :param int robot_port: port of the robot
        :param int modbus_port: modbus port of the robot
        :return:
        """
        self.robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.robot_socket.settimeout(5)
        try:
            self.robot_socket.connect((robot_ip, robot_port))
        except socket.gaierror as e:
            self.logger.error('Connection error to robot: %s' % e)
            return (False, e)
        except socket.timeout as e:
            self.logger.error("Connection timed out")
            return (False,e)

        self.modbus_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.modbus_socket.connect((robot_ip, modbus_port))
        except socket.gaierror as e:
            self.logger.info('Connection error to modbus: %s' % e)
            return (False, e)

        
        self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.data_socket.connect((data_ip, data_port))
        except socket.gaierror as e:
            self.logger.info(f'Connection error due to data port: {e}')
            return (False, e)
        self.logger.info('Connected to data_port')

        return (True, '')

    def disconnect(self):
        self.robot_socket.close()

    def set_parameters(self,
                    acc:float = None,
                    velocity:float = None,
                    acc_rot:float = None,
                    vel_rot:float = None,
                    base=None):
        if acc is not None:
            self.acc = acc
        if velocity is not None:
            self.velocity = velocity
        if acc_rot is not None:
            self.acc_rot = acc_rot
        if vel_rot is not None:
            self.vel_rot = vel_rot
        if base is not None:
            if base == 'base':
                self.base = base
            elif base == 'tcp':
                self.base = base

    def set_max_displacement(self, max_disp, max_disp_delta=0.002):
        self.max_disp = max_disp
        self.max_disp_delta = max_disp_delta

    def set_initial_pos(self):
        self.initial_pos = np.copy(self.pos)
        self.initial_angle = np.copy(self.angle)
        self.initial_joints = np.copy(self.joints)

        self.data_socket.send(b"TODO set_origin ")
        self.logger.info('THIS IS HUGE WHY ISNT IT GOING')
        time.sleep(0.01)

    def movel(self, pos_to, angle_to, t=0.0) -> bool:
        if self._check_move_displacement(pos_to):
            cmd = (b'movel(p[%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f],a=%1.2f,v='
                b'%1.2f,t=%2.2f,r=0)' % (pos_to[0], pos_to[1], pos_to[2],
                angle_to[0], angle_to[1], angle_to[2], self.acc, self.velocity, t))
            self.robot_socket.send(b'sync()\n')
            time.sleep(self.sleep_time)
            self.robot_socket.send(cmd + b'\n')
            if t == 0.0:
                dist = np.linalg.norm(np.subtract(pos_to, self.pos))  # move distance
                t_a = self.velocity / self.acc  # time to accelerate
                t_v = dist / self.velocity
                time.sleep(t_a + t_v)
            else:
                time.sleep(t)
            return True
        return False

    def speedl(self, speed_vect, rotation_vect, lag) -> bool:
        if self._check_speed_displacement(speed_vect):
            # multiplies the incoming motion vectors by the velocity
            speed_vect = self.velocity * speed_vect
            rotation_vect = self.vel_rot * rotation_vect
            # magnitude of the vector
            speed = np.linalg.norm(speed_vect)
            if speed > self.velocity:
                speed_vect = (self.velocity / speed) * speed_vect
            speed = np.linalg.norm(rotation_vect)
            if speed > self.vel_rot:
                rotation_vect = (self.vel_rot / speed) * rotation_vect
            # Normalize the vectors so that the magnitude is equal to vel_rot

            # damn bruh idk what this does
            speed_vect = self._change_base(speed_vect)
            rotation_vect = self._change_base(rotation_vect)

            # cmd = b'speedl([%1.3f,%1.3f,%1.3f,%2.3f,%2.3f,%2.3f],%1.3f,%2.3f,%1.3f)' % \
            #       (speed_vect[0], speed_vect[1], speed_vect[2], rotation_vect[0], rotation_vect[1], rotation_vect[2],
            #        self.acc, lag, self.acc_rot)
            cmd = (b'speedl([%1.3f,%1.3f,%1.3f,%2.3f,%2.3f,%2.3f],%1.3f,%2.3f)'
                % (speed_vect[0], speed_vect[1], speed_vect[2], rotation_vect[0],
                rotation_vect[1], rotation_vect[2], self.acc, lag))
            self.robot_socket.send(cmd + b'\n')
            return True
        else:
            self.stopl()
        return False

    def ur_servo(self, dpos, dangle=None, t=0.002, lh_t=0.02, gain=800):
        pos = np.copy(self.initial_pos) + self._change_base(dpos)
        if dangle is not None:
            angle = np.copy(self.initial_angle) + dangle
        else:
            angle = np.copy(self.initial_angle)
        cmd = (b'servoj(get_inverse_kin(p[%1.4f,%1.4f,%1.4f,%2.4f,%2.4f,%2.4f])'
                b',0,0,%2.3f,%2.3f,%d)' % (pos[0], pos[1], pos[2], angle[0],
                angle[1], angle[2], t, lh_t, gain))
        self.robot_socket.send(cmd + b'\n')

    def init_servo(self, lag, Kp=0.01, Ki=0.0, Kd=0.0, servo_type='delta'):
        self.servo_t = time.time()
        self.servo_t_nm1 = self.servo_t
        self.servo_lag = lag
        self.servo_Kp = Kp
        self.servo_Ki = Ki
        self.servo_Kd = Kd
        self.servo_type = servo_type
        self.servo_error = 0.0
        self.servo_error_nm1 = 0.0
        self.servo_error_int = 0.0
        self.servo_setpoint = np.copy(self.pos)
        self.servo_prev_setpoint = np.zeros((3, 1))
        self.initial_pos = np.copy(self.pos)
        self.initial_angle = np.copy(self.angle)

    def set_servo_parameters(self, Kp=None, Ki=None, Kd=None, lag=None):
        if Kp is not None:
            self.servo_Kp = Kp
        if Ki is not None:
            self.servo_Ki = Ki
        if Kd is not None:
            self.servo_Kd = Kd
        if lag is not None:
            self.servo_lag = lag

    def update_servo(self, setpoint=None):
        self.servo_error = self.pos - self.initial_pos - self._change_base(setpoint)
        self.servo_t = time.time()
        dt = self.servo_t - self.servo_t_nm1

        if self.servo_Ki != 0:
            self.servo_error_int = (self.servo_error_int + 0.5 * 
                (self.servo_error - self.servo_error_nm1) * dt)

        servo_prop = self.servo_Kp * self.servo_error
        servo_int = self.servo_Ki * self.servo_error_int
        if dt > 0.001:
            servo_dev = (self.servo_Kd * (self.servo_error -
                self.servo_error_nm1) / dt)
        else:
            servo_dev = 0.0
        servo = servo_prop + servo_int + servo_dev
        if self._check_speed_displacement(-servo):
            cmd = (b'speedl([%1.5f,%1.5f,%1.5f,0,0,0],%1.3f,%2.3f,%1.3f)' % 
                (-servo[0], -servo[1], -servo[2], 2.0, self.servo_lag, 2.0))
            self.robot_socket.send(cmd + b'\n')
        else:
            self.stopl()

        self.servo_t_nm1 = self.servo_t
        self.servo_error_nm1 = self.servo_error

    def stopl(self, acc=1) -> bool:
        self.robot_socket.send(b'stopl(%2.3f)\n' % acc)
        return True

    def freedrive(self) -> bool:
        self.robot_socket.send(b'freedrive_mode()\n')
        return True

    def end_freedrive(self) -> bool:
        self.robot_socket.send(b'end_freedrive_mode()\n')
        self._get_pos()
        self.get_joint_angles()
        self.initial_pos = np.copy(self.pos)
        self.initial_angle = np.copy(self.angle)
        return True

    def update(self):
        self._get_pos()
        self.get_joint_angles()

        # UR uses the axis-angle system for rotation vectors
        total_angle = np.add(self.angle, self.tcp_rot)
        theta = np.linalg.norm(total_angle)
        ux = total_angle[0] / theta
        uy = total_angle[1] / theta
        uz = total_angle[2] / theta

        self.rotation_matrix[0, 0] = cos(theta) + ux ** 2 * (1. - cos(theta))
        self.rotation_matrix[0, 1] = ux * uy * (1. - cos(theta)) - uz * sin(theta)
        self.rotation_matrix[0, 2] = ux * uz * (1. - cos(theta)) + uy * sin(theta)
        self.rotation_matrix[1, 0] = ux * uy * (1. - cos(theta)) + uz * sin(theta)
        self.rotation_matrix[1, 1] = cos(theta) + uy ** 2 * (1. - cos(theta))
        self.rotation_matrix[1, 2] = uy * uz * (1. - cos(theta)) - ux * sin(theta)
        self.rotation_matrix[2, 0] = ux * uz * (1. - cos(theta)) - uy * sin(theta)
        self.rotation_matrix[2, 1] = uy * uz * (1. - cos(theta)) + ux * sin(theta)
        self.rotation_matrix[2, 2] = cos(theta) + uz ** 2 * (1. - cos(theta))

        self.rotation_matrix_inv = np.linalg.inv(self.rotation_matrix)

    def initialize(self):
        self._get_tcp_offset()
        self.update()
        self.initial_pos = np.copy(self.pos)
        self.initial_angle = np.copy(self.angle)

    def get_delta_pos(self):
        '''Returns the current position and angle of either the end effector or the
        tool on the end of the arm, in the form of a pair of vertical (3,1) sized
        numpy matrices.'''
        return (self._change_base(self.pos - self.initial_pos, inv=True), 
               self._change_base(self.angle - self.initial_angle, inv=True))

    def get_joint_angles(self):
        # Get registry 270 to 275 in modbus, ie 0x190 w/ read size of 6
        cmd = b'\x00\x04\x00\x00\x00\x06\x00\x03\x01\x0E\x00\x06'
        self.modbus_socket.send(cmd)
        reg = self.modbus_socket.recv(1024)
        self.logger.debug(f"Joint angle modbus reply recieved: {reg}")
        
        # Get x, registry 270 in modbus, ie 0x190
        self.joints[0] = _reg_convert(reg[-12:-10].hex(), 'angle')

        # Get y, registry 271 in modbus, ie 0x191
        self.joints[1] = _reg_convert(reg[-10:-8].hex(), 'angle')

        # Get z, registry 272 in modbus, ie 0x192
        self.joints[2] = _reg_convert(reg[-8:-6].hex(), 'angle')

        # Get Rx, registry 273 in modbus, ie 0x193
        self.joints[3] = _reg_convert(reg[-6:-4].hex(), 'angle')

        # Get Ry, registry 274 in modbus, ie 0x194
        self.joints[4] = _reg_convert(reg[-4:-2].hex(), 'angle')

        # Get Rz, registry 275 in modbus, ie 0x195
        self.joints[5] = _reg_convert(reg[-2:].hex(), 'angle')

    def _get_pos(self):
        # Get registry 400 to 405 in modbus, ie 0x190 w/ read size of 6
        cmd = b'\x00\x04\x00\x00\x00\x06\x00\x03\x01\x90\x00\x06'
        self.modbus_socket.send(cmd)
        reg = self.modbus_socket.recv(1024)
        self.logger.debug(f"Position modbus reply recieved: {reg}")
        
        # Get x, registry 400 in modbus, ie 0x190
        self.pos[0] = _reg_convert(reg[-12:-10].hex(), 'pos')

        # Get y, registry 401 in modbus, ie 0x191
        self.pos[1] = _reg_convert(reg[-10:-8].hex(), 'pos')

        # Get z, registry 402 in modbus, ie 0x192
        self.pos[2] = _reg_convert(reg[-8:-6].hex(), 'pos')

        # Get Rx, registry 403 in modbus, ie 0x193
        self.angle[0] = _reg_convert(reg[-6:-4].hex(), 'angle')

        # Get Ry, registry 404 in modbus, ie 0x194
        self.angle[1] = _reg_convert(reg[-4:-2].hex(), 'angle')

        # Get Rz, registry 405 in modbus, ie 0x195
        self.angle[2] = _reg_convert(reg[-2:].hex(), 'angle')

    def _get_tcp_offset(self):
        # Get x offset, registry 420 in modbus, ie 0x1A4
        cmd = b'\x00\x04\x00\x00\x00\x06\x00\x03\x01\xA4\x00\x01'
        self.modbus_socket.send(cmd)
        reg = self.modbus_socket.recv(1024)
        reg = reg.replace(b'\x00\x04\x00\x00\x00\x05\x00\x03\x02', b'')
        #reg = reg.encode('hex')
        reg = reg.hex()
        self.tcp_offset[0] = _reg_convert(reg, 'tcp_offset')

        # Get y offset, registry 421 in modbus, ie 0x1A5
        cmd = b'\x00\x04\x00\x00\x00\x06\x00\x03\x01\xA5\x00\x01'
        self.modbus_socket.send(cmd)
        reg = self.modbus_socket.recv(1024)
        reg = reg.replace(b'\x00\x04\x00\x00\x00\x05\x00\x03\x02', b'')
        #reg = reg.encode('hex')
        reg = reg.hex()
        self.tcp_offset[1] = _reg_convert(reg, 'tcp_offset')

        # Get z offset, registry 422 in modbus, ie 0x1A6
        cmd = b'\x00\x04\x00\x00\x00\x06\x00\x03\x01\xA6\x00\x01'
        self.modbus_socket.send(cmd)
        reg = self.modbus_socket.recv(1024)
        reg = reg.replace(b'\x00\x04\x00\x00\x00\x05\x00\x03\x02', b'')
        #reg = reg.encode('hex')
        reg = reg.hex()
        self.tcp_offset[2] = _reg_convert(reg, 'tcp_offset')

        # Get Rx rotation, registry 423 in modbus, ie 0x1A7
        cmd = b'\x00\x04\x00\x00\x00\x06\x00\x03\x01\xA7\x00\x01'
        self.modbus_socket.send(cmd)
        reg = self.modbus_socket.recv(1024)
        reg = reg.replace(b'\x00\x04\x00\x00\x00\x05\x00\x03\x02', b'')
        #reg = reg.encode('hex')
        reg = reg.hex()
        self.tcp_rot[0] = _reg_convert(reg, 'angle')

        # Get Ry rotation, registry 424 in modbus, ie 0x1A8
        cmd = b'\x00\x04\x00\x00\x00\x06\x00\x03\x01\xA8\x00\x01'
        self.modbus_socket.send(cmd)
        reg = self.modbus_socket.recv(1024)
        reg = reg.replace(b'\x00\x04\x00\x00\x00\x05\x00\x03\x02', b'')
        #reg = reg.encode('hex')
        reg = reg.hex()
        self.tcp_rot[1] = _reg_convert(reg, 'angle')

        # Get Rz rotation, registry 425 in modbus, ie 0x1A9
        cmd = b'\x00\x04\x00\x00\x00\x06\x00\x03\x01\xA9\x00\x01'
        self.modbus_socket.send(cmd)
        reg = self.modbus_socket.recv(1024)
        reg = reg.replace(b'\x00\x04\x00\x00\x00\x05\x00\x03\x02', b'')
        #reg = reg.encode('hex')
        reg = reg.hex()
        self.tcp_rot[2] = _reg_convert(reg, 'angle')
        
    def _change_base(self, vect, inv=False):
        if self.base == 'base':
            return vect
        elif self.base == 'tcp':
            if inv:
                return np.dot(self.rotation_matrix_inv, vect)
            else:
                return np.dot(self.rotation_matrix, vect)
        return None

    def _check_move_displacement(self, pos_to) -> bool:
        disp = np.linalg.norm(np.subtract(pos_to, self.initial_pos))
        if disp >= self.max_disp:
            return False
        else:
            return True

    def _check_speed_displacement(self, speed_vect) -> bool:
        tcp_direction = np.subtract(self.pos, self.initial_pos)
        #print(tcp_direction)
        disp = np.linalg.norm(tcp_direction)
        #print(disp)
        tcp_direction = tcp_direction / disp
        speed_direction = speed_vect / np.linalg.norm(speed_vect)

        if disp < self.max_disp - self.max_disp_delta:
            return True
        return False

    def _powerdown(self) -> bool:
        print("attempting to power down")
        self.robot_socket.send(b'powerdown()\n')
        return True

    def generic_test_command_response(self, command, no_bytes=1024) \
            -> tuple[bytes,str]:
        self.robot_socket.send(command)
        reg = self.robot_socket.recv(no_bytes)
        return reg,reg.hex()

    def generic_io_response(self, command, no_bytes=4096):
        '''This is a white whale of the project, send a generic URscript command
        to the robot and parse all the data that it sends back. This is
        essential for doing more intricate motions with the end effector.'''

        # Send the transmit requeset and listen for 4096 bytes back 
        # (enough to store any packet)
        self.robot_socket.send(command)
        data = self.robot_socket.recv(no_bytes)

        # initialize i to track position in the packet
        i = 0
        logstring = f"Toggling the command {command}\n"

        if data:
            logstring += "Logging new packet.\n"

            # extract packet length, timestamp, and packet type from the start
            # of the packet
            # Length of overall package: 32bit integer
            msglen = (struct.unpack('!i', data[0:4]))[0]
            # Unsigned char messageType (corresponds to hard-coded message types)
            print(data[4])
            # msgtype = (struct.unpack('!b', data[4]))[0]
            msgtype = data[4]

            logstring += f"Message type: {msgtype}, {uconf.message_type[msgtype]}\n"
            logstring += f"Message length: {msglen}\n"
            logstring += f"Full raw message data: {data}\n"

            i = 5
            while i+5 < msglen:
                # Extract the length and type of the message within the packet
                # msglen is the first four bytes
                packlen = (struct.unpack('!i', data[i:i+4]))[0]
                print(data[i+4])
                # packtype = (struct.unpack('!b', data[i+4]))[0]
                packtype = data[i+4]
                logstring += f"Sub-package length: {packlen}\n"
                logstring += f"Sub-package type: {packtype}, {uconf.package_type[packtype]}\n"

                logstring += f"Sub-package Data (including 5 header bits): {data[i:i+packlen]}\n"

                # If this sub-package is of a type I've manually described (UR3e_config.py)
                if uconf.sub_package_types.keys().__contains__(packtype):
                    # sub-package iterator
                    j = i

                    # 'package' contains an ordered list of tuples representing the data format
                    # defined by this sub-package type. Each tuple is in the form (data_type, field_name)
                    package = uconf.sub_package_types[packtype]
                    for k in range(len(package)):
                        # extract the type and name from the current item
                        (data_type, name) = package[k]
                        # look up the appropriate "struct.unpack" format and the number of bytes
                        # associated with this data type (('B', 1) for an unsigned int for example)
                        (struct_format, byte_len) = uconf.struct_key[data_type]
                        # Unpack the value from the data and store it's name and value in the logstring
                        val = (struct.unpack('!' + struct_format, data[j:j+byte_len]))[0]
                        logstring += f"\t{name}: {val}\n"
                        # Increment j to mark our new point in the string
                        j += byte_len

                i += packlen      

        return logstring

    def toggle_freedrive(self):
        if self.freedrive_active:
            self.data_socket.send(b'SET FREE %i ' % 0)
        else:
            self.data_socket.send(b'SET FREE %i ' % 1)
        time.sleep(0.01)
        self.data_socket.send(b'TODO check_free ')
        time.sleep(0.01)
        self.freedrive_active = not self.freedrive_active

    def data_string_io(self, to_send) -> bytes:
        '''Recieves a bytes-like object and sends it to the data port on the robot.
        Return strue if the transmission is successful.'''
        self.data_socket.send(to_send)
        return self.data_socket.recv(1024)

    def test_URScript_API(self):
        '''This experiment has failed. The following code should now power off 
        the robot, and yet it does. Something here is clearly wrong.'''
        
        cmds = [b"test_script()\n"]

        for cmd in cmds:
            self.robot_socket.send(cmd)

    def movel_to_target(self, next_point: np.ndarray):
        '''Moves the robot to the given target point in the robot's floating
        coordinate system.
        
        next_point should be given in the form of a (6,) ndarray.
        '''
        # t_pos,t_angle = (next_point[0],next_point[1])
        tx,ty,tz,tRx,tRy,tRz = next_point.tolist()
        # tRx,tRy,tRz = t_angle

        self.current_relative_target = next_point

        # Note: The Tar_x values on the python side are in mm, and they'll
        # need to be converted to m on the robot end
        self.data_socket.send(b'SET TAR_X %i ' % (tx * 10))
        self.data_socket.send(b'SET TAR_Y %i ' % (ty * 10))
        self.data_socket.send(b'SET TAR_Z %i ' % (tz * 10))
        self.data_socket.send(b'SET TAR_Rx %i ' % (np.deg2rad(tRx) * 10000))
        self.data_socket.send(b'SET TAR_Ry %i ' % (np.deg2rad(tRy) * 10000))
        self.data_socket.send(b'SET TAR_Rz %i ' % (np.deg2rad(tRz) * 10000))

        self.logger.info("Robot requested to go to point "
            f"(({tx},{ty},{tz}),({tRx},{tRy},{tRz}))")
        self.data_socket.send(b'TODO move2tar ')
        time.sleep(0.01)
        self.data_socket.send(b'SET atTar %i ' % (-1))
        time.sleep(0.01)

    def get_current_rel_target(self) -> np.ndarray:
        '''Returns the current relative target, in the form of a (6,) ndarray'''
        return self.current_relative_target

    def at_tar(self) -> bool:
        self.data_socket.send(b"GET atTar")
        response = self.data_socket.recv(1024)

        # print(b"Debugging atTar: " + response)
        return (response == b"atTar 1")

    def wait_for_at_tar(self) -> int:
        at_tar = False
        i = 0
        now = time.time()
        while not at_tar and i < 1000:
            time.sleep(0.02)
            self.data_socket.send(b"GET atTar")
            at_tar = (self.data_socket.recv(1024) == b"atTar 1")
            i+=1
            if i % 4 == 0:
                self.data_socket.send(b'TODO move2tar ')

        self.logger.info(f"Robot moved to target in {time.time() - now} seconds.")

        return i


class Fake_UR3e(UR3e):
    '''This class is a bare-minimum skeleton of the UR3 module, meant
    only to allow headless tests to run without crashing or heavily
    modifying my code elsewhere.'''
    def __init__(self):
        super().__init__()

    def disconnect(self):
        pass
    
    def set_parameters(self,acc=None,velocity=None,acc_rot=None,vel_rot=None,base=None):
        pass

    def set_initial_pos(self):
        pass

    def movel(self, pos_to, angle_to, t=0.0):
        return True

    def update(self):
        pass

    def get_delta_pos(self):
        '''Returns a pair of vertical (3,1) sized
        numpy matrices defaulting to -1'''
        a = np.ones((3, 1))*-1
        return a,a

    def movel_to_target(self, next_point: np.ndarray):
        '''Fake'''
        self.current_relative_target = next_point
        time.sleep(0.1)

    def get_current_rel_target(self):
        return self.current_relative_target

    def wait_for_at_tar(self):
        time.sleep(0.05)
        self.logger.info(f"Robot travsered to target in {-1} seconds. (Tenet-style :))")

        return -1

