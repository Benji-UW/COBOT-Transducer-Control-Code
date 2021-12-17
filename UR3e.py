"""
Unit convention: m/kg/s

"""

import numpy as np
import socket
import time
from math import *


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
        self.pos = np.zeros((3, 1))
        
        self.angle = np.zeros((3, 1))
        self.tcp_offset = np.zeros((3, 1))
        self.tcp_rot = np.zeros((3, 1))

        self.max_disp = 0.1
        self.initial_pos = np.zeros((3, 1))
        self.initial_angle = np.zeros((3, 1))
        self.max_disp_delta = 0.002

        self.acc = 0.1
        self.velocity = 0.025
        self.acc_rot = 0.4
        self.vel_rot = 0.05

        self.robot_socket = None
        self.modbus_socket = None

        self.base = 'tcp'

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

    def connect(self, robot_ip, robot_port=30002, modbus_port=502):
        """
        Connect to the robot.

        :param str robot_ip: IP address of the robot
        :param int robot_port: port of the robot
        :param int modbus_port: modbus port of the robot
        :return:
        """
        self.robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.robot_socket.connect((robot_ip, robot_port))
        except socket.gaierror as e:
            print('Connection error to robot: %s' % e)
            return (False, e)

        self.modbus_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.modbus_socket.connect((robot_ip, modbus_port))
        except socket.gaierror as e:
            print('Connection error to modbus: %s' % e)
            return (False, e)

        return (True, '')

    def disconnect(self):
        self.robot_socket.close()

    def set_parameters(self, acc=None, velocity=None, acc_rot=None, vel_rot=None, base=None):
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

    def movel(self, pos_to, angle_to, t=0.0):
        if self._check_move_displacement(pos_to):
            cmd = b'movel(p[%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f],a=%1.2f,v=%1.2f,t=%2.2f,r=0)' % \
                  (pos_to[0], pos_to[1], pos_to[2], angle_to[0], angle_to[1], angle_to[2], self.acc, self.velocity, t)
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

    def speedl(self, speed_vect, rotation_vect, lag):
        if self._check_speed_displacement(speed_vect):
            speed_vect = self.velocity * speed_vect
            rotation_vect = self.vel_rot * rotation_vect
            speed = np.linalg.norm(speed_vect)
            if speed > self.velocity:
                speed_vect = (self.velocity / speed) * speed_vect
            speed = np.linalg.norm(rotation_vect)
            if speed > self.vel_rot:
                rotation_vect = (self.vel_rot / speed) * rotation_vect
            speed_vect = self._change_base(speed_vect)
            rotation_vect = self._change_base(rotation_vect)
            cmd = b'speedl([%1.3f,%1.3f,%1.3f,%2.3f,%2.3f,%2.3f],%1.3f,%2.3f,%1.3f)' % \
                  (speed_vect[0], speed_vect[1], speed_vect[2], rotation_vect[0], rotation_vect[1], rotation_vect[2],
                   self.acc, lag, self.acc_rot)
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
        cmd = b'servoj(get_inverse_kin(p[%1.4f,%1.4f,%1.4f,%2.4f,%2.4f,%2.4f]),0,0,%2.3f,%2.3f,%d)' % \
              (pos[0], pos[1], pos[2], angle[0], angle[1], angle[2], t, lh_t, gain)
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
            self.servo_error_int = self.servo_error_int + 0.5 * (self.servo_error - self.servo_error_nm1) * dt

        servo_prop = self.servo_Kp * self.servo_error
        servo_int = self.servo_Ki * self.servo_error_int
        if dt > 0.001:
            servo_dev = self.servo_Kd * (self.servo_error - self.servo_error_nm1) / dt
        else:
            servo_dev = 0.0
        servo = servo_prop + servo_int + servo_dev
        if self._check_speed_displacement(-servo):
            cmd = b'speedl([%1.5f,%1.5f,%1.5f,0,0,0],%1.3f,%2.3f,%1.3f)' % \
                  (-servo[0], -servo[1], -servo[2], 2.0, self.servo_lag, 2.0)
            self.robot_socket.send(cmd + b'\n')
        else:
            self.stopl()

        self.servo_t_nm1 = self.servo_t
        self.servo_error_nm1 = self.servo_error

    def stopl(self, acc=1):
        self.robot_socket.send(b'stopl(%2.3f)\n' % acc)
        return True

    def freedrive(self):
        self.robot_socket.send(b'freedrive_mode()\n')
        return True

    def end_freedrive(self):
        self.robot_socket.send(b'end_freedrive_mode()\n')
        self._get_pos()
        self.initial_pos = np.copy(self.pos)
        self.initial_angle = np.copy(self.angle)
        return True

    def update(self):
        self._get_pos()

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
        return self._change_base(self.pos - self.initial_pos, inv=True), \
               self._change_base(self.angle - self.initial_angle, inv=True)

    def _get_pos(self):
        # Get registry 400 to 405 in modbus, ie 0x190 w/ read size of 6
        cmd = b'\x00\x04\x00\x00\x00\x06\x00\x03\x01\x90\x00\x06'
        self.modbus_socket.send(cmd)
        reg = self.modbus_socket.recv(1024)
        
        # Get x, registry 400 in modbus, ie 0x190
        #self.pos[0] = _reg_convert(reg[-12:-10].encode('hex'), 'pos')
        self.pos[0] = _reg_convert(reg[-12:-10].hex(), 'pos')

        # Get y, registry 401 in modbus, ie 0x191
        #self.pos[1] = _reg_convert(reg[-10:-8].encode('hex'), 'pos')
        self.pos[1] = _reg_convert(reg[-10:-8].hex(), 'pos')

        # Get z, registry 402 in modbus, ie 0x192
        #self.pos[2] = _reg_convert(reg[-8:-6].encode('hex'), 'pos')
        self.pos[2] = _reg_convert(reg[-8:-6].hex(), 'pos')

        # Get Rx, registry 403 in modbus, ie 0x193
        # self.angle[0] = _reg_convert(reg[-6:-4].encode('hex'), 'angle')
        self.angle[0] = _reg_convert(reg[-6:-4].hex(), 'angle')

        # Get Ry, registry 404 in modbus, ie 0x194
        #self.angle[1] = _reg_convert(reg[-4:-2].encode('hex'), 'angle')
        self.angle[1] = _reg_convert(reg[-4:-2].hex(), 'angle')

        # Get Rz, registry 405 in modbus, ie 0x195
        #self.angle[2] = _reg_convert(reg[-2:].encode('hex'), 'angle')
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

    def _check_move_displacement(self, pos_to):
        disp = np.linalg.norm(np.subtract(pos_to, self.initial_pos))
        if disp >= self.max_disp:
            return False
        else:
            return True

    def _check_speed_displacement(self, speed_vect):
        tcp_direction = np.subtract(self.pos, self.initial_pos)
        disp = np.linalg.norm(tcp_direction)
        tcp_direction = tcp_direction / disp
        speed_direction = speed_vect / np.linalg.norm(speed_vect)

        if disp < self.max_disp - self.max_disp_delta:
            return True
        return False
