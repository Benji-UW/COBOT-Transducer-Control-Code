import socket
import time
import pygame
import math
import os
import numpy as np
from pygame.locals import *
from Controller import Controller
from UR3e import *
from RobotGUI import *
from MotionPrediction import *
from datetime import datetime

BLACK = pygame.Color('black')
WHITE = pygame.Color('white')


class RobotTreatment:
    def __init__(self):
        self.robot = None
        # adding another comment for Git purposes
        self.matlab_socket = None
        self.screen = None
        self.screen_resolution = (480, 940)
        self.controller = None
        self.robot_gui = None

        self.v_list = [0.005, 0.0125, 0.025, 0.05, 0.1, 0.2, 0.4, 0.8]
        self.vR_list = [0.025, 0.05, 0.1, 0.2, 0.4, 0.8, 1.0, 2.0]
        self.a_list = [0.025, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 1.0]
        self.aR_list = [0.1, 0.2, 0.4, 0.6, 0.8, 1, 1.2, 2.0]
        self.speed_preset = 2
        self.joy_min = 0.01

        self.refresh_rate = 112.
        self.lag = 0.1 #10. / self.refresh_rate #0.1
        self.true_refresh_rate = None
        self.button_hold = []
        self.max_disp = 0.02

        self.waypoints = []
        self.current_waypoint = 0

        self.motion_comp = 0
        self.z_adjust = 0

        self.Kp = 10.
        self.Ki = 0.0
        self.Kd = 0.1  # 0.01
        self.servo_lag = self.lag / 2.

        self.servo_error = 0.0

        self.mp_x = MotionPrediction()
        self.mp_z = MotionPrediction()
        self.t_pred = 0.126
        self.mp_t_upd = 0.1
        self.mp_t_lu = 0.

        self.servo_t = 0.02
        self.servo_lh_t = 0.035
        self.servo_gain = 1000

        self.imager_x_vect = np.array([])
        self.x_vect = np.array([])
        self.mp_x_vect = np.array([])
        self.imager_z_vect = np.array([])
        self.z_vect = np.array([])
        self.mp_z_vect = np.array([])
        self.imager_t_vect = np.array([])

    def connect_to_matlab(self, server_ip='localhost', port=50008):
        self.matlab_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.matlab_socket.connect((server_ip, port))
        except socket.gaierror, e:
            print('Connection error to robot: %s' % e)
            return False, e
        return True, ''

    def disconnect_from_matlab(self):
        self.matlab_socket.send('end')
        self.matlab_socket.close()

    def initialize(self, ip_robot='192.168.0.10'):
        pygame.init()

        self.screen = pygame.display.set_mode(self.screen_resolution)
        pygame.display.set_caption('Robot Treatment v0.14')

        pygame.joystick.init()
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        self.controller = Controller(joystick)

        self.robot_gui = RobotGUI()

        self.robot = UR3e()
        if not self.robot.connect(ip_robot)[0]:
            return False
        self.robot.initialize()
        v = self.v_list[self.speed_preset]
        vR = self.vR_list[self.speed_preset]
        a = self.a_list[self.speed_preset]
        aR = self.aR_list[self.speed_preset]
        self.robot.set_parameters(acc=a, velocity=v, acc_rot=aR, vel_rot=vR)
        self.robot.set_max_displacement(self.max_disp)

        if not os.path.isdir('./motion'):
            os.mkdir('./motion')

    def start(self):
        run_bool = True
        setpoint = np.zeros((3, 1))
        self.true_refresh_rate = np.zeros((int(self.refresh_rate),))
        i_rr = 0
        t_nm1 = time.time()
        t_0 = time.time()
        pos_ref = 0.
        n_sample = 2000
        imager_t_nm1 = 0.

        filepath = r'C:\Users\ander\OneDrive - UW\Robotics lab material\Robotics Control Code\Ben\'s Control Code\Records\\' + time.strftime('%c') + '.txt'

        file1 = open(filepath, "w+")

        # Z-motion comp stuff
        self.z_adjust = 0

        while run_bool:
            t0 = time.time()

            self.robot.update()

            self.screen.fill(WHITE)
            self.robot_gui.reset()
            self._write_pos_info()
            if self.imager_t_vect.size > 4:
                self.robot_gui.plot_graph(self.screen, (self.screen_resolution[0], 380), self.imager_t_vect,
                                          self.imager_x_vect, hold=True)
                self.robot_gui.plot_graph(self.screen, (self.screen_resolution[0], 380), self.imager_t_vect,
                                          self.x_vect, colour=pygame.Color(205, 25, 25), hold=True)
                self.robot_gui.plot_graph(self.screen, (self.screen_resolution[0], 380), self.imager_t_vect,
                                          self.imager_z_vect, colour=pygame.Color(25, 205, 25), hold=True)
                self.robot_gui.plot_graph(self.screen, (self.screen_resolution[0], 380), self.imager_t_vect,
                                          self.z_vect, colour=pygame.Color(205, 25, 205), hold=False)
            
            pygame.event.pump()
            keys = pygame.key.get_pressed()
            buttons = self.controller.get_buttons()
            joy_vect = self.controller.get_sticks()
            joy_hat = self.controller.get_hat()

            self.motion_comp = 0


            if self.z_adjust == 1:
                self.matlab_socket.send('motion')
                msg = self.matlab_socket.recv(1024)

                # print('recieved msg ' + msg)
                latestLoop = float(msg[1:3])
                dist_z = float(msg[4:13]) / 1.0E3
                
                if np.abs(dist_z) < 0.05:
                    self.z_adjust = 0
                    file1.write(time.asctime() + ' | Exit z_adjustment with z_dist ' + str(self.z_adjust))
                    return
                # dist_z is the vertical distance we need to travel in mm
                t_1 = time.time()
                if latestLoop != lastLoop:
                    t_0 = time.time()
                    z_tran = np.minimum((dist_z / 7.5), 1)
                    speed_vect[2] = z_tran
                    # print('set speed to ' + str(z_tran))
                    file1.write(time.asctime() + ' | Received message ' + str(msg) + ' set speed to ' + str(z_tran) + '\n')
                elif t_1 - t_0 <= 0.5:
                    speed_vect = np.zeros((3,1))
                    rot_vect = np.zeros((3,1))
                    speed_vect[2] = z_tran

                    self.robot.speedl(speed_vect, rot_vect, self.lag)
                    
                    file1.write(time.asctime() + ' | Received message ' + str(msg) + ' triggered speed' + str(z_tran) + '\n')

                else:
                    speed_vect = np.zeros((3,1))
                    rot_vect = np.zeros((3,1))
                    # print('took a break from moving')
                    file1.write(time.asctime() + ' | Received message ' + str(msg) + ' took a break from moving' + '\n')


                # if latestLoop != lastLoop:
                #     self.robot.speedl(speed_vect, rot_vect, self.lag)
                
                lastLoop = latestLoop

            # Formerly you would toggle motion control with this button, but I don't have any
            # of Gilles's motion control scripts so I'm just getting rid of all of that.
            if self.controller.P in buttons and self.controller.P not in self.button_hold:
                print('Hey you just pressed the plus button :)')
                self.button_hold.append(self.controller.P)
            elif self.controller.P in self.button_hold and self.controller.P not in buttons:
                self.button_hold.remove(self.controller.P)

            # Test stuff with X
            if self.controller.X in buttons and self.controller.X not in self.button_hold:
                self.z_adjust = 1

                self.speed_preset == 0
                v = self.v_list[self.speed_preset]
                vR = self.vR_list[self.speed_preset]
                a = self.a_list[self.speed_preset]
                aR = self.aR_list[self.speed_preset]

                lastLoop = 0

                self.robot.set_parameters(acc=a, velocity=v, acc_rot=aR, vel_rot=vR)

                #self.button_hold.append(self.controller.X)
            elif self.controller.X in self.button_hold and self.controller.X not in buttons:
                self.button_hold.remove(self.controller.X)

            if self.motion_comp == 0:
                # Add a waypoint at current position with button A
                if self.controller.A in buttons and self.controller.A not in self.button_hold:
                    self.waypoints.append([np.copy(self.robot.pos), np.copy(self.robot.angle)])
                    self.button_hold.append(self.controller.A)
                elif self.controller.A in self.button_hold and self.controller.A not in buttons:
                    self.button_hold.remove(self.controller.A)

                # Remove last waypoint with button B
                if self.controller.B in buttons and self.controller.B not in self.button_hold:
                    if len(self.waypoints) > 0:
                        self.waypoints.pop()
                        if self.current_waypoint > len(self.waypoints):
                            self.current_waypoint -= 1
                    self.button_hold.append(self.controller.B)
                elif self.controller.B in self.button_hold and self.controller.B not in buttons:
                    self.button_hold.remove(self.controller.B)

                # Go to first waypoint with button Y
                if self.controller.Y in buttons and self.controller.Y not in self.button_hold:
                    if len(self.waypoints) > 0:
                        self.robot.movel(self.waypoints[0][0], self.waypoints[0][1])
                        self.current_waypoint = 0
                    self.button_hold.append(self.controller.Y)
                elif self.controller.Y in self.button_hold and self.controller.Y not in buttons:
                    self.button_hold.remove(self.controller.Y)

                # Go to next waypoint with button R
                if self.controller.R in buttons and self.controller.R not in self.button_hold:
                    if 0 <= self.current_waypoint < len(self.waypoints) - 1:
                        self.current_waypoint += 1
                        self.robot.movel(self.waypoints[self.current_waypoint][0], self.waypoints[self.current_waypoint][1])
                    self.button_hold.append(self.controller.R)
                elif self.controller.R in self.button_hold and self.controller.R not in buttons:
                    self.button_hold.remove(self.controller.R)

                # Go to previous waypoint with button L
                if self.controller.L in buttons and self.controller.L not in self.button_hold:
                    if self.current_waypoint > 0:
                        self.current_waypoint -= 1
                        self.robot.movel(self.waypoints[self.current_waypoint][0], self.waypoints[self.current_waypoint][1])
                        self.button_hold.append(self.controller.L)
                elif self.controller.L in self.button_hold and self.controller.L not in buttons:
                    self.button_hold.remove(self.controller.L)

                # Change base with L3 stick button
                if self.controller.L3 in buttons and self.controller.L3 not in self.button_hold:
                    if self.robot.base == 'base':
                        self.robot.set_parameters(base='tcp')
                    elif self.robot.base == 'tcp':
                        self.robot.set_parameters(base='base')
                    self.button_hold.append(self.controller.L3)
                elif self.controller.L3 in self.button_hold and self.controller.L3 not in buttons:
                    self.button_hold.remove(self.controller.L3)

                # Reset initial position with minus button
                if self.controller.M in buttons and self.controller.M not in self.button_hold:
                    self.robot.set_initial_pos()
                    self.button_hold.append(self.controller.M)
                elif self.controller.M in self.button_hold and self.controller.M not in buttons:
                    self.button_hold.remove(self.controller.M)

                # This looks like it's activating freedrive as long as the button is pushed :)
                if self.controller.ZL in buttons and self.controller.ZL not in self.button_hold:
                    self.robot.freedrive()
                    self.button_hold.append(self.controller.ZL)
                elif self.controller.ZL in self.button_hold and self.controller.ZL not in buttons:
                    self.robot.end_freedrive()
                    self.button_hold.remove(self.controller.ZL)

                # Change speed preset value
                if (self.controller.get_hat()[0] > 0) and (self.controller.PAD not in self.button_hold):
                    if self.speed_preset < len(self.v_list) - 1:
                        self.speed_preset += 1
                        v = self.v_list[self.speed_preset]
                        vR = self.vR_list[self.speed_preset]
                        a = self.a_list[self.speed_preset]
                        aR = self.aR_list[self.speed_preset]
                        self.robot.set_parameters(acc=a, velocity=v, acc_rot=aR, vel_rot=vR)
                    self.button_hold.append(self.controller.PAD)
                elif (joy_hat[0] < 0) and (self.controller.PAD not in self.button_hold):
                    if self.speed_preset > 0:
                        self.speed_preset -= 1
                        v = self.v_list[self.speed_preset]
                        vR = self.vR_list[self.speed_preset]
                        a = self.a_list[self.speed_preset]
                        aR = self.aR_list[self.speed_preset]
                        self.robot.set_parameters(acc=a, velocity=v, acc_rot=aR, vel_rot=vR)
                    self.button_hold.append(self.controller.PAD)
                elif (joy_hat[1] < 0) and (self.controller.PAD not in self.button_hold):
                    if self.max_disp > 0.01:
                        self.max_disp -= 0.01
                        self.robot.set_max_displacement(self.max_disp)
                    self.button_hold.append(self.controller.PAD)
                elif (joy_hat[1] > 0) and (self.controller.PAD not in self.button_hold):
                    if self.max_disp < 0.5:
                        self.max_disp += 0.01
                        self.robot.set_max_displacement(self.max_disp)
                    self.button_hold.append(self.controller.PAD)
                elif joy_hat[0] == 0 and joy_hat[1] == 0 and \
                        self.controller.PAD in self.button_hold:
                    self.button_hold.remove(self.controller.PAD)

                # Move/rotate with the joysticks
                if self.controller.ZR in buttons:
                    if np.linalg.norm(joy_vect) > self.joy_min:
                        rot_vect = np.zeros((3, 1))
                        rot_vect[0] = joy_vect[1]
                        rot_vect[1] = joy_vect[0]
                        rot_vect[2] = joy_vect[2]
                        speed_vect = np.zeros((3, 1))
                        self.robot.speedl(speed_vect, rot_vect, self.lag)
                elif np.linalg.norm(joy_vect) > self.joy_min:
                    speed_vect = np.copy(joy_vect[[0, 1, 3]])
                    speed_vect[0] = -speed_vect[0]
                    rot_vect = np.zeros((3, 1))
                    self.robot.speedl(speed_vect, rot_vect, self.lag)

            if keys[K_q]:
                if self.Kp > 0:
                    self.Kp -= 0.1
                self.robot.set_servo_parameters(Kp=self.Kp)
            elif keys[K_w]:
                self.Kp += 0.1
                self.robot.set_servo_parameters(Kp=self.Kp)
            if keys[K_a]:
                if self.servo_lag > 0.002:
                    self.servo_lag -= 0.001
                self.robot.set_servo_parameters(lag=self.servo_lag)
            elif keys[K_s]:
                self.servo_lag += 0.001
                self.robot.set_servo_parameters(lag=self.servo_lag)
            if keys[K_z]:
                if self.Kd > 0:
                    self.Kd -= 0.001
                self.robot.set_servo_parameters(Kd=self.Kd)
            elif keys[K_x]:
                self.Kd += 0.001
                self.robot.set_servo_parameters(Kd=self.Kd)
            if keys[K_e]:
                self.t_pred -= 0.001
            elif keys[K_r]:
                self.t_pred += 0.001

            if keys[K_ESCAPE]:
                self.robot.disconnect()
                if self.matlab_socket is not None:
                    self.disconnect_from_matlab()
                run_bool = False
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.robot.disconnect()
                    print 'Robot disconnected.'
                    run_bool = False
                    if self.matlab_socket is not None:
                        self.disconnect_from_matlab()
                        print 'Disconnected from server.'

            pygame.display.flip()
            if i_rr >= self.refresh_rate:
                i_rr = 0
            t = time.time()
            if 1.0 / self.refresh_rate - (t - t0) > 0:
                time.sleep(1.0 / self.refresh_rate - (t - t0))
            self.true_refresh_rate[i_rr] = time.time() - t_nm1
            t_nm1 = time.time()
            i_rr += 1

    def _write_pos_info(self):
        pos = self.robot.pos
        angle = self.robot.angle
        tcp_offset = self.robot.tcp_offset
        tcp_angle = self.robot.tcp_rot
        delta_pos, delta_angle = self.robot.get_delta_pos()
        v = self.v_list[self.speed_preset]
        vR = self.vR_list[self.speed_preset]
        a = self.a_list[self.speed_preset]
        aR = self.aR_list[self.speed_preset]

        self.robot_gui.tprint(self.screen, 'TCP offset:')
        self.robot_gui.indent()
        self.robot_gui.tprint(self.screen, 'x= %4.1f mm, Rx= %3.0f deg' %
                              (tcp_offset[0] * 1000., tcp_angle[0] * 180 / math.pi))
        self.robot_gui.tprint(self.screen, 'y= %4.1f mm, Ry= %3.0f deg' %
                              (tcp_offset[1] * 1000., tcp_angle[1] * 180 / math.pi))
        self.robot_gui.tprint(self.screen, 'z= %4.1f mm, Rz= %3.0f deg' %
                              (tcp_offset[2] * 1000., tcp_angle[2] * 180 / math.pi))
        self.robot_gui.unindent()

        self.robot_gui.tprint(self.screen, 'Current TCP position in base:')
        self.robot_gui.indent()
        self.robot_gui.tprint(self.screen, 'x= %4.1f mm, Rx= %3.0f deg' %
                              (pos[0] * 1000., angle[0] * 180 / math.pi))
        self.robot_gui.tprint(self.screen, 'y= %4.1f mm, Ry= %3.0f deg' %
                              (pos[1] * 1000., angle[1] * 180 / math.pi))
        self.robot_gui.tprint(self.screen, 'z= %4.1f mm, Rz= %3.0f deg' %
                              (pos[2] * 1000., angle[2] * 180 / math.pi))
        self.robot_gui.unindent()

        self.robot_gui.tprint(self.screen, 'TCP position in relation to its initial position:')
        self.robot_gui.indent()
        self.robot_gui.tprint(self.screen, 'x= %4.1f mm, Rx= %3.0f deg' %
                              (delta_pos[0] * 1000., delta_angle[0] * 180 / math.pi))
        self.robot_gui.tprint(self.screen, 'y= %4.1f mm, Ry= %3.0f deg' %
                              (delta_pos[1] * 1000., delta_angle[1] * 180 / math.pi))
        self.robot_gui.tprint(self.screen, 'z= %4.1f mm, Rz= %3.0f deg' %
                              (delta_pos[2] * 1000., delta_angle[2] * 180 / math.pi))
        self.robot_gui.unindent()

        self.robot_gui.tprint(self.screen, 'Current base: %s' % self.robot.base)

        self.robot_gui.tprint(self.screen, 'Speed preset [0-%d]: %d' % (len(self.v_list) - 1, self.speed_preset))
        self.robot_gui.indent()
        self.robot_gui.tprint(self.screen, 'v= %1.3f m/s, a= %1.3f m/s2' % (v, a))
        self.robot_gui.tprint(self.screen, 'vR= %1.3f rad/s, aR= %1.3f rad/s2' % (vR, aR))
        self.robot_gui.unindent()

        self.robot_gui.tprint(self.screen, 'Max total TCP displacement allowed: %3.0f mm' % (self.max_disp * 1000.))

        rr = np.mean(self.true_refresh_rate)
        if rr > 0.:
            self.robot_gui.tprint(self.screen, 'Current robot mean refresh rate is: %3.1f Hz' % (1. / rr))
        else:
            self.robot_gui.tprint(self.screen, 'Current robot mean refresh rate is: 0.0 Hz')


        if self.z_adjust:
            self.robot_gui.tprint(self.screen, 'Transducer adjustment is on')

        self.robot_gui.tprint(self.screen, 'Waypoints (current: %d): ' % self.current_waypoint)
        self.robot_gui.indent()
        i = 0
        for w in self.waypoints:
            self.robot_gui.tprint(self.screen, '%d: [%4.0f, %4.0f, %4.0f, %3.0f, %3.0f, %3.0f]' %
                                  (i, w[0][0] * 1000, w[0][1] * 1000, w[0][2] * 1000, w[1][0] * 180. / math.pi,
                                   w[1][1] * 180. / math.pi, w[1][2] * 180. / math.pi))
            i = i + 1
        self.robot_gui.unindent()

# ;C:\Users\ander\AppData\Local\GitHubDesktop\app-2.9.0\resources\app\git\cmd