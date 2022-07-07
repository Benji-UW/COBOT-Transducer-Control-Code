'''
Unit convention: mm/kg/s/deg
'''
import socket
import time
import pygame
import os
import numpy as np
import logging
from logging.handlers import TimedRotatingFileHandler
from logging import Formatter
from pygame.locals import *
from Controller import Controller
from UR3e import *
from RobotGUI import *
from Pathfinders import *

# TODO delete these if commenting them out makes no difference.
# # Magic constants for the pygame interface
# BLACK = pygame.Color('black')
# WHITE = pygame.Color('white')

date_time_str = time.strftime(r"%Y-%m-%d_%H-%M-%S")
file_itr = 0

path = os.path.dirname(__file__)

while os.path.exists(path + "\\Scans\\test_%s.json" % file_itr):
    file_itr += 1

# Configure the root logger to a particular folder, format, and level. Lower the level when things
# are working better or worse.
root_logger = logging.getLogger()
handler = TimedRotatingFileHandler(filename=f"Rebuilding_Everything\logging\\runtime_test.log",\
    when='D',backupCount=8,encoding="utf-8")
formatter = Formatter(fmt='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
root_logger.addHandler(handler)
root_logger.setLevel(logging.INFO)
logger = logging.getLogger(__name__)
logger.debug("Debug log for the robot starting on " + date_time_str)

np.set_printoptions(formatter={'float': '{: 0.3f}'.format})

def main():
    '''Initialize a transducer_homing object and run that mf'''
    robot = Transducer_homing()
    robot.initialize()

    robot.connect_to_matlab()

    # robot.test_functions()
    robot.start()

class Transducer_homing:
    '''The UR3e robot keeps the position and angle of the TCP in meters/radians, 
    this class mostly stores them in mm/deg'''
    def __init__(self):
        self.robot = None
        self.matlab_socket = None
        self.screen = None
        self.screen_resolution = (480, 940)
        self.controller = None
        self.robot_gui = None
        self.latest_loop = -1

        self.v_list = [0.005, 0.0125, 0.025, 0.05, 0.1, 0.2, 0.4, 0.8]
        self.vR_list = [0.025, 0.05, 0.1, 0.2, 0.4, 0.8, 1.0, 2.0]
        self.a_list = [0.025, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 1.0]
        self.aR_list = [0.1, 0.2, 0.4, 0.6, 0.8, 1, 1.2, 2.0]

        self.speed_presets = [(0.005, 0.025,0.025,0.1),\
                            (0.0125,0.05,0.05,0.2),\
                            (0.025,0.1,0.1,0.4),\
                            (0.05,0.2,0.2,0.6),\
                            (0.1,0.4,0.3,0.8),
                            (0.2,0.8,0.4,1),
                            (0.4,1.0,0.5,1.2),
                            (0.8,2.0,1.0,2.0)]

        self.speed_preset = 3
        # TODO Delete excess self.variables if commenting them out has no impact
        # self.joy_min = 0.01

        self.refresh_rate = 112.
        self.lag = 0.2 #10. / self.refresh_rate #0.1
        # self.true_refresh_rate = None
        # self.button_hold = []
        self.max_disp = 0.5

        # self.motion_comp = 0
        # self.z_adjust = 0
        # self.z_jerk_coeff = 3
        # self.amp_hist = []

        # self.Kp = 10.
        # self.Ki = 0.0
        # self.Kd = 0.1  # 0.01
        # self.servo_lag = self.lag / 2.

        # self.servo_error = 0.0

        # self.t_pred = 0.126
        # self.mp_t_upd = 0.1
        # self.mp_t_lu = 0.

        # self.servo_t = 0.02
        # self.servo_lh_t = 0.035
        # self.servo_gain = 1000

        self.range_of_motion = {'X': 0,'Y': 0,'Z':8,'Rx':30,'Ry':30,'Rz':0}

    def connect_to_matlab(self, server_ip='localhost', port=508):
        '''Connects to the central socket server that coordinates information between
        this module and the MATLAB signal processing info.'''
        self.matlab_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.matlab_socket.connect((server_ip, port))
        except socket.gaierror as e:
            logger.debug(f'Connection error to robot: {e}')
            return (False,e)
        logger.debug('Connected to MATLAB')
        return (True,'')

    def disconnect_from_matlab(self):
        self.matlab_socket.send(b'end')
        self.matlab_socket.close()

    def initialize(self, ip_robot='192.168.0.10'):
        pygame.init()

        self.screen = pygame.display.set_mode(self.screen_resolution)
        pygame.display.set_caption('Transducer homing')

        pygame.joystick.init()
        try:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
        except pygame.error:
            joystick = -1
        self.controller = Controller(joystick)

        self.robot_gui = RobotGUI()

        self.robot = UR3e()
        if not self.robot.connect(ip_robot)[0]:
            return False
        logger.info("robot successfully connected to all ports!")
        
        self.robot.initialize()
        # TODO: delete the excess lists if this works
        # v = self.v_list[self.speed_preset]
        # vR = self.vR_list[self.speed_preset]
        # a = self.a_list[self.speed_preset]
        # aR = self.aR_list[self.speed_preset]

        v,vR,a,aR = self.speed_presets[self.speed_preset]
        
        self.robot.set_parameters(acc=a, velocity=v, acc_rot=aR, vel_rot=vR)
        self.robot.set_max_displacement(self.max_disp)
        self.robot.set_parameters(base='base')
        print("Robot initialized :)")

    def start(self):
        self.robot_gui.reset()
        run_bool = True
        router = False
        translate = True

        self.listener = self.MATLAB_listener()
        # self.listener = self.fake_MATLAB_listener()

        self.last_ten_refresh_rate = np.zeros((10,0))

        self.starting_pos = [np.copy(self.robot.pos), np.copy(self.robot.angle)]
        changed_preset = False
        nextpoint = None

        i_rr = 0

        self.t = time.time()
        (new_mag, latest_mag) = next(self.listener)

        self.i=-1
        print('About to start the main loop')
        go_next = True
        path = os.path.dirname(__file__)

        while run_bool:
            t0 = time.time()
            self.robot.update()
            self.main_menu_GUI(router, nextpoint)
            #self.robot.get_tcp_force()

            (new_mag, latest_mag) = next(self.listener)

            # This section of code is entirely for handling a new signal reading from MATLAB
            if new_mag:
                # Iterate a counter that keeps track of the magnitude readings we've recieved
                self.i+=1
                # Log how much time has elapsed since the previous reading
                self.last_ten_refresh_rate[self.i%10] = time.time() - self.t
                self.t = time.time()
                pos,angle = self.robot.get_current_rel_target()

                if router:
                    self.pathfinder.newMag(((pos,angle), latest_mag), go_next)
                    # You are here                

            pygame.event.pump()
            keys = pygame.key.get_pressed()
            pressed_buttons=self.controller.get_buttons(keys)

            # Press esc to escape the program and close out everything.
            if pressed_buttons["exit"] or keys[pygame.key.key_code("q")] == 1:
                run_bool = False
            if pressed_buttons["speed_preset_up"] or keys[pygame.key.key_code("]")] == 1:
                if self.speed_preset < len(self.v_list) - 1:
                    self.speed_preset += 1
                    changed_preset = True
            elif pressed_buttons["speed_preset_down"] or keys[pygame.key.key_code("[")] == 1:
                if self.speed_preset > 0:
                    self.speed_preset -= 1
                    changed_preset = True
            
            # Press d to trigger the demo pathfinder
            if pressed_buttons["switch_space"]:
                translate = not translate
            # Starts a demo pathfinder if the 'd' key is pressed
            if keys[pygame.key.key_code("d")] == 1 and not router:
                router = True
                self.pathfinder = Pathfinder(20,25,25)
                self.robot.set_initial_pos()
            if keys[pygame.key.key_code("k")] == 1 and not router:
                router = True
                self.pathfinder = FullScan((0.8,1),20,16,16,path=path + f"\\Scans\\test_{file_itr}.json")
                nextpoint = self.pathfinder.next()
            # Press x to stop the running pathfinder
            if keys[pygame.key.key_code("x")] == 1 and router:
                router = False
                path = os.path.dirname(__file__)
                self.pathfinder.save_points(path + f"\\Scans\\test_{file_itr}.json")
                logger.debug(f"triggering movel to {nextpoint}")
                self.robot.movel_to_target(nextpoint)
            if keys[pygame.key.key_code("a")] == 1 and not router:
                self.change_range_gui()
            if keys[pygame.key.key_code("h")] == 1 and not router:
                self.robot.movel(self.starting_pos[0], self.starting_pos[1])

            # changes the speed preset of the robot if that occured.
            if changed_preset:
                # v = self.v_list[self.speed_preset]
                # vR = self.vR_list[self.speed_preset]
                # a = self.a_list[self.speed_preset]
                # aR = self.aR_list[self.speed_preset]
                v,vR,a,aR = self.speed_presets[self.speed_preset]

                self.robot.set_parameters(acc=a, velocity=v, acc_rot=aR, vel_rot=vR)


            if router:
                # print("waiting...")
                v = self.robot.wait_for_at_tar()
                print(f"Made it in {v} loops")
                nextpoint = self.pathfinder.next()
                if nextpoint == 1:
                    router = False
                    path = os.path.dirname(__file__)
                    self.pathfinder.save_points(path + f'\\Scans\\test_{file_itr}.json')
                else:
                    logger.debug(f"triggering movel to {nextpoint}")
                    self.robot.movel_to_target(nextpoint)
                    # literally do not pass go, do not collect $200 until
                    # the target has been reached.

            else:
                speed_vect = np.zeros((3,1))
                rot_vect = np.zeros((3,1))
                joy_vect = self.controller.get_hat(keys)
                if translate:
                    speed_vect = joy_vect
                else:
                    rot_vect[0] = joy_vect[1]
                    rot_vect[1] = joy_vect[0]
                    rot_vect[2] = joy_vect[2]
                self.robot.speedl(speed_vect,rot_vect,self.lag)


            logger.debug("----------------------------------------------")
            logger.debug("Position info about the robot:")
            logger.debug(f"Initial pos/angle: ({self.robot.initial_pos.T}, {self.robot.initial_angle.T})")
            logger.debug(f"Current pos/angle: ({self.robot.pos.T}, {self.robot.angle.T}")
            logger.debug("----------------------------------------------")

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    run_bool = False
            pygame.display.flip()
            if i_rr >= self.refresh_rate:
                i_rr = 0
            t = time.time()
            if 1.0 / self.refresh_rate - (t - t0) > 0:
                time.sleep(1.0 / self.refresh_rate - (t - t0))
        #-----------------------------Bottom of loop-----------------------------------#

        # Return robot to starting position (comment out when you don't wanna do this)
        self.robot.movel(self.starting_pos[0], self.starting_pos[1])

        # loop is exited
        self.robot.disconnect()
        logger.info('Robot disconnected.')
        if self.matlab_socket is not None:
            self.disconnect_from_matlab()
            logger.info('Disconnected from server.')
        if router:
            path = os.path.dirname(__file__)
            self.pathfinder.save_points(path + '\\Scans\\test_' + file_itr + '.json')

    def get_delta_pos(self):
        '''Converts the get_delta_pos method built into the UR3e method into
        the local units used in the pathfinders, mm/kg/s/deg'''
        d_pos,d_angle = self.robot.get_delta_pos()
        
        d_pos = d_pos*1000
        d_angle = np.rad2deg(d_angle)
        
        for i in range(3):
            if (d_angle[i] > 180):
                d_angle[i] -= 360
            elif (d_angle[i] < -180):
                d_angle[i] += 360

        #print(d_pos, d_angle)
        return (d_pos,d_angle)

# TODO: Fully delete interpolate motion method if it causes no problems by deleting.
    # def interpolate_motion(self,next_target):
    #     '''The bug is in here'''
    #     # This data is in mm and deg thanks to internal positioning methods
    #     c_pos,c_angle = self.get_delta_pos()

    #     logger.debug("\n\n-------------------------------------------------")
    #     logger.debug("Entering a new round of motion interpolation")
    #     logger.debug("-------------------------------------------------")

    #     logger.debug("1. Current change in position from the origin: ")
    #     logger.debug(f"\n{c_pos}, {c_angle}")
        
    #     # This data is also in mm and deg due to convention
    #     t_pos,t_angle = (next_target[0],next_target[1])
    #     t_pos,t_angle = np.array([t_pos]).T,np.array([t_angle]).T

    #     logger.debug("2. Current target position relative to the origin: \n (Should be the same shape)")
    #     logger.debug(f"\n{t_pos}, {t_angle}")

    #     #print('target pos:')
    #     #print(t_pos, t_angle)
    #     delta_pos = t_pos - c_pos
        
    #     logger.debug("3. Change in position required to get from the current to the target position")
    #     logger.debug(f"{delta_pos}")

    #     if (np.max(np.abs(delta_pos)) != 0):
    #         speed_vect = delta_pos / max(np.max(np.abs(delta_pos)),5)
    #         logger.debug("4. Calculated speed-vector to reach that delta_pos in a decent amount of time:")
    #         logger.debug(f"{speed_vect}")
    #     else:
    #         speed_vect = delta_pos

    #     delta_ang = t_angle - c_angle

    #     logger.debug("5. Change in angle required to get from the current to the target position")
    #     logger.debug(delta_ang.T)

    #     rot_vect = np.zeros((3,1))
        
    #     if (np.max(np.abs(delta_ang)) != 0):
    #         delta_ang = delta_ang / np.max(np.abs(delta_ang))
    #         rot_vect[0] = 1 * delta_ang[0] # 1 - Rx
    #         rot_vect[1] = -1 * delta_ang[1] 
    #         # rot_vect[2] = -0 * delta_ang[1]# 0 - Rz
    #         rot_vect[2] = 0
    #         logger.debug("6. Calculated angular-speed to reach that delta_pos in a decent amount of time:")
    #         logger.debug(f"{rot_vect}")
        
    #     # logger.debug("7. Translation vector about to be sent to the robot for execution: (trans, rot)")
    #     # logger.debug(f"{speed_vect}, {rot_vect}")

    #     return speed_vect,rot_vect

    def MATLAB_listener(self):
        '''This method creates a generator for listening to the server for new data from
        MATLAB. At each yield statement it returns a tuple containing a boolean and a float,
        the boolean indicating whether the magnitude is new and the float representing the
        magnitude.'''   
        mag,self.latest_loop = -1,-1        
        while True:
            self.matlab_socket.send(b'motion')
            msg = self.matlab_socket.recv(1024)

            mag = float(msg[4:13]) / 1.0E3
            loop = int(msg[1:3])

            if loop == self.latest_loop:
                yield (False, mag)
            else:
                self.latest_loop = loop
                yield (True, mag)
    
    def fake_MATLAB_listener(self):
        '''This method fakes the input from the MATLAB listener, can be used to debug the
        motion of the robotic arm on occasions when we aren't using the transducer itself yet.'''
        while True:
            mag = round(time.time()) % 59
            yield (True, mag)

#TODO: Delete following method if code can be run with it commented out
    # def MATLAB_next(self):
    #     '''Hopefully can be deprecated once the generator is used successfully.'''
    #     self.matlab_socket.send(b'motion')
    #     msg = self.matlab_socket.recv(1024)
    #     print(msg)
    #     mag = float(msg[4:13]) * 1.0E3
    #     loop = int(msg[1:3])
    #     if loop == self.latest_loop:
    #         return (False, mag)
    #     else:
    #         self.latest_loop = loop
    #         return (True, mag)

    def main_menu_GUI(self,router_bool,current_target):
        self.screen.fill(WHITE)
        self.robot_gui.reset()
        self.write_pos_info(router_bool,current_target)

    def change_range_gui(self):
        '''It is not a priority right now but I would ultimately like there to be
        a GUI option for adjusting the default range of motion for the pathfinder.
        By default the range is +/- 8 mm along the z axis and +/- 45 degrees along
        the Rx and Ry axes.'''
        pass
        # not_set = True
        # cursor = 0
        # while not_set:
        #     self.screen.fill(WHITE)

        #     pygame.event.pump()
        #     keys = pygame.key.get_pressed()

    def write_pos_info(self, router, current_target):
        pos = self.robot.pos
        angle = self.robot.angle
        #TODO: Delete commented things if I don't wind up using them :)
        # tcp_offset = self.robot.tcp_offset
        # tcp_angle = self.robot.tcp_rot
        delta_pos,delta_angle = self.get_delta_pos()

        # v = self.v_list[self.speed_preset]
        # vR = self.vR_list[self.speed_preset]
        # a = self.a_list[self.speed_preset]
        # aR = self.aR_list[self.speed_preset]

        # v,vR,a,aR = self.speed_presets[self.speed_preset]

        self.robot_gui.tprint(self.screen, 'Current base: %s' % self.robot.base)

        self.robot_gui.tprint(self.screen, 'TCP position in relation to its initial position:')
        self.robot_gui.indent()
        self.robot_gui.tprint(self.screen, 'x= %4.1f mm, Rx= %3.0f deg' %
                              (delta_pos[0], delta_angle[0]))
        self.robot_gui.tprint(self.screen, 'y= %4.1f mm, Ry= %3.0f deg' %
                              (delta_pos[1], delta_angle[2]))
        self.robot_gui.tprint(self.screen, 'z= %4.1f mm, Rz= %3.0f deg' %
                              (delta_pos[2], delta_angle[1]))
        self.robot_gui.unindent()

        self.robot_gui.skip_line(1)

        # self.robot_gui.tprint(self.screen, 'TCP position in base:')
        self.robot_gui.tprint(self.screen, "TCP position in base: ((%4.1f, %4.1f, %4.1f), (%3.0f,%3.0f,%3.0f))" % 
                            (pos[0]*1000,pos[1]*1000,pos[2]*1000,np.rad2deg(angle[0]),np.rad2deg(angle[1]),np.rad2deg(angle[2])))
        # self.robot_gui.indent()
        # self.robot_gui.tprint(self.screen, 'x= %4.1f mm, Rx= %3.0f deg' %
        #                       (pos[0]*1000, np.rad2deg(angle[0])))
        # self.robot_gui.tprint(self.screen, 'y= %4.1f mm, Ry= %3.0f deg' %
        #                       (pos[1]*1000, np.rad2deg(angle[1])))
        # self.robot_gui.tprint(self.screen, 'z= %4.1f mm, Rz= %3.0f deg' %
        #                       (pos[2]*1000, np.rad2deg(angle[2])))
        # self.robot_gui.unindent()

        if current_target is not None and current_target != 1:
            self.robot_gui.skip_line(1)
            t_pos = current_target[0]
            t_ang = current_target[1]
            self.robot_gui.tprint(self.screen, "Next target: ((%4.2f, %4.2f, %4.2f), (%3.1f,%3.1f,%3.1f))" % 
                                (t_pos[0],t_pos[1],t_pos[2],t_ang[0],t_ang[1],t_ang[2]))


        self.robot_gui.skip_line(1)

        self.robot_gui.tprint(self.screen, f'Speed preset: {self.speed_preset}')

        self.robot_gui.skip_line(1)
        self.robot_gui.tprint(self.screen, f'Recent refresh rate: {np.mean(self.last_ten_refresh_rate)}')

        self.robot_gui.skip_line(3)
        
        if not router:
            self.robot_gui.tprint(self.screen, 'Press (d)emo to demonstrate the basic pathrouting module')
            self.robot_gui.tprint(self.screen, 'Press (k) to trigger a full scan with hard-coded resolution.')
            self.robot_gui.indent()
            self.robot_gui.tprint(self.screen, 'Beware this will override the controller until the pathfinder is cancelled or has finished the task.')
            self.robot_gui.unindent()
        else:
            # next_target = self.pathfinder.next()
            t_pos,t_angle = (current_target[0],current_target[1])
            self.robot_gui.tprint(self.screen, 'Press (x) to cancel the running pathfinder')
            self.robot_gui.tprint(self.screen, "Press (m) to movel to the next target point.")
            self.robot_gui.skip_line(2)
            self.robot_gui.tprint(self.screen, 'Current target:')
            self.robot_gui.indent()
            self.robot_gui.tprint(self.screen, 'x= %4.2f mm, Rx= %3.1f deg' %
                                (t_pos[0], t_angle[0]))
            self.robot_gui.tprint(self.screen, 'y= %4.2f mm, Ry= %3.1f deg' %
                                (t_pos[1], t_angle[1]))
            self.robot_gui.tprint(self.screen, 'z= %4.2f mm, Rz= %3.1f deg' %
                                (t_pos[2], t_angle[2]))
            self.robot_gui.unindent()
            self.robot_gui.tprint(self.screen, 'Progress of the current running pathfinder:')
            self.robot_gui.indent()
            for i in self.pathfinder.progress_report():
                self.robot_gui.tprint(self.screen, i)
            self.robot_gui.unindent()

        
        self.robot_gui.skip_line(2)
        self.robot_gui.tprint(self.screen, time.ctime())


if __name__=="__main__":
  main()
else:
  print("run from import")