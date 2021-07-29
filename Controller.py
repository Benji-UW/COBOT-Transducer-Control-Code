import numpy as np


class Controller:
    def __init__(self, joystick):
        self.joystick = joystick
        self.name = self.joystick.get_name()
        self.joy_min = 0.1
        if 'HORI' in self.name:
            self.A = 0
            self.B = 1
            self.X = 2
            self.Y = 3
            self.L = 4
            self.R = 5
            self.M = 6
            self.P = 7
            self.L3 = 8
            self.R3 = 9
            self.ZL = 10
            self.ZR = 11
            self.PAD = 12
            self.max_buttons = 10
        elif 'Game' in self.name or 'Pro' in self.name:
            self.B = 0
            self.A = 1
            self.Y = 2
            self.X = 3
            self.L = 4
            self.R = 5
            self.ZL = 6
            self.ZR = 7
            self.M = 8
            self.P = 9
            self.L3 = 10
            self.R3 = 11
            self.H = 12
            self.S = 13
            self.PAD = 14
            self.max_buttons = 14

    def get_buttons(self):
        buttons = []
        if 'HORI' in self.name:
            for i in range(0, self.max_buttons):
                if self.joystick.get_button(i):
                    buttons.append(i)
            if self.joystick.get_axis(2) > 0.1:
                buttons.append(self.ZL)
            elif self.joystick.get_axis(2) < -0.1:
                buttons.append(self.ZR)
        elif 'Game' in self.name or 'Pro' in self.name:
            for i in range(0, self.max_buttons):
                if self.joystick.get_button(i):
                    buttons.append(i)
        return buttons

    def get_sticks(self):
        joy_vect = np.zeros((4, 1))
        if 'HORI' in self.name:
            if abs(self.joystick.get_axis(0)) > self.joy_min:
                joy_vect[0] = self.joystick.get_axis(0)
            if abs(self.joystick.get_axis(1)) > self.joy_min:
                joy_vect[1] = -self.joystick.get_axis(1)
            if abs(self.joystick.get_axis(4)) > self.joy_min:
                joy_vect[2] = self.joystick.get_axis(4)
            if abs(self.joystick.get_axis(3)) > self.joy_min:
                joy_vect[3] = -self.joystick.get_axis(3)
        elif 'Game' in self.name or 'Pro' in self.name:
            if abs(self.joystick.get_axis(0)) > self.joy_min:
                joy_vect[0] = self.joystick.get_axis(0)
            if abs(self.joystick.get_axis(1)) > self.joy_min:
                joy_vect[1] = -self.joystick.get_axis(1)
            if abs(self.joystick.get_axis(2)) > self.joy_min:
                joy_vect[2] = self.joystick.get_axis(2)
            if abs(self.joystick.get_axis(3)) > self.joy_min:
                joy_vect[3] = -self.joystick.get_axis(3)
        return joy_vect

    def get_hat(self):
        return self.joystick.get_hat(0)
