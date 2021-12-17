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
        elif 'Game' in self.name:
            self.B = 0 # B Button
            self.A = 1 # A Button
            self.Y = 2 # Y Button
            self.X = 3 # X Button
            self.L = 4
            self.R = 5
            self.ZL = 6 # Z-Left trigger, 
            self.ZR = 7 #
            self.M = 8 # Minus button
            self.P = 9 # Plus button
            self.L3 = 10 # Pushing down on the left joystick
            self.R3 = 11 # Pushing down on the right joystick
            self.H = 12
            self.S = 13
            self.PAD = 14
            self.max_buttons = 14
        elif 'Pro' in self.name:
            self.A = 0
            self.B = 1
            self.X = 2
            self.Y = 3
            self.M = 4
            self.home = 5
            self.P = 6
            self.L3 = 7
            self.R3 = 8
            self.L = 9
            self.R = 10
            self.D_up = 11
            self.D_down = 12
            self.D_right = 13
            self.D_lef = 14
            self.capture = 15
            self.max_buttons = 17
            self.PAD = 18
            self.ZL = 16
            self.ZR = 17
            

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
        elif 'Game' in self.name:
            for i in range(0, self.max_buttons):
                if self.joystick.get_button(i):
                    buttons.append(i)
        elif  'Pro' in self.name:
            for i in range(self.max_buttons - 3):
                if self.joystick.get_button(i):
                    buttons.append(i)
            # Equivalent l
            #buttons = [i for i in range(self.max_buttons) if self.joystick.get_button(i)]
            if self.joystick.get_axis(4) > 0.1:
                buttons.append(self.ZL)
            elif self.joystick.get_axis(5) > 0.1:
                buttons.append(self.ZR)
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
                joy_vect[3] = self.joystick.get_axis(3)
        return joy_vect

    def get_hat(self):
        if 'Pro' in self.name:
            up_down = 0
            if self.joystick.get_button(self.D_up):
                up_down = 1
            elif self.joystick.get_button(self.D_down):
                up_down = -1
            left_right = 0
            if self.joystick.get_button(self.D_lef):
                left_right = 1
            elif self.joystick.get_button(self.D_right):
                left_right = -1
            return [left_right, up_down]
        else:
            return self.joystick.get_hat(0)
