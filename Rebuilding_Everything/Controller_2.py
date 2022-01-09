import numpy as np
import json
import pygame

class Controller_2:
    def __init__(self,joystick):
        self.joy_min = 0.1
        self.buttons = {}
        self.mapping = {}

        # Go into this if statement if the joystick is a joystick
        if (joystick != -1):
            self.joystick = joystick
            self.name = self.joystick.get_name()
            controller_file = open(r'Rebuilding_Everything\controllers.json')
            data = json.load(controller_file)
            keep_going = True
            
            try:
                short_name = self.short_name(self.name, data.keys())
            except KeyError as e:
                print("That controller isn't saved, you will have to configure it yourself.")
                print("For now we're sticking to keyboard mode :)")
                self.joystick = None
                #keep_going = False

            if (keep_going):
                self.buttons = data[short_name]
                mapping_file = open(r'Rebuilding_Everything\mapping.json')
                data = json.load(mapping_file)

                if short_name == 'Pro':
                    self.mapping = data['default_Pro']
                else:
                    print("You're going to need to configure a mapping for that controller :/")
                    print("For now we're sticking to keyboard mode :)")
                    self.joystick = None
        else:
            self.name = 'keyboard'
            mapping_file = open('mapping.json')
            data = json.load(mapping_file)

            self.mapping = data["default_keyboard"]
            
    def short_name(self, name, options):
        for shortened in options:
            if shortened in name:
                return shortened
        raise KeyError("Mapping key not found")

    def get_buttons(self, keys):
        pressed_buttons = {}
        funcs = list(self.mapping.keys())[3:] # Skips the first three because they're joystick axes
        
        if self.name == 'keyboard':
            butts = [keys[pygame.key.key_code(self.mapping[a])] for a in funcs]
            pressed_buttons = dict(zip(funcs, butts))
            print(pressed_buttons)
            # pressed_buttons["switch space"] = keys[pygame.key.key_code(self.mapping["switch_space"])]
            # pressed_buttons["speed up"] = keys[pygame.key.key_code(self.mapping["speed_preset_up"])]
            # pressed_buttons["speed down"] = keys[pygame.key.key_code(self.mapping["speed_preset_down"])]
            # pressed_buttons["set origin"] = keys[pygame.key.key_code(self.mapping["set_o"])]
            # pressed_buttons["toggle freedrive"] = keys[pygame.key.key_code(self.mapping["toggle_freedrive"])]
            # pressed_buttons["goto origin"] = keys[pygame.key.key_code(self.mapping["goto_o"])]
            # pressed_buttons["change base"] = keys[pygame.key.key_code(self.mapping["change_base"])]
            # pressed_buttons["exit"] = keys[pygame.key.key_code(self.mapping["exit"])]
        else:
            butts = [self.joystick.get_button(self.buttons[self.mapping[a]]) for a in funcs]
            pressed_buttons = dict(zip(funcs, butts))
        
        return pressed_buttons

    def get_hat(self,keys):
        joy_vect = np.zeros((3,1))
        axes = list(self.mapping.keys())[:3]
        if self.name == 'keyboard':
            joy_vect[0] = keys[pygame.key.key_code(axes['x-axis'][0])] - keys[pygame.key.key_code(axes['x-axis'][1])]
            joy_vect[0] = keys[pygame.key.key_code(axes['y-axis'][0])] - keys[pygame.key.key_code(axes['y-axis'][1])]
            joy_vect[0] = keys[pygame.key.key_code(axes['z-axis'][0])] - keys[pygame.key.key_code(axes['z-axis'][1])]
            joy_vect = joy_vect / np.linalg.norm(joy_vect)
        else:
            joy_vect[0]=self.joystick.get_axis(self.mapping['x-axis'])
            joy_vect[1]=self.joystick.get_axis(self.mapping['y-axis'])
            joy_vect[2]=self.joystick.get_axis(self.mapping['z-axis'])
            if np.linalg.norm(joy_vect) < self.joy_min:
                joy_vect = np.zeros((3,1))
        
        return joy_vect
    