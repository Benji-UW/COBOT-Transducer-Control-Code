import numpy as np
import json
import pygame

class Controller_2:
    def __init__(self, joystick=None):
        
        self.joy_min = 0.1
        self.buttons = {}
        self.mapping = {}
        if (self.joystick is not None):
            self.joystick = joystick
            self.name = self.joystick.get_name()
            controller_file = open('controllers.json')
            data = json.load(controller_file)
            keep_going = True
            
            try:
                short_name = self.short_name(self.name, data.keys())
            except KeyError as e:
                print("That controller isn't saved, you will ahve to configure it yourself.")
                print("For now we're sticking to keyboard mode :)")
                self.joystick = None
                keep_going = False

            if (keep_going):
                self.buttons = data[short_name]
                mapping_file = open('mapping.json')
                data = json.load(mapping_file)

                if short_name is 'Pro':
                    self.mapping = data['default_Pro']
                else:
                    print("You're going to need to configure a mapping for that controller :/")
                    print("For now we're sticking to keyboard mode :)")
                    self.joystick = None
        elif (self.joystick is None):
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
        if self.name is 'keyboard':
            switch_space = keys[pygame.key.key_code(self.mapping["switch_space"])]
            speed_preset_up = keys[pygame.key.key_code(self.mapping["speed_preset_up"])]
            speed_preset_down = keys[pygame.key.key_code(self.mapping["speed_preset_down"])]
            set_o = keys[pygame.key.key_code(self.mapping["set_o"])]
            toggle_freedrive = keys[pygame.key.key_code(self.mapping["toggle_freedrive"])]
            goto_o = keys[pygame.key.key_code(self.mapping["goto_o"])]
            change_base = keys[pygame.key.key_code(self.mapping["change_base"])]
            exit = keys[pygame.key.key_code(self.mapping["exit"])]
            pass # do a bunch of things for the keyboard mode
        else:
            pass # Do all the things for if it's the 

    