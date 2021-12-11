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
            
            try:
                short_name = self.short_name(self.name, data.keys())
            except KeyError as e:
                print("That controller isn't saved, you will ahve to configure it yourself.")
                print("For now we're sticking to keyboard mode :)")
                self.joystick = None
                break
            self.buttons = data[short_name]
            
            mapping_file = open('mapping.json')
            data = json.load(mapping_file)

            if short_name is 'Pro':
                self.mapping = data['default_Pro']
            else:
                print("You're going to need to configure a mapping for that controller :/")
                print("For now we're sticking to keyboard mode :)")
                self.joystick = None
                break
        elif (self.joystick is None):
            self.name = 'keyboard'
                
    def short_name(self, name, options):
        for shortened in options:
            if shortened in name:
                return shortened
        raise KeyError("Mapping key not found")

    