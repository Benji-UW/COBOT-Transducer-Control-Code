import json
import sys
import os
path = os.path.abspath('')
print("======================")
print(path)
sys.path.append(path)
print("======================")
print(sys.path)
from Pathfinders import *

file_no = 6

current_file = r"\Scans\test_%s.json" % file_no
path = os.path.abspath('..\\') + current_file

with open(path, 'r') as infile:
    json_data = json.load(infile)


r_o_m = json_data["range of motion"]
resolution = json_data["resolution"]
points = json_data["points"]
max_point = json_data["max_point"]


# Plan: observe what the current fullscan pathfinder returns,
# write code to migrate that info into a new pathfinder module,
# then save the data in a new scan test to make sure it works
# properly, then profit. Also really gotta add code so that
# the notes page mentions what kind of scan was performed. 