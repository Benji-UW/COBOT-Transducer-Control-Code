# import json
# import sys
# import os
# path = os.path.abspath('')
# # print("======================")
# # print(path)
# sys.path.append(path)
# # print("======================")
# # print(sys.path)
# from Pathfinders import *

# file_no = 5

# # current_file = r"\Scans\test_%s.json" % file_no
# path = path + "\\Scans\\test_%s.json" % file_no

'''
-------------------------------------------------------------------

        Data migration is done, hopefully don't touch this
          again unless further breaking improvements are 
           brought to the pathfinder save_points method.

-------------------------------------------------------------------
'''

# print("Opening JSON file")
# with open(path, 'r') as infile:
#     json_data = json.load(infile)


# r_o_m = json_data["range of motion"]
# resolution = json_data["resolution"]
# points = json_data["points"]
# max_point = json_data["max_point"]

# res = tuple(resolution)
# (X,Y,Z,Rx,Ry,Rz) = (r_o_m['X'][1],
#                     r_o_m['Y'][1],
#                     r_o_m['Z'][1],
#                     r_o_m['Rx'][1],
#                     r_o_m['Ry'][1],
#                     r_o_m['Rz'][1])

# mig = FullScan(res,Z,Rx,Ry,X,Y)

# print("Starting point migration...")
# for point in points:    
#     a = np.array(point[0][0])
#     a = np.append(a,point[0][1])
#     a = np.append(a,point[1])
#     mig.newMag(a)
# print("Point migration complete.")

# print("Saving new JSON file...")
# mig.save_points()
# print("Done.")
# # Plan: observe what the current fullscan pathfinder returns,
# # write code to migrate that info into a new pathfinder module,
# # then save the data in a new scan test to make sure it works
# # properly, then profit. Also really gotta add code so that
# # the notes page mentions what kind of scan was performed. 
