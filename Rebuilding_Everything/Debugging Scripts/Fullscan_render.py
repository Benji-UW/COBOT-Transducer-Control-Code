from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import os
import json

path = os.path.dirname(__file__)
path = path + '\\ugh.json'

with open(path, 'r') as outfile:
    point_mags = json.load(outfile)

print(point_mags[0])

Zs = []
Rxs = []
Rys = []
mags = []

for p in point_mags:
    Zs.append(p[0][0][2])
    Rxs.append(p[0][1][0])
    Rys.append(p[0][1][1])
    mags.append(p[1])

ax = plt.axes(projection='3d')
ax.scatter(Rxs, Rys, Zs, c=mags, cmap='viridis', linewidth=0.5)