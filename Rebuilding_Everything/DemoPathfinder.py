'''
Unit convention: mm/kg/s/deg
'''
import math
import numpy as np
import json

class Pathfinder:
    '''A class for controlling where the arm goes. The tl;dr is you define a search space,
    with a starting and ending value for each degree of freedom. It can do between one and
    six degrees of freedom. The units for the linear dimensions are mm, the units for the
    angular dimensions are degrees. Everything is defined relative to the initial starting
    position, so the initial position, as far as this is concerned, is ((0,0,0),(0deg,0deg,0deg))
    All internal points are stored as tuples in the form ((X,Y,Z),(Rx,Ry,Rz))'''

    def __init__(self, z_range,Rx_range=0,Ry_range=0,x_range =0,y_range=0,Rz_range=0):
        '''Accepts as input a series of values indicating the range of different
        points in space it is allowed to exist between. This defines the search
        space of the object. It will not investigate any points outside these bounds.
        The only range it is require to accept is the z_range, otherwise it will
        default to zero. Z also defines the range to give more room to move backwards
        than forwards, in order to reduce the chances of the robot moving into the
        sample.'''
        z_r = [-z_range * 1.25, z_range * 0.75]
        Rx_r = [-Rx_range, Rx_range]
        Ry_r = [-Ry_range, Ry_range]
        x_r = [-x_range, x_range]
        y_r = [-y_range, y_range]
        Rz_r = [-Rz_range, Rz_range]
        self.range_of_motion = {'X': x_r,'Y': y_r,'Z':z_r,'Rx':Rx_r,'Ry':Ry_r,'Rz':Rz_r}

        self.points = []
        self.to_travel = []

        '''Sets the degrees of freedom of this pathfinder, only Z defaults to true.'''
        self.degrees_of_freedom = {'X': x_range!=0,'Y': y_range!=0,'Z':z_range!=0,'Rx':Rx_range!=0,'Ry':Ry_range!=0,'Rz':Rz_range!=0}
        self.starting_point_loader()

    def newMag(self, point_mag):
        '''Accepts as in put a tuple in the form (((X,Y,Z),(Rx,Ry,Rz)), mag), 
        where the first element in the tuple is a 6-member tuple representing
        the point in 6D space, and the second element is a float representing
        the signal magnitude at that point.'''
        self.points.append(point_mag)
        latest_point = point_mag[0]
        if self.close_enough(latest_point):
            self.to_travel.pop(0)

    def pointYielder(self):
        while True:
            yield self.to_travel[0]

    def next(self):
        return self.to_travel[0]

    def starting_point_loader(self):
        '''This method gets called when the pathfinder is initializes, and adds the center of 
        the search space as well as all the corners to the "to-travel" list.'''
        center_pt = [np.mean(self.range_of_motion[a]) for a in self.range_of_motion.keys()]
        self.to_travel.append((tuple(center_pt[:3]),tuple(center_pt[3:])))

        corners = set()

        for i in range(64):
            pos = (self.range_of_motion['X'][((-1)**int(i))],self.range_of_motion['Y'][((-1)**int(i/2))],self.range_of_motion['Z'][((-1)**int(i/4))])
            ang = (self.range_of_motion['Rx'][((-1)**int(i/8))],self.range_of_motion['Ry'][((-1)**int(i/16))],self.range_of_motion['Rz'][((-1)**int(i/32))])
            corners.add((pos,ang))
        
        for pt in corners:
            self.to_travel.append(pt)
        
    def close_enough(self, point, tolerance=(0.5,2)):
        '''Accepts as input a point and a tuple containing the dimensional tolerances,
        in the form of (mm, deg), where the first element is the toleranace of linear
        dimensions and the second is the angular tolerance. They default to 0.5 mm and 2 degrees.'''
        for i in range(3):
            if abs(self.to_travel[0][0][i] - point[0][i]) > tolerance[0]:
                return False
            if abs(self.to_travel[0][1][i] - point[1][i]) > tolerance[1]:
                return False
        return True

    def save_points(self, path):
        with open(path, 'w') as outfile:
            json.dump(self.points, outfile)

    