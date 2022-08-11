import numpy as np

def r_squared(func, popt, x_dat, y_dat):
    residuals = y_dat - func(x_dat, *popt)
    ss_res = np.sum(residuals**2)

    ss_tot = np.sum((y_dat - np.mean(y_dat))**2)

    return 1 - (ss_res/ss_tot)

def polynomial_2(x, a, b, c):
    '''2nd order polynomial'''
    return (a*(x**2)) + (b*x) + c

def rational_2(x,a,b,c,d):
    '''2nd Order rational function'''
    return (1/((a*(x**2)) + (b*x) + c))

def gauss(x,a,b,x_offset,y_offset):
    '''Gaussian fit'''
    return (a * (np.e)**(-b * (x-x_offset)**2)) + y_offset

def arctan(x,a,b,x_offset):
    '''a * arctan(b/(x-x_offset)^2)'''
    return (a * np.arctan(b/(x-x_offset)**2))

def rational_3(x,a,b,c,d):
    '''Third order rational function 8/'''
    return ((1/((a*(x**3)) + (b*(x**2)) + c*x + d)))

def absolute(x,a,b,x_offset):
    '''Simple absolute value function'''
    return a * np.abs((x-x_offset)) + b

def inv_absolute(x,a,b,x_offset,y_offset):
    '''Rational absolute value function'''
    return (a / (np.abs((x-x_offset)) + b)) + y_offset

def inv_abs_poly(x,a,b,x_offset):
    '''Rational absolute value polynomial'''
    return a / (np.abs((x-x_offset)**2 + b))

def inv_abs_poly_4(x,a,b,c,d,e,f,y_offset):
    '''Rational absolute value 4th degree polynomial'''
    return (a / (np.abs(x**4 + (x*b)**3 + (x*c)**2 + (x*d) + e) + f)) + y_offset





def ang_plus_z(data,a,b,c,x_offset,y_offset,z_offset):
    '''The inv_absolute should be applied to the Z data, gauss to the Ry data'''
    x=data[0]
    y=data[1]

    return (gauss(x,a,b,x_offset,z_offset) * inv_absolute(y,1,c,y_offset,0))

def double_ang(data,a,b,x_offset,y_offset,z_offset):
    x=data[0]
    y=data[1]

    return (a * (np.e)**(-b * (np.arccos(np.cos(np.deg2rad(x-x_offset)) * np.cos(np.deg2rad(y-y_offset))))**2)) + z_offset
    # return (a * (np.e)**(-b * (np.arccos(np.cos(np.deg2rad(x-x_offset)) * np.cos(np.deg2rad(y))))**2)) + z_offset

def Z_Rx_Ry(data,a,b,c,d,Rx_offset,Ry_offset,z_offset,mag_offset):
    '''Must be passed the data in the form [Z,Rx,Ry]'''
    z=data[0]
    Rx=data[1]
    Ry=data[2]

    return (double_ang([Rx,Ry],a,b,Rx_offset,Ry_offset,mag_offset) * inv_absolute(z,c,d,z_offset,0)) + mag_offset