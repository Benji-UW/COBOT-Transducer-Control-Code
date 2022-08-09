import numpy as np

def r_squared(func, popt, x_dat, y_dat):
    residuals = y_dat - func(x_dat, *popt)
    ss_res = np.sum(residuals**2)

    ss_tot = np.sum((y_dat - np.mean(y_dat))**2)

    return 1 - (ss_res/ss_tot)

def polynomial_2(x, a, b, c):
    '''2nd order polynomial (is terrible)'''
    return (a*(x**2)) + (b*x) + c

def rational_2(x,a,b,c,d):
    '''2nd Order rational function'''
    return (1/((a*(x**2)) + (b*x) + c))

def gauss(x,a,b,c):
    '''Gaussian'''
    return (a * (np.e)**(-b * (x-c)**2))

def gauss_plus(x,a,b,c,d):
    '''Gaussian plus'''
    return (a * (np.e)**(-b * (x-c)**2)) + d

def arctan(x,a,b,c):
    '''b * arctan(c/(x-a)^2)'''
    return (b * np.arctan(c/(x-a)**2))

def rational_3(x,a,b,c,d):
    '''Third order rational function 8/'''
    return ((1/((a*(x**3)) + (b*(x**2)) + c*x + d)))

def absolute(x,a,b,c):
    '''Simple absolute value function'''
    return a * np.abs((x-b)) + c

def inv_absolute(x,a,b,c):
    '''Rational absolute value function'''
    return (a / (np.abs((x-b)) + c))

def inv_abs_poly(x,a,b,c):
    '''Rational absolute value polynomial'''
    return a / (np.abs((x-b)**2 + c))

def inv_abs_poly_4(x,a,b,c,d,e,f,g,h):
    '''Rational absolute value 4th degree polynomial'''
    return (a / (np.abs(x**4 + (x*c)**3 + (x*d)**2 + (x*e) + f) + g)) + h





def ang_plus_z(data,a,b,c,d,e,f,g):
    '''The inv_absolute should be applied to the Z data, gauss to the Ry data'''
    y=data[0]
    x=data[1]

    return (gauss(x,a,b,c) * inv_absolute(y,d,e,f)) + g

def double_ang(data,a,b,c):
    # print(data)
    x=data[0]
    y=data[1]
    # print(x[0:10])
    # print(y[0:10])
    # print(np.deg2rad(x)[0:10])
    # print(np.deg2rad(y)[0:10])
    # print(np.cos(np.deg2rad(x)[0:10]))
    # print(np.cos(np.deg2rad(y)[0:10]))
    # t = np.rad2deg(np.arccos(np.cos(np.deg2rad(y)) * np.cos(np.deg2rad(x))))
    
    # print('------------------')
    # print(t[0:10])


    return (a * (np.e)**(-b * (np.arccos(np.cos(np.deg2rad(x)) * np.cos(np.deg2rad(y))))**2)) + c

def Z_Rx_Ry(data,a,b,c,d,e,f,g):
    '''Must be passed the data in the form [Z,Rx,Ry]'''
    z=data[0]
    Rx=data[1]
    Ry=data[2]

    return (double_ang([Rx,Ry], a,b,c) * inv_absolute(z,d,e,f)) + g