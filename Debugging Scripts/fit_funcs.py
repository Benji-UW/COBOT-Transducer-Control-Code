import numpy as np

def r_squared(func, popt, x_dat, y_dat):
    residuals = y_dat - func(x_dat, *popt)
    ss_res = np.sum(residuals**2)

    ss_tot = np.sum((y_dat - np.mean(y_dat))**2)

    return 1 - (ss_res/ss_tot)

def polynomial_2(x, a, b, c):
    '''2nd order polynomial (is terrible)'''
    return (a*(x**2)) + (b*x) + c

def rational_2(x,a,b,c):
    '''2nd Order rational function :)'''
    return (1/((a*(x**2)) + (b*x) + c))

def gauss(x,a,b,c):
    '''Simplified Gaussian'''
    return (a * (np.e)**(-b * (x-c)**2))

def arctan(x,a,b,c):
    '''b * arctan(c/(x-a)^2)'''
    return (b * np.arctan(c/(x-a)**2))

def rational_3(x,a,b,c,d):
    '''Third order rational function 8/'''
    return ((1/((a*(x**3)) + (b*(x**2)) + c*x + d)))

def absolute(x,a,b,c):
    '''Simple absolute value function'''
    return a * np.abs((x-b)) + c