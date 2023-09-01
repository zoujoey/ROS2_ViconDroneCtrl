import math
from math import sin, cos, tan, pi
def position_function(time):
    """
    This is the function that takes a time input and produces
    x, y, z, and yaw positions as output. 
    Modify this function as needed for continuous linear setpoints.
    """
    time = time/100
    x = time * 0.05  # Example position calculation
    y = 0.5*cos((2*pi*(x)))  # Example position calculation
    z = -(0.45) # Example position calculation
    yaw = 1.57  # Example yaw calculation
    
    return (x, y, z, yaw)