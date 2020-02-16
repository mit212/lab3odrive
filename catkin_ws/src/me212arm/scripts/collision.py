#!/usr/bin/python

from planner import fk1, fk
import numpy as np

def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

# Return true if line segments AB and CD intersect
def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

def in_collision(q, obstacle_segs):
    arm_segments = [( (0,0), fk1(q) ), ( fk1(q), fk(q))]

    for seg in arm_segments:
        for obs in obstacle_segs:
            if intersect(seg[0], seg[1], obs[0], obs[1]):
                return True
    
    return False
    ## return True if there are segments from arm_segments and segments from obstacle_segs intersect

if __name__=="__main__":
    
    obstacle_segs = [ [[0.0,0.0], [0.6,0]] ]  # line segs ((x1,z1)--(x2,z2))
    print in_collision( [0,0], obstacle_segs)                       # False
    print in_collision( [np.pi/4, -3*np.pi/4], obstacle_segs)     # True
    print in_collision( [-0.45709828817786735, -1.4971869034039356], obstacle_segs) # False
    
    #   For Debugging: 
    # obstacle_segs = [ [[0.0, 0.1], [0.6, 0.1]] ]  # line segs ((x1,z1)--(x2,z2))
    # print in_collision( [0,0], obstacle_segs)                       # False
    # print in_collision( [-np.pi/4, 0], obstacle_segs)     # True
    # print in_collision( [np.pi/4, 0], obstacle_segs) # False
    # obstacle_segs = [ [[0.25,0.15], [0.4,0.2]], [[0.25,0.15-0.01], [0.4,0.2-0.01]] ]
    # print in_collision( [-1.1412249054766512, 1.1705410289473315], obstacle_segs)