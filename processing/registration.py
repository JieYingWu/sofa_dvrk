import sys
from numpy import *
from math import sqrt
import geometry_util as geo

# Got from http://nghiaho.com/uploads/code/rigid_transform_3D.py_
# Input: expects Nx3 matrix of points
# Returns R,t
# R = 3x3 rotation matrix
# t = 3x1 column vector

def rigid_transform_3D(A, B):
    
    assert len(A) == len(B)

    N = A.shape[0]; # total points

    centroid_A = mean(A, axis=0)
    centroid_B = mean(B, axis=0)
    
    # centre the points
    AA = A - tile(centroid_A, (N, 1))
    BB = B - tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    H = dot(transpose(AA), BB)
    
    U, S, Vt = linalg.svd(H)

    R = dot(Vt.T, U.T)

    # special reflection case
    if linalg.det(R) < 0:
       print "Reflection detected"
       Vt[2,:] *= -1
       R = Vt.T * U.T

    t = dot(-R,centroid_A.T) + centroid_B.T

#    print t

    return R, t


def strToArr(s):
    s = s.replace('[', '') 
    s = s.split(']')
    np = array([fromstring(s[0], dtype=float, sep=' '), fromstring(s[1], dtype=float, sep=' '), fromstring(s[2], dtype=float, sep=' '), fromstring(s[3], dtype=float, sep=' ')])
    return np

if __name__=='__main__':
    output = open(sys.argv[2], 'w')
    with open(sys.argv[1], 'r') as fp:
        base = strToArr(fp.readline())
        line = fp.readline()
        while line:
            arr = strToArr(line)
            R, t = rigid_transform_3D(base, arr)
            q = geo.matToQuat(R)
            output.write(geo.arrToStr(t) + q + '\n')
            line = fp.readline()
    output.close()
