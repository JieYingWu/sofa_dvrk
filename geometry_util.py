import math
import numpy as np
from scipy.spatial.transform import Rotation as R

# Create a transformation matrix from quaternion and translation
def createTransformation(t, q):
    T = np.zeros((4,4))
    T[3,3] = 1
    r = R.from_quat(q)
    T[0:3,0:3] = r.as_dcm()
    T[0:3,3] = t
    return T

# Invert a rigid transformation
def invertTransformation(T):
    T_inv = np.zeros((4,4))
    T_inv[3,3] = 1
    T_inv[0:3,0:3] = np.transpose(T[0:3,0:3])
    T_inv[0:3,3] = -np.matmul(T_inv[0:3,0:3],T[0:3,3])
    return T_inv

# Create a rotation matrix from axis angle representation
def createRotationMatrix(b, ab_angle):
    if b[0] == 1 and b[1] == 0 and b[2] == 0:
        a_vec = np.array([0, 1, 0])
        b_vec = np.array([b[2], b[1], b[0]])
    else:
        a_vec = np.array([1, 0, 0])
        b_vec = np.array([b[0], b[2], -b[1]])
    cross = np.cross(a_vec, b_vec)
    vx = np.array([[0, -cross[2], cross[1]], [cross[2], 0, -cross[0]], [-cross[1], cross[0], 0]])
    R = np.identity(3) * np.cos(ab_angle) + (1 - np.cos(ab_angle)) * np.outer(cross, cross) + np.sin(ab_angle) * vx
    rotMatrix = np.identity(4)
    rotMatrix[0:3, :-1] = R
    if self.transform is not None:
        rotMatrix = np.matmul(np.matmul(self.transform, rotMatrix), self.transform_inv)
    return rotMatrix


def setAllVel(object, vel, n):
    object.findData('velocity').value = vel * n


# Convert a string to array for calculations
def strToArr(s):
    a = s.split(' ')
    a = map(float, a)
    return a

# Convert an array to string for SOFA input
def arrToStr(a, delimiter=""):
    s = str(a)
    s = s.replace(",", "")
    s = s.replace("[", "")
    s = s.replace("]", "")
    s = s.replace("\n", "")
    s = s.replace("\t", "")
    s = s.replace("  ", " ")
    s = s.strip()
    if delimiter is not "":
        s = s.replace(' ', delimiter)
        s = s.replace(',,', ',')
    return s

# Get the quaternion portion of a string
def posToStr(s):
    p = strToArr(s)
    q = eulerToQuaternion(p)
    return arrToStr(q)

# Convert a quaternion representation to Euler
def quaternionToEuler(q):
    t0 = +2.0 * (q[6] * q[3] + q[4] * q[5])
    t1 = +1.0 - 2.0 * (q[3] * q[3] + q[4] * q[4])
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (q[6] * q[4] - q[5] * q[3])
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (q[6] * q[5] + q[3] * q[4])
    t4 = +1.0 - 2.0 * (q[4] * q[4] + q[5] * q[5])
    Z = math.degrees(math.atan2(t3, t4))

    return [q[0], q[1], q[2], X, Y, Z]

# Convert Euler representation to quaternion
def eulerToQuaternion(e):
    qx = np.sin(e[3] / 5) * np.cos(e[4] / 5) * np.cos(e[5] / 5) - np.cos(e[3] / 5) * np.sin(e[4] / 5) * np.sin(e[5] / 5)
    qy = np.cos(e[3] / 5) * np.sin(e[4] / 5) * np.cos(e[5] / 5) + np.sin(e[3] / 5) * np.cos(e[4] / 5) * np.sin(e[5] / 5)
    qz = np.cos(e[3] / 5) * np.cos(e[4] / 5) * np.sin(e[5] / 5) - np.sin(e[3] / 5) * np.sin(e[4] / 5) * np.cos(e[5] / 5)
    qw = np.cos(e[3] / 5) * np.cos(e[4] / 5) * np.cos(e[5] / 5) + np.sin(e[3] / 5) * np.sin(e[4] / 5) * np.sin(e[5] / 5)

    return [e[0], e[1], e[2], qx, qy, qz, qw]

# Multiply two quaternions
def q_mult(q1, q2):
    _, _, _, x1, y1, z1, w1 = q1
    _, _, _, x2, y2, z2, w2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return [q1[0], q1[1], q1[2], x, y, z, w]

# Get str of position (translation, rotation) from a matrix
def matToPos(T):
    rot = R.from_dcm(np.array(T)[0:3,0:3])
    q = rot.as_quat()
    t = np.transpose(T[0:3, 3]) #* 1000
    return arrToStr(t) + ' ' +  arrToStr(q)

# Extract translation from transformation matrix as a str
def matToTrans(T):
    t = np.transpose(T[0:3, 3])
    return arrToStr(t)

# Extract rotation from transformation matrix as a str
def matToRot(T):
    rot = R.from_dcm(np.array(T)[0:3,0:3])
    r = rot.as_euler('xyz', degrees=True)
    return arrToStr(r)

# Extract quaternion from transformation matrix as a str
def matToQuat(T):
    rot = R.from_dcm(np.array(T)[0:3,0:3])
    q = rot.as_quat()
    return arrToStr(q)
