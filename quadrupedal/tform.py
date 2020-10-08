import numpy as np


def getTransFromRp(R,p):
    T = np.vstack((np.hstack((R, np.array([[p[0]],[p[1]],[p[2]]]))), np.array([0,0,0,1])))
    return T

def getRotationRoll(theta):
    R = np.array([[1,0,0], [0,np.cos(theta),-np.sin(theta)],[0,np.sin(theta),np.cos(theta)]])
    return R

def getRotationPitch(theta):
    R = np.array([[np.cos(theta),0,np.sin(theta)], [0,1,0],[-np.sin(theta),0,np.cos(theta)]])
    return R

def getRotationYaw(theta):
    R = np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
    return R

def getPositionFromT(T):
    p = T[0:3,3]
    return p

def getRotationFromT(T):
    R = T[0:3,0:3]
    return R

def getRotationAndPositionFromT(T):
    p = T[0:3,3]
    R = T[0:3,0:3]
    return R, p

def getRollPitchYawFromR(R):
    pitch = np.arcsin(-R[2,0])
    yaw = np.arctan2(R[1,0], R[0,0])
    roll = np.arctan2(R[2,1],R[2,2])

    return np.array([roll, pitch, yaw])

def skewSymmetricMatrix(v):
    matrix = np.array([[0, -v[2], v[1]],
                        [v[2], 0, -v[0]],
                        [-v[1], v[0], 0]])
    return matrix

def skewSymmetricMatrixInv(m):
    vector = np.zeros(3)
    vector[0] = m[2,1]
    vector[1] = m[0,2]
    vector[2] = m[1,0]

    return vector



def roadriguesEquation(E, a, theta):
    a_h = skewSymmetricMatrix(a)
    return E + a_h * np.sin(theta) + a_h.dot(a_h) * (1-np.cos(theta))
