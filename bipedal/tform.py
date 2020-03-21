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
