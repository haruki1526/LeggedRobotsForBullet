import numpy as np
from robot import Bipedal
import tform as tf
import time





if __name__ == "__main__":

    bipedal = Bipedal() 

    initializeTime = 1.0
    bipedal.positionInitialize(initializeTime=initializeTime)


    tarPosR = [0.0, -0.065, -0.4]
    tarPosL = [0.0, 0.065, -0.4]
    targetRPY = [0.0, 0.0, 0.0]
    while(1):
        PosR = bipedal.inverseKinematics(tarPosR, targetRPY, bipedal.R)
        PosL = bipedal.inverseKinematics(tarPosL, targetRPY, bipedal.L)
        bipedal.setLeftLegJointPositions(PosL)
        bipedal.setRightLegJointPositions(PosR)
        #R, p = bipedal.forwardKinematics(jointPositions=bipedal.getJointPositions(bipedal.R), leg=bipedal.R)
        #omega = tf.getRollPitchYawFromR(R)
        print("pos=", PosR)


        bipedal.oneStep()





    p.disconnect()

