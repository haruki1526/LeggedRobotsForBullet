import numpy as np
from robot import Bipedal
import tform as tf
import time





if __name__ == "__main__":

    bipedal = Bipedal() 



    jointPositions = bipedal.getJointPositions(bipedal.R)
    print(jointPositions)
    tarPosR = [0.0, -0.065, -0.35]
    tarPosL = [0.0, 0.065, -0.35]
    targetRPY = [0.0, 0.0, 0.0]
    posR = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    startJointPosR = [0.0, 0.0, -0.6,  2*0.6, -0.6, 0.0]
    startJointPosL = [0.0, 0.0, -0.6,  2*0.6, -0.6, 0.0]
    bipedal.setLeftLegJointPositions(startPosL)
    bipedal.setRightLegJointPositions(startPosR)
    bipedal.oneStep()
    time.sleep(2)
    while(1):
        PosR = bipedal.inverseKinematics(tarPosR, targetRPY, bipedal.R)
        PosL = bipedal.inverseKinematics(tarPosL, targetRPY, bipedal.L)
        bipedal.setLeftLegJointPositions(PosL)
        bipedal.setRightLegJointPositions(PosR)
        #R, p = bipedal.forwardKinematics(jointPositions=bipedal.getJointPositions(bipedal.R), leg=bipedal.R)
        #omega = tf.getRollPitchYawFromR(R)
        #print("omega=",omega)


        bipedal.oneStep()





    p.disconnect()

