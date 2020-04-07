import numpy as np
from robot import Bipedal
import time
from mpl_toolkits.mplot3d import Axes3D



if __name__ == "__main__":
    bipedal = Bipedal() 
    zc = 0.45 #Center of Mass height

    targetRPY = [0.0, 0.0, 0.0]
    targetPositionL = [0.0,0.065,-zc]
    targetPositionR = [0.0,-0.065,-zc]
    bipedal.positionInitialize(initializeTime=0.2)

    debugCycle = 20
    while(1):
        PosL = bipedal.inverseKinematics(targetPositionL, targetRPY, bipedal.L)
        PosR = bipedal.inverseKinematics(targetPositionR, targetRPY, bipedal.R)
        bipedal.setLeftLegJointPositions(PosL)
        bipedal.setRightLegJointPositions(PosR)

        bipedal.oneStep()



    bipedal.disconnect()

