
import numpy as np
from robot import Bipedal





if __name__ == "__main__":

    bipedal = Bipedal() #



    


    jointPositions = bipedal.getJointPositions()
    print(jointPositions)
    pos = [0.0 for i in range(12)]
    while(1):
        bipedal.setMotorPositionByArray(goalPositions=pos)
        jointPositions = bipedal.getJointPositions()
        bipedal.oneStep()





    p.disconnect()

