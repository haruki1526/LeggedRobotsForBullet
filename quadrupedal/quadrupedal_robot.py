import pybullet as p
import time
import pybullet_data

import numpy as np

from robot import Quadrupedal

MAX_FORCE = 12
MAX_VELOCITY = 7



if __name__ == "__main__":
    motorDict = {"rfHip":0, "rfKnee":1, "lfHip":2, "lfKnee":3, "rhHip":4, "rhKnee":5, "lhHip":6, "lhKnee":7}

    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0,0,-9.8)
    planeId = p.loadURDF("plane.urdf")
    robotStartPos = [0,0,0.5]
    robotStartOrientation = p.getQuaternionFromEuler([0,0,0])
    robotId = p.loadURDF("urdf/quadrupedal_robot2.urdf",robotStartPos, robotStartOrientation)

    quadrupedal = Quadrupedal(robotId, motorDict=motorDict)
    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=-30, cameraPitch=-30, cameraTargetPosition=[0.0, 0.0, 0.25])

    #p.setRealTimeSimulation(1)
    pos = np.array([0.0, 0.0, 0.3])

    while(1):
        robotPosition = quadrupedal.getPosition()
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=90, cameraPitch=-30, cameraTargetPosition=robotPosition)
        p.stepSimulation()
        time.sleep(1./200.)
        quadrupedal.setLegEndEffectorPositionByArray(rfLegPos=pos, lfLegPos=pos, rhLegPos=pos, lhLegPos=pos)



    p.disconnect()
            
