import time
import numpy as np
from robot import Quadrupedal



if __name__ == "__main__":
    targetPositionRF = np.array([0.2,-0.11,-0.2])
    targetPositionRH = np.array([-0.2,-0.11,-0.2])
    targetPositionLF = np.array([0.2,0.11,-0.2])
    targetPositionLH = np.array([-0.2,0.11,-0.2])

    qdrp = Quadrupedal(initialCoMheight=0.3,startPosition=[0,0,0.55],startOrientation=[0.,0.,0.], CoMposition_b=np.array([0.,0.,-0.02]), maxForce=12,robotPATH="urdf/quadrupedal.urdf")
    print("kita")

    while(1):
        for i in np.arange(0,0.1,0.001):
            targetPositionLF[2] += 0.001
            targetPositionRF[2] += 0.001
            targetPositionLH[2] += 0.001
            targetPositionRH[2] += 0.001
            print("tar=",targetPositionLF)
            LFjointPositions = qdrp.inverseKinematics(targetPositionRH,targetLeg=qdrp.legRH)
            RFjointPositions = qdrp.inverseKinematics(targetPositionRF,targetLeg=qdrp.legRF)
            LHjointPositions = qdrp.inverseKinematics(targetPositionLF,targetLeg=qdrp.legLF)
            RHjointPositions = qdrp.inverseKinematics(targetPositionLH,targetLeg=qdrp.legLH)
            qdrp.legLF.setJointPositions(LFjointPositions)
            qdrp.legRF.setJointPositions(RFjointPositions)
            qdrp.legLH.setJointPositions(LHjointPositions)
            qdrp.legRH.setJointPositions(RHjointPositions)
            qdrp.oneStep()

        for i in np.arange(0,0.1,0.001):
            targetPositionLF[2] -= 0.001
            targetPositionRF[2] -= 0.001
            targetPositionLH[2] -= 0.001
            targetPositionRH[2] -= 0.001
            LFjointPositions = qdrp.inverseKinematics(targetPositionRH,targetLeg=qdrp.legRH)
            RFjointPositions = qdrp.inverseKinematics(targetPositionRF,targetLeg=qdrp.legRF)
            LHjointPositions = qdrp.inverseKinematics(targetPositionLF,targetLeg=qdrp.legLF)
            RHjointPositions = qdrp.inverseKinematics(targetPositionLH,targetLeg=qdrp.legLH)
            qdrp.legLF.setJointPositions(LFjointPositions)
            qdrp.legRF.setJointPositions(RFjointPositions)
            qdrp.legLH.setJointPositions(LHjointPositions)
            qdrp.legRH.setJointPositions(RHjointPositions)
            qdrp.oneStep()
