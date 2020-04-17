import time
import numpy as np
from robot import Quadrupedal



if __name__ == "__main__":
    targetPositionRF = np.array([0.2,-0.11,0.3])
    targetPositionRH = np.array([-0.2,-0.11,0.3])
    targetPositionLF = np.array([0.2,0.11,0.3])
    targetPositionLH = np.array([-0.2,0.11,0.3])

    qdrp = Quadrupedal(initialCoMheight=0.3,startPosition=[0,0,0.55],startOrientation=[0.,0.,0.], CoMposition_b=np.array([0.,0.,-0.02]), maxForce=12,robotPATH="urdf/quadrupedal.urdf")

    while(1):
        qdrp.inverseKinematics(targetPositionRF,targetLeg=qdrp.legRF)
        qdrp.inverseKinematics(targetPositionRH,targetLeg=qdrp.legRH)
        qdrp.inverseKinematics(targetPositionLF,targetLeg=qdrp.legLF)
        qdrp.inverseKinematics(targetPositionLH,targetLeg=qdrp.legLH)

        qdrp.oneStep()