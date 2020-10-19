import numpy as np
from robot import Bipedal
import tform as tf
import time
from walking_generator import PreviewControl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



if __name__ == "__main__":
    bipedal = Bipedal() 
    zc = 0.45 #Center of Mass height [m]
    stride = 0.1
    CoM_to_body = np.array([0.0, 0.0, 0.0]) #from CoM to body coordinate

    targetRPY = [0.0, 0.0, 0.0]
    pre = PreviewControl(Tsup_time=0.3,Tdl_time=0.1, previewStepNum=190)#preview control
    bipedal.positionInitialize(initializeTime=0.2)
    CoMTrajectory = np.empty((0,3), float)

    trjR_log = np.empty((0,3), float)
    trjL_log = np.empty((0,3), float)
    walkingCycle = 50
    supPoint = np.array([0.,0.065])
    for w in range(walkingCycle):
        comTrj,footTrjL,footTrjR = pre.footPrintAndCOMtrajectoryGenerator(inputTargetZMP=supPoint, inputFootPrint=supPoint) #generate one cycle trajectory


        CoMTrajectory = np.vstack((CoMTrajectory, comTrj))
        trjR_log = np.vstack((trjR_log, footTrjR ))
        trjL_log = np.vstack((trjL_log, footTrjL ))
            
        com_len = len(comTrj)
        for i in range(com_len):
            targetPositionR = footTrjR[i] - comTrj[i]
            targetPositionL = footTrjL[i] - comTrj[i]

            PosR = bipedal.inverseKinematics(targetPositionR, targetRPY, bipedal.R)
            PosL = bipedal.inverseKinematics(targetPositionL, targetRPY, bipedal.L)
            bipedal.setLeftLegJointPositions(PosL)
            bipedal.setRightLegJointPositions(PosR)

            bipedal.oneStep()

        supPoint[0] += stride
        supPoint[1] = -supPoint[1]
        
    
    #debug Com
    figx = plt.figure(figsize=(10,10))
    comx = figx.add_subplot(111)
    comx.plot(CoMTrajectory[:,0],label="CoMtrj",color="blue")
    comx.plot(pre.px_ref_log[:],label="targetZMP",color="black")
    comx.plot(pre.px,label="ZMP",color="red")

    figy = plt.figure(figsize=(10,10)) 
    comy = figy.add_subplot(111)
    comy.plot(CoMTrajectory[:,1],label="CoMtrj",color="blue")
    comy.plot(pre.py_ref_log[:],label="targetZMP",color="black")
    comy.plot(pre.py,label="ZMP",color="red")
    plt.legend()

    #foot trajectory 

    fig = plt.figure(figsize=(10,10))
    ax1 = Axes3D(fig)
    ax1.set_xlabel("x")
    ax1.set_ylabel("y")
    ax1.set_zlabel("z")
    ax1.plot(trjL_log[:,0],trjL_log[:,1],trjL_log[:,2],marker="o",linestyle='None')
    ax1.plot(trjR_log[:,0],trjR_log[:,1],trjR_log[:,2],marker="o",linestyle='None')

    ax1.plot(CoMTrajectory[:,0],CoMTrajectory[:,1],CoMTrajectory[:,2],marker="o",linestyle='None')
    plt.show()



    bipedal.disconnect()

