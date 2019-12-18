import pybullet as p
from motor import Motor
import numpy as np

class Quadrupedal:
    def __init__(self, robotId, motorDict, controlMode=p.POSITION_CONTROL, L1=0.18, L2=0.18, maxForce = 9.0):
        self.numMotor = p.getNumJoints(robotId)
        self.robotId = robotId
        self.controlMode = controlMode
        self.motorIdList = [i for i in range(self.numMotor)]

        self.rfHipMotor = Motor(robotId, motorDict["rfHip"])
        self.rfKneeMotor = Motor(robotId, motorDict["rfKnee"])

        self.rhHipMotor = Motor(robotId, motorDict["rhHip"])
        self.rhKneeMotor = Motor(robotId, motorDict["rhKnee"])
        
        self.lfHipMotor = Motor(robotId, motorDict["lfHip"])
        self.lfKneeMotor = Motor(robotId, motorDict["lfKnee"])

        self.lhHipMotor = Motor(robotId, motorDict["lhHip"])
        self.lhKneeMotor = Motor(robotId, motorDict["lhKnee"])

        self.L1 = L1
        self.L2 = L2

        self.maxForce = maxForce

        self.LegIdDict = {"rfLeg":[motorDict["rfHip"], motorDict["rfKnee"]], "lfLeg":[motorDict["lfHip"], motorDict["lfKnee"]], "rhLeg":[motorDict["rhHip"], motorDict["rhKnee"]], "lhLeg":[motorDict["lhHip"], motorDict["lhKnee"]] }
        

    def setMotorPositionByArray(self, jointIndices, goalPositions, maxForces):
        p.setJointMotorControlArray(self.robotId, jointIndices=self.motorIdList, controlMode=self.controlMode, forces=maxForces, targetPositions=goalPositions)

    
    def calculateIK(self, pos):
        x, y, z = pos
        M = np.sqrt(x**2 + z**2)
        theta1 = -np.arctan2(x,z) + np.arccos((self.L2*self.L2 - self.L1*self.L1 - M*M)/(-2*M*self.L1))  #koko
        theta2 = -np.pi + np.arccos((M*M - self.L1*self.L1 - self.L2*self.L2)/(-2 * self.L1 * self.L2))
        return [theta1, theta2]


    def torqueControlModeEnableForAll(self):
        self.rfHipMotor.torqueControlModeEnable()
        self.rfKneeMotor.torqueControlModeEnable() 

        self.rhHipMotor.torqueControlModeEnable()
        self.rhKneeMotor.torqueControlModeEnable() 
        
        self.lfHipMotor.torqueControlModeEnable()
        self.lfKneeMotor.torqueControlModeEnable()

        self.lhHipMotor.torqueControlModeEnable()
        self.lhKneeMotor.torqueControlModeEnable()

    def getMotorStates(self):
        p.getJointStates(self.robotId, JointIndices=self.IdList)


    def resetRobotPosAndOrient(self, position, orientation):
        p.resetBasePositionAndOrientation(self.robotId, position, orientation)

    def getEuler(self):
        _, qua = p.getBasePositionAndOrientation(self.robotId)
        return p.getEulerFromQuaternion(qua)

    def getQuaternion(self):
        _, orientation = p.getBasePositionAndOrientation(self.robotId)
        return orientation

    def getPosition(self):
        position, _ = p.getBasePositionAndOrientation(self.robotId)
        return position


    def setLegEndEffectorPositionByArray(self, rfLegPos, lfLegPos, rhLegPos, lhLegPos):
        rfHipPos, rfKneePos = self.calculateIK(rfLegPos)
        lfHipPos, lfKneePos = self.calculateIK(lfLegPos)
        rhHipPos, rhKneePos = self.calculateIK(rhLegPos)
        lhHipPos, lhKneePos = self.calculateIK(lhLegPos)

        rfHipPos, rfKneePos = rfHipPos*(-1), rfKneePos*(-1)
        rhHipPos, rhKneePos = rhHipPos*(-1), rhKneePos*(-1)

        self.setMotorPositionByArray(jointIndices=[self.rfHipMotor.Id, self.rfKneeMotor.Id, self.lfHipMotor.Id, self.lfKneeMotor.Id, self.rhHipMotor.Id, self.rhKneeMotor.Id, self.lhHipMotor.Id, self.lhKneeMotor.Id],
                goalPositions=[rfHipPos, rfKneePos, lfHipPos, lfKneePos, rhHipPos, rhKneePos, lhHipPos, lhKneePos],
                maxForces=[self.rfHipMotor.maxForce, self.rfKneeMotor.maxForce, self.lfHipMotor.maxForce, self.lfKneeMotor.maxForce, self.rhHipMotor.maxForce, self.rhKneeMotor.maxForce, self.lhHipMotor.maxForce, self.lhKneeMotor.maxForce])

    def setLegEndEffectorPosition(self, LegName, goalPosition):
        HipPos, KneePos = self.calculateIK(goalPosition)
        self.setMotorPositionByArray(goalPositions=[HipPos, KneePos], jointIndices=self.LegIdDict[LegNmae])

