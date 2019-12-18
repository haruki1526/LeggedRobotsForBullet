import pybullet as p
import numpy as np

MAX_FORCE = 6
MAX_VELOCITY = 10

class Motor:


    def __init__(self, robotId, motorId, mode=p.POSITION_CONTROL):
        self.maxForce = MAX_FORCE
        self.maxVelocity = MAX_VELOCITY
        self.kp = 1
        self.kd = 1
        self.mode = mode
        self.Id = motorId
        
    def setGoalPosition(self, goalPosition):
        p.setJointMotorControl2(robotId, jointIndex=self.Id, controlMode=self.mode, force=self.maxForce, targetPosition=goalPosition)

    def getInfo(self):
        return p.getJointInfo(robotId, JointIndex=self.Id)

    def getState(self):
        return getJointState(robotId, jointIndex=self.Id)

    def torqueControlModeEnable(self):
        p.setJointMotorControl2(robotId, JointIndex=self.Id, controlMode=p.VELOCITY_CONTROL, force=0)
        self.mode = p.TORQUE_CONTROL

    def getPosition(self):
        state = p.getJointState(self.robotId, jointIndex=self.Id)
        return state[0]

    def getVelocity(self):
        state = p.getJointState(self.robotId, jointIndex=self.Id)
        return state[1]

    def getTorque(self):
        state = p.getJointState(self.robotId, jointIndex=self.Id)
        return state[3]
