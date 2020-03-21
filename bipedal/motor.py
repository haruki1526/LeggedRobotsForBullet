import pybullet as p
import numpy as np


class Motor:


    def __init__(self, robotId, motorId, mode=p.POSITION_CONTROL, maxForce=6, maxVelocity=10):
        self.maxForce = maxForce
        self.maxVelocity = maxVelocity
        self.kp = 0.1
        self.kd = 0.1
        self.Id = motorId
        self._mode = mode
        self._robotId = robotId
        
    def setGoalPosition(self, goalPosition):
        p.setJointMotorControl2(robotId, jointIndex=self.Id, controlMode=self._mode, force=self.maxForce, targetPosition=goalPosition)

    def getInfo(self):
        return p.getJointInfo(self._robotId, JointIndex=self.Id)

    def getState(self):
        return getJointState(self._robotId, jointIndex=self.Id)

    def torqueControlModeEnable(self):
        p.setJointMotorControl2(robotId, JointIndex=self.Id, controlMode=p.VELOCITY_CONTROL, force=0)
        self._mode = p.TORQUE_CONTROL

    def getPosition(self):
        state = p.getJointState(self._robotId, jointIndex=self.Id)
        return state[0]

    def getVelocity(self):
        state = p.getJointState(self._robotId, jointIndex=self.Id)
        return state[1]

    def getTorque(self):
        state = p.getJointState(self._robotId, jointIndex=self.Id)
        return state[3]

    def PDcontrol(self, goalPosition):
        if self._mode == p.TORQUE_CONTROL:
            p = self.getPosition()
            p_ref = goalPosition 
            torque = (p - p_ref) * kp - self.getVelocity() * kd
            p.setJointMotorControl2(self._robotId, JointIndex=self.Id, controlMode=self._mode, force=torque)

        else:
            print("ModeError: must be TORQUE_CONTROL Mode")

        
