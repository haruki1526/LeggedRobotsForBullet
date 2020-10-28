import pybullet as pb
import pybullet_data
import numpy as np
import time
import tform as tf
import scipy.linalg as la

class Robot:
    def __init__(self, robotPATH, startPosition, startOrientation, maxForce, controlMode=pb.POSITION_CONTROL, planePATH="plane.urdf"):
        physicsClient = pb.connect(pb.GUI)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.setGravity(0,0,-9.8)
        self._planeId = pb.loadURDF(planePATH)
        self._robotId = pb.loadURDF(robotPATH,startPosition, pb.getQuaternionFromEuler(startOrientation))
        self._controlMode = controlMode
        self.numJoint = pb.getNumJoints(self._robotId)
        self._jointIdList = list(range(self.numJoint))

        self.maxForce = maxForce
        self._maxForceList = [maxForce]*12

        self._timeStep = 1./240. 


    def getEuler(self):
        _, qua = pb.getBasePositionAndOrientation(self._robotId)
        return pb.getEulerFromQuaternion(qua)

    def getQuaternion(self):
        _, orientation = pb.getBasePositionAndOrientation(self._robotId)
        return orientation

    def getRobotPosition(self):
        position, _ = pb.getBasePositionAndOrientation(self._robotId)
        return position

    def resetRobotPositionAndOrientation(self, position, orientation):
        pb.resetBasePositionAndOrientation(self._robotId, position, orientation)

    def setMotorTorqueByArray(self, targetJointTorqueList):
        if self._controlMode is pb.TORQUE_CONTROL:
            pb.setJointMotorControlArray(self._robotId, jointIndices=self._jointIdList, controlMode=pb.TORQUE_CONTROL, forces=targetJointTorqueList)
        else:
            print("Error: Mode must be set to TORQUE MODE")
        

    def setMotorPositionByArray(self, targetJointPositionList):
        pb.setJointMotorControlArray(self._robotId, jointIndices=self._jointIdList, controlMode=self._controlMode, forces=self._maxForceList, targetPositions=targetJointPositionList)

    def oneStep(self):
        
        robotPosition, _ = pb.getBasePositionAndOrientation(self._robotId)
        pb.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=135, cameraPitch=-10, cameraTargetPosition=robotPosition)
        pb.stepSimulation()
        time.sleep(self._timeStep)


class Bipedal(Robot):

    def __init__(self, startPosition=[0,0,0.55], startOrientation=[0,0,0], CoMposition_b=np.array([0.,0.,-0.02]),maxForce=9.0, controlMode=pb.POSITION_CONTROL, robotPATH="urdf/bipedal.urdf", planePATH="plane.urdf"):
        super().__init__(robotPATH, startPosition, startOrientation, maxForce, controlMode=controlMode, planePATH=planePATH)

        self._lambda = 1.0
        self._L1 = 0.18
        self._L2 = 0.18
        self.R = np.array([0,-0.065,-0.175]) - CoMposition_b #from CoM to hipyaw joint
        self.L = np.array([0,0.065,-0.175]) - CoMposition_b
        self.LEG_DOF = 6

        self._jointIdListR = [0,1,2,3,4,5]
        self._jointIdListL = [6,7,8,9,10,11]
        self._maxForceListForLeg = [maxForce]*self.LEG_DOF

        #joint axis matrix
        self._a = np.array([[0,0,1], 
                                [1,0,0],
                                [0,1,0],
                                [0,1,0],
                                [0,1,0],
                                [1,0,0]])

        self._E = np.eye(3)



    def setRightLegJointPositions(self, targetJointPositions):
        pb.setJointMotorControlArray(self._robotId, jointIndices=self._jointIdListR, controlMode=self._controlMode, forces=self._maxForceListForLeg, targetPositions=targetJointPositions)

    def setLeftLegJointPositions(self, targetJointPositions):
        pb.setJointMotorControlArray(self._robotId, jointIndices=self._jointIdListL, controlMode=self._controlMode, forces=self._maxForceListForLeg, targetPositions=targetJointPositions)

    

    def torqueControllModeEnableForAll(self):

        pb.setJointMotorControlArray(self._robotId, jointIndices=self._jointIdList, controlMode=pb.VELOCITY_CONTROL, forces=[0]*12)
        self._controlMode = pb.TORQUE_CONTROL
        
        
    def getLegTrans(self, jointPositions, leg):
        hipyaw = jointPositions[0]
        hiproll = jointPositions[1]
        hippitch = jointPositions[2]
        knee = jointPositions[3]
        anklepitch = jointPositions[4]
        ankleroll = jointPositions[5]
        zero_v = np.zeros(3)

        T_0_1 = tf.getTransFromRp(tf.roadriguesEquation(self._E, self._a[0], hipyaw),leg)
        T_0_2 = T_0_1.dot( tf.getTransFromRp(tf.roadriguesEquation(self._E, self._a[1], hiproll), zero_v))
        T_0_3 = T_0_2.dot( tf.getTransFromRp(tf.roadriguesEquation(self._E, self._a[2], hippitch), zero_v))
        T_0_4 = T_0_3.dot( tf.getTransFromRp(tf.roadriguesEquation(self._E, self._a[3], knee), [0,0,-self._L1]))
        T_0_5 = T_0_4.dot( tf.getTransFromRp(tf.roadriguesEquation(self._E, self._a[4], anklepitch), [0,0,-self._L2]))
        T_0_6 = T_0_5.dot( tf.getTransFromRp(tf.roadriguesEquation(self._E, self._a[5], ankleroll), zero_v))

        return T_0_1, T_0_2, T_0_3, T_0_4, T_0_5, T_0_6
        
        
        
    def forwardKinematics(self, jointPositions, leg):

        T_0_6 = self.getLegTrans(jointPositions, leg)[5]

        return tf.getRotationAndPositionFromT(T_0_6)

    def inverseKinematics(self, p_ref, omega_ref, leg):
        q = self.getJointPositions(leg)
        R, p = self.forwardKinematics(q, leg)
        omega = np.array(tf.getRollPitchYawFromR(R))

        dp = p_ref - p
        domega = omega_ref - omega
        dp_domega = np.append(dp,domega)

        dq = self._lambda * la.inv(self.jacobian(q, leg)).dot(dp_domega)


        return q+dq

    def jacobian(self, q, leg):
        T0 = self.getLegTrans(q, leg)
        zero_v = np.zeros(3)

        R = [tf.getRotationFromT(T0[i]) for i in range(len(T0))]
        p = [tf.getPositionFromT(T0[i]) for i in range(len(T0))]

        wa = [R[i].dot(self._a[i]) for i in range(len(R))]

        Jp = np.vstack((  np.hstack((np.cross( wa[i],(p[5]-p[i])), wa[i])) for i in range(len(wa)-1)))
        J = np.vstack(( Jp, np.hstack((zero_v, wa[5])) )).T

        return J

    def getJointPositions(self, leg):
        if np.sum(leg == self.R) == len(leg):
            jointStates = pb.getJointStates(self._robotId, jointIndices=self._jointIdListR)
            jointPositions = [jointStates[i][0] for i in range(len(jointStates))]

        elif np.sum(leg == self.L) == len(leg):
            jointStates = pb.getJointStates(self._robotId, jointIndices=self._jointIdListL)
            jointPositions = [jointStates[i][0] for i in range(len(jointStates))]
        
        else:
            raise ValueError("invalid parameter")

        return jointPositions

    def positionInitialize(self, startCOMheight=0.45, initialLegRPY=[0,0,0], initializeTime=1.0, initialJointPosRL=[0.0,0.0,-0.44,0.88,-0.44,0.0]):
        initializeStep = np.arange(0,initializeTime/self._timeStep,1)
        initialLegPosR = [0,self.R[1],-startCOMheight]
        initialLegPosL = [0,self.L[1],-startCOMheight]

        for i in initializeStep: 
            self.setLeftLegJointPositions(initialJointPosRL)
            self.setRightLegJointPositions(initialJointPosRL)
            self.resetRobotPositionAndOrientation(position=[0,0,startCOMheight+0.02], orientation=[0,0,0,1])
            self.oneStep()
        
        for i in initializeStep:
            PosR = self.inverseKinematics(initialLegPosR, initialLegRPY, self.R)
            PosL = self.inverseKinematics(initialLegPosL, initialLegRPY, self.L)
            self.setRightLegJointPositions(PosR)
            self.setLeftLegJointPositions(PosL)
            #self.resetRobotPositionAndOrientation(position=[0,0,startCOMheight], orientation=[0,0,0,1])
            self.oneStep()
    
    def disconnect(self):
        pb.disconnect()
