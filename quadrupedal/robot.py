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
        self._jointIdList = [i for i in range(self.numJoint)]

        self.maxForce = maxForce
        self._maxForceList = [maxForce for i in range(self.numJoint)]

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

    def oneStep(self):
        
        robotPosition, _ = pb.getBasePositionAndOrientation(self._robotId)
        pb.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=135, cameraPitch=-10, cameraTargetPosition=robotPosition)
        pb.stepSimulation()
        time.sleep(self._timeStep)

class Leg:
    def __init__(self,robotId,name,firstJointOrigin,DoF,IdIndices,axisMatrix, maxForce=12, maxVelocity=10, mode=pb.POSITION_CONTROL):
        self._robotId = robotId
        self.firstJointOrigin = firstJointOrigin
        self.DOF = DoF
        self.axisMatrix = axisMatrix
        self.name = name
        self.IdIndices = IdIndices
        self.maxVelocity = maxVelocity
        self.maxForceList = [maxForce for i in range(DoF)]
        self.controlMode = mode

    def getJointPositions(self):
        jointStates = pb.getJointStates(self._robotId, jointIndices=self.IdIndices)
        jointPositions = [jointStates[i][0] for i in range(self.DOF)]

        return jointPositions

    def setJointPositions(self,targetPositions):
        pb.setJointMotorControlArray(self._robotId, jointIndices=self.IdIndices, controlMode=self.controlMode, forces=self.maxForceList, targetPositions=targetPositions)





class Quadrupedal(Robot):
    def __init__(self, robotPATH,initialCoMheight,startPosition=[0,0,0.55], startOrientation=[0.,0.,0.], CoMposition_b=np.array([0.,0.,-0.01]),maxForce=9.0, controlMode=pb.POSITION_CONTROL, planePATH="plane.urdf"):
        super().__init__(robotPATH=robotPATH, startPosition=startPosition, startOrientation=startOrientation, maxForce=maxForce, controlMode=controlMode, planePATH=planePATH)

        self.L1 = 0.18
        self.L2 = 0.18
        #joint axis matrix [roll, pitch, yaw]
        self.a = np.array([[1,0,0],
                            [0,1,0],
                            [0,1,0]])

        #RF:Right Forward
        #RH:Right Hind
        #LF:Left Forward
        #LH:Left Hind
        self.legRF = Leg(self._robotId,"RF",np.array([0.2,-0.11,0.]),3,[0,1,2],self.a)
        self.legLF = Leg(self._robotId,"LF",np.array([0.2,0.11,0.]),3,[3,4,5],self.a)
        self.legRH = Leg(self._robotId,"RH",np.array([-0.2,-0.11,0.]),3,[6,7,8],self.a)
        self.legLH = Leg(self._robotId,"LH",np.array([-0.2,0.11,0.]),3,[9,10,11],self.a)

        self.LEG_NUM = 4
        self._E = np.eye(3)
        

        self.jacobi_lambda = 1.

        self.initializer(initialFootPrints=np.array([ np.hstack((self.legLF.firstJointOrigin[0:2], -initialCoMheight)),
                                        np.hstack((self.legLH.firstJointOrigin[0:2], -initialCoMheight)),  
                                        np.hstack((self.legRF.firstJointOrigin[0:2], -initialCoMheight)),
                                        np.hstack((self.legRH.firstJointOrigin[0:2], -initialCoMheight)) ]),
                        initialPosition=startPosition, 
                        initialOrientation=[0.,0.,0.,1.])

    def forwardKinematics(self, jointPositions, targetLeg, fullReturn=False):
        abadJoint = jointPositions[0]
        hipJoint = jointPositions[1]
        kneeJoint = jointPositions[2]

        zero_v = np.zeros(3)

        T_0_1 = tf.getTransFromRp(tf.roadriguesEquation(self._E, targetLeg.axisMatrix[0], abadJoint),targetLeg.firstJointOrigin)
        T_0_2 = T_0_1.dot( tf.getTransFromRp(tf.roadriguesEquation(self._E, targetLeg.axisMatrix[1], hipJoint), zero_v))
        T_0_3 = T_0_2.dot( tf.getTransFromRp(tf.roadriguesEquation(self._E, targetLeg.axisMatrix[2], kneeJoint), [0,0,-self.L1]))
        T_0_4 = T_0_3.dot( tf.getTransFromRp(np.eye(3,3), [0,0,-self.L2]))

        if fullReturn:
            return T_0_1, T_0_2, T_0_3, T_0_4
        else: 
            return tf.getRotationAndPositionFromT(T_0_4)



    def inverseKinematics(self, position_ref, targetLeg):
        q = targetLeg.getJointPositions()
        _, position = self.forwardKinematics(q, targetLeg)

        dp = position_ref - position

        dq = self.jacobi_lambda * la.inv(self.jacobian(q, targetLeg)).dot(dp)

        return q+dq

    def jacobian(self, q, targetLeg):
        T_0_E = self.forwardKinematics(q, targetLeg, fullReturn=True)
        zero_v = np.zeros(3)

        R = [tf.getRotationFromT(T_0_E[i]) for i in range(len(T_0_E))]
        p = [tf.getPositionFromT(T_0_E[i]) for i in range(len(T_0_E))]

        wa = [R[i].dot(targetLeg.axisMatrix[i]) for i in range(targetLeg.DOF)]

        J = np.vstack(( np.cross( wa[0],(p[-1]-p[0])), np.cross( wa[1],(p[-1]-p[1])), np.cross( wa[2],(p[-1]-p[2])) )).T

        return J

    def initializer(self,initialFootPrints,initialPosition, initialOrientation, initialJointPosition=np.array([0.,0.2,-0.4]), initializeTime=1.):

        for i in np.arange(0,initializeTime/self._timeStep,1):
            self.resetRobotPositionAndOrientation(initialPosition, initialOrientation)
            self.legRF.setJointPositions(initialJointPosition)
            self.legLF.setJointPositions(initialJointPosition)
            self.legRH.setJointPositions(initialJointPosition)
            self.legLH.setJointPositions(initialJointPosition)
            self.oneStep()

        for i in np.arange(0,initializeTime/self._timeStep,1):
            posLF = self.inverseKinematics(initialFootPrints[0], self.legLF)
            posLH = self.inverseKinematics(initialFootPrints[1], self.legLH)
            posRF = self.inverseKinematics(initialFootPrints[2], self.legRF)
            posRH = self.inverseKinematics(initialFootPrints[3], self.legRH)
            self.legRF.setJointPositions(posLF)
            self.legLF.setJointPositions(posLH)
            self.legRH.setJointPositions(posRF)
            self.legLH.setJointPositions(posRH)
            self.oneStep()

    def disconnect(self):
        pb.disconnect()
