import pybullet as pb
import pybullet_data
import numpy as np
import time
import tform as tf
import scipy.linalg as la
from inverse_dynamics import InverseDynamics


class IMU:
    def __init__(self, robotId):
        self._robotId = robotId

    def getEularAndPosition(self):
        pos, qua = pb.getBasePositionAndOrientation(self._robotId)
        return pb.getEulerFromQuaternion(qua), pos

    def getLinerAndAngularVelocity(self):
        return pb.getBaseVelocity(self._robotId)


class Robot:
    def __init__(self, timeStep, robotPATH, startPosition, startOrientation, maxForce,
                controlMode=pb.POSITION_CONTROL, planePATH="plane.urdf"):
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

        self._timeStep = timeStep
        self._bodyLinkId = -1


    def getTimeStep(self):
        return self._timeStep

    def getBodyLinkState(self):
        return pb.getLinkState(self._robotId, self._bodyLinkId)

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
        pb.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=180, cameraPitch=-10, cameraTargetPosition=robotPosition)
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
        self.linkInertiaList =  [np.matrix(np.zeros((3,3))),
                                np.matrix([[0.002312157023,0.,0.],
                                                [0., 0.002476174735, 0.],
                                                [0.,0.,0.00028213453]]),

                                np.matrix([[0.000664235357,0.,0.],
                                                [0.,0.000664515268,0.],
                                                [0.,0.,0.000019650311]])]
        self.jointPosVectors = [firstJointOrigin, 
                                         np.zeros(3),
                                         np.array([0,0,-0.18]), 
                                         np.array([0,0,-0.18])]

        self.inverseDynamics = InverseDynamics(jointVectorList=self.jointPosVectors, 
                                                linkInertiaList=self.linkInertiaList,
                                                comVectorList=[np.zeros(3),np.array([0.,0.,-0.033267]),np.array([0.,0.,-0.155989])],
                                                linkMassList=[0.,0.376687,0.140882])


    def getJointPositions(self):
        jointStates = pb.getJointStates(self._robotId, jointIndices=self.IdIndices)
        jointPositions = [jointStates[i][0] for i in range(self.DOF)]

        return jointPositions

    def getJointVelocity(self):
        jointStates = pb.getJointStates(self._robotId, jointIndices=self.IdIndices)
        jointVelocity = [jointStates[i][1] for i in range(self.DOF)]

        return jointVelocity

    def setJointPositions(self,targetPositions):
        pb.setJointMotorControlArray(self._robotId, jointIndices=self.IdIndices, controlMode=self.controlMode, 
                                    forces=self.maxForceList,targetPositions=targetPositions)


    def torqueControlModeEnable(self):
        pb.setJointMotorControlArray(self._robotId, jointIndices=self.IdIndices, controlMode=pb.VELOCITY_CONTROL, forces=[0 for i in range(self.DOF)])
        self.controlMode = pb.TORQUE_CONTROL

    
    def setTorqueArray(self,torque):

        pb.setJointMotorControlArray(self._robotId, jointIndices=self.IdIndices, controlMode=self.controlMode, forces=torque)






class Quadrupedal(Robot):
    def __init__(self, timeStep, robotPATH,initialCoMheight,startPosition=[0,0,0.55], startOrientation=[0.,0.,0.],maxForce=9.0, controlMode=pb.POSITION_CONTROL, planePATH="plane.urdf"):
        super().__init__(timeStep=timeStep,robotPATH=robotPATH, startPosition=startPosition, startOrientation=startOrientation, maxForce=maxForce, controlMode=controlMode, planePATH=planePATH)

        self.L1 = 0.18
        self.L2 = 0.18
        
        self.a = np.array([[1,0,0],  #joint axis matrix [roll, pitch, yaw]
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
                        initialOrientation=pb.getQuaternionFromEuler(startOrientation))

        self.inertiaTensor = np.matrix(np.zeros((3,3)))
        self.inertiaTensor[0,0] = 0.017409405067
        self.inertiaTensor[1,1] = 0.043070296402
        self.inertiaTensor[2,2] = 0.052179256932

        self.imu = IMU(self._robotId)

    def torqueControlModeEnableAll(self):
        self.legRF.torqueControlModeEnable()
        self.legLF.torqueControlModeEnable()
        self.legRH.torqueControlModeEnable()
        self.legLH.torqueControlModeEnable()


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
        #dq = self.jacobi_lambda * la.inv(self.jacobian2(q, targetLeg.jointPosVectors)).dot(dp)

        return q+dq

    def inverseDynamics(self, acceleration_ref,position_ref, velocity_ref,targetLeg):
        jointPositions = targetLeg.getJointPositions()
        jointVelocity = targetLeg.getJointVelocity()
        j = self.jacobian2(theta=jointPositions,linkVector=targetLeg.jointPosVectors)
        dj = self.diffJacobian(theta=jointPositions,diffTheta=jointVelocity,linkVector=targetLeg.jointPosVectors)

        _, x =  self.forwardKinematics(jointPositions=jointPositions,targetLeg=targetLeg)


        tau = targetLeg.inverseDynamics.solve(acceleration_ref=acceleration_ref,position=x,position_ref=position_ref,velocity_ref=velocity_ref,
                                            jointVelocity=jointVelocity,jointPosition=jointPositions,jacobian=j,diffJacobian=dj)
        
        return tau
        
    def jacobian(self, q, targetLeg):
        T_0_E = self.forwardKinematics(q, targetLeg, fullReturn=True)
        zero_v = np.zeros(3)

        R = [tf.getRotationFromT(T_0_E[i]) for i in range(len(T_0_E))]
        p = [tf.getPositionFromT(T_0_E[i]) for i in range(len(T_0_E))]

        wa = [R[i].dot(targetLeg.axisMatrix[i]) for i in range(targetLeg.DOF)]

        J = np.vstack(( np.cross( wa[0],(p[-1]-p[0])), np.cross( wa[1],(p[-1]-p[1])), np.cross( wa[2],(p[-1]-p[2])) )).T

        return J

    def jacobian2(self,theta,linkVector):
        b2 = linkVector[1]
        b3 = linkVector[2]
        b4 = linkVector[3]

        sinq1 = np.sin(theta[0])
        sinq2 = np.sin(theta[1])
        sinq3 = np.sin(theta[2])
        cosq1 = np.cos(theta[0])
        cosq2 = np.cos(theta[1])
        cosq3 = np.cos(theta[2])
        cosq2q3 = np.cos(theta[1]+theta[2])
        sinq2q3 = np.sin(theta[1]+theta[2])
        dxdq1 = 0
        dxdq2 = b4[2]*cosq2q3-b4[0]*sinq2q3+b3[2]*cosq2-b3[0]*sinq2
        dxdq3 = b4[2]*cosq2q3-b4[0]*sinq2q3
        dydq1 = b4[0]*cosq1*sinq2q3+sinq1*(-b2[1]-b3[1]-b4[1])-b2[2]*cosq1-b4[2]*cosq1*cosq2q3-b3[2]*cosq1*cosq2+b3[0]*cosq1*sinq2
        dydq2 = sinq1*(b4[0]*cosq2q3+b4[2]*sinq2q3+b3[0]*cosq2+b3[2]*sinq2)
        dydq3 = sinq1*(b4[0]*cosq2q3+b4[2]*sinq2q3)
        dzdq1 = b4[0]*sinq1*sinq2q3 + b4[2]*sinq1*cosq2q3 + cosq1*(b2[1]+b3[1]+b4[1])+sinq1*(-b2[2]-b3[2]*cosq2+b3[0]*sinq2)
        dzdq2 = -cosq1*(b4[0]*cosq2q3+b4[2]*sinq2q3+b3[0]*cosq2+b3[2]*sinq2)
        dzdq3 = -cosq1*(b4[0]*cosq2q3+b4[2]*sinq2q3)
        j = np.matrix([[dxdq1,dxdq2,dxdq3],
                [dydq1,dydq2,dydq3],
                [dzdq1,dzdq2,dzdq3]])
        return j
        
    def diffJacobian(self,theta,diffTheta,linkVector):
        b2 = linkVector[1]
        b3 = linkVector[2]
        b4 = linkVector[3]
        sinq1 = np.sin(theta[0])
        sinq2 = np.sin(theta[1])
        sinq3 = np.sin(theta[2])
        cosq1 = np.cos(theta[0])
        cosq2 = np.cos(theta[1])
        cosq3 = np.cos(theta[2])
        cosq2q3 = np.cos(theta[1]+theta[2])
        sinq2q3 = np.sin(theta[1]+theta[2])
        dq1 = diffTheta[0]
        dq2 = diffTheta[1]
        dq3 = diffTheta[2]
        diffdxdq1 = 0
        diffdxdq2 = b4[2]*(-sinq2q3*(dq2+dq3))-b4[0]*cosq2q3*(dq2+dq3)-b3[2]*dq2*sinq2-b3[0]*dq2*cosq2
        diffdxdq3 = b4[2]*(-sinq2q3*(dq2+dq3))-b4[0]*cosq2q3*(dq2+dq3)
        diffdydq1 = b4[0]*(-dq1*sinq1*sinq2q3+cosq1*(dq2+dq3)*cosq2q3)+dq1*cosq1*(-b2[1]-b3[1]-b4[1])+b2[2]*dq1*sinq1 \
        -b4[2]*(-dq1*sinq1*cosq2q3-cosq1*(dq2+dq3)*sinq2q3)-b3[2]*(-dq1*sinq1*cosq2-dq2*cosq1*sinq2)+b3[0]*(-dq1*sinq1*sinq2+dq2*cosq1*cosq2) 
        diffdydq2 = dq1*cosq1*(b4[0]*cosq2q3+b4[2]*sinq2q3+b3[0]*cosq2+b3[2]*sinq2)+sinq1*(-b4[0]*(dq2+dq3)*sinq2q3+b4[2]*(dq2+dq3)*cosq2q3-b3[0]*dq2*sinq2+b3[2]*dq2*cosq2)
        diffdydq3 = dq1*cosq1*(b4[0]*cosq2q3+b4[2]*sinq2q3)+sinq1*(-b4[0]*(dq2+dq3)*sinq2q3+b4[2]*(dq2+dq3)*cosq2q3)
        diffdzdq1 = b4[0]*(dq1*cosq1*sinq2q3+sinq1*(dq2+dq3)*cosq2q3) \
                    +b4[2]*(dq1*cosq1*cosq2q3-sinq1*(dq2+dq3)*sinq2q3)-dq1*sinq1*(b2[1]+b3[1]+b4[1]) \
                    +dq1*cosq1*(-b2[2]-b3[2]*cosq2+b3[0]*sinq2)+sinq1*(b3[2]*dq2*sinq2+b3[0]*dq2*cosq2)
        diffdzdq2 = dq1*sinq1*(b4[0]*cosq2q3+b4[2]*sinq2q3+b3[0]*cosq2+b3[2]*sinq2)-cosq1*(-b4[0]*(dq2+dq3)*sinq2q3+b4[2]*(dq2+dq3)*cosq2q3-b3[0]*dq2*sinq2+b3[2]*dq2*cosq2)
        diffdzdq3 = dq1*sinq1*(b4[0]*cosq2q3+b4[2]*sinq2q3)-cosq1*(-b4[0]*(dq2+dq3)*sinq2q3+b4[2]*(dq2+dq3)*cosq2q3)

        dj = np.matrix([[diffdxdq1,diffdxdq2,diffdxdq3],
                    [diffdydq1,diffdydq2,diffdydq3],
                    [diffdzdq1,diffdzdq2,diffdzdq3]])

        return dj



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

    def getTorque(self,index):
        return pb.getJointState(self._robotId,index)[3]

    def getTorqueList(self,indices):
        jointStates = pb.getJointStates(self._robotId, jointIndices=indices)
        l = len(indices)
        jointTorqueList = [jointStates[i][3] for i in range(l)]
        return jointTorqueList


    def disconnect(self):
        pb.disconnect()
