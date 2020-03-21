import pybullet as pb
import pybullet_data
from motor import Motor
import numpy as np
import time
import tform as tf

class Robot:
    def __init__(self, robotPATH, StartPosition, startOrientation, maxForce, controlMode=pb.POSITION_CONTROL, planePATH="plane.urdf"):
        physicsClient = pb.connect(pb.GUI)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.setGravity(0,0,-9.8)
        self._planeId = pb.loadURDF(planePATH)
        self._robotId = pb.loadURDF(robotPATH,StartPosition, pb.getQuaternionFromEuler(startOrientation))
        self._controlMode = controlMode
        self.numJoint = pb.getNumJoints(self._robotId)
        self._jointIdList = [i for i in range(self.numJoint)]

        self.maxForce = maxForce
        self._maxForceList = [maxForce for i in range(12)]

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
        pb.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=130, cameraPitch=-30, cameraTargetPosition=robotPosition)
        pb.stepSimulation()
        time.sleep(self._timeStep)


class Bipedal(Robot):

    def __init__(self, robotPATH="urdf/bipedal.urdf", planePATH="plane.urdf", StartPosition=[0,0,0.53], startOrientation=[0,0,0], maxForce = 9.0, controlMode=pb.POSITION_CONTROL):
        super().__init__(robotPATH, StartPosition, startOrientation, maxForce, planePATH=planePATH)


        self.RYawHipJointMotor = Motor(self._robotId, 0, mode=controlMode)
        self.RRollHipJointMotor = Motor(self._robotId, 1, mode=controlMode)
        self.RPitchHipJointMotor = Motor(self._robotId, 2, mode=controlMode)
        self.RKneeJointMotor = Motor(self._robotId, 3, mode=controlMode)
        self.RPitchAnkleJointMotor = Motor(self._robotId, 4, mode=controlMode)
        self.RRollAnkleJointMotor = Motor(self._robotId, 5, mode=controlMode)

        self.LYawHipJointMotor = Motor(self._robotId, 6, mode=controlMode)
        self.LRollHipJointMotor = Motor(self._robotId, 7, mode=controlMode)
        self.LPitchHipJointMotor = Motor(self._robotId, 8, mode=controlMode)
        self.LKneeJointMotor = Motor(self._robotId, 9, mode=controlMode)
        self.LPitchAnkleJointMotor = Motor(self._robotId, 10, mode=controlMode)
        self.LRollAnkleJointMotor = Motor(self._robotId, 11, mode=controlMode)

        self._rightIdList = [self.RYawHipJointMotor.Id, self.RRollHipJointMotor.Id, self.RPitchHipJointMotor.Id, 
                            self.RKneeJointMotor.Id, self.RPitchAnkleJointMotor.Id, self.RRollAnkleJointMotor.Id]

        self._leftIdList = [self.LYawHipJointMotor.Id, self.LRollHipJointMotor.Id, self.LPitchHipJointMotor.Id, 
                            self.LKneeJointMotor.Id, self.LPitchAnkleJointMotor.Id, self.LRollAnkleJointMotor.Id]

        self.COMtoL = np.array([0,0.065,-0.175])
        self.COMtoR = np.array([0,-0.065,-0.175])


    def setMotorPositionByArray(self, goalPositions):
        pb.setJointMotorControlArray(self._robotId, jointIndices=self._jointIdList, controlMode=self._controlMode, forces=self._maxForceList, targetPositions=goalPositions)

    


    def torqueControlModeEnableForAll(self):

        self.RYawHipJointMotor.torqueControlModeEnable()
        self.RRollHipJointMotor.torqueControlModeEnable()
        self.RPitchHipJointMotor.torqueControlModeEnable()
        self.RKneeJointMotor.torqueControlModeEnable()
        self.RPitchAnkleJointMotor.torqueControlModeEnable()
        self.RRollAnkleJointMotor.torqueControlModeEnable()

        self.LYawHipJointMotor.torqueControlModeEnable()
        self.LRollHipJointMotor.torqueControlModeEnable()
        self.LPitchHipJointMotor.torqueControlModeEnable()
        self.LKneeJointMotor.torqueControlModeEnable()
        self.LPitchAnkleJointMotor.torqueControlModeEnable()
        self.LRollAnkleJointMotor.torqueControlModeEnable()


    def setLegEndEffectorPositionAndOrientation(self, LegIdList, goalPosition, goalOrientation):
        theta_vector = self.calculateIK(LegPosition, LegOrientation)
        self.setMotorPositionByArray(goalPositions=theta_vector, jointIndices=LegIdList)

    def getLegTrans(self, jointPositions):
        hipyaw = jointPositions[0]
        hiproll = jointPositions[1] + np.pi/2
        hippitch = jointPositions[2]
        knee = jointPositions[3]
        anklepitch = jointPositions[4]
        ankleroll = jointPositions[5]


        L1 = 0.18
        L2 = 0.18
        CoMtoL = np.array([0,0.1,-0.15])
        T_w_0 = tf.getTransFromRp(tf.getRotationYaw(np.pi/2),CoMtoL)
        zero_v = np.zeros(3)
        I3 = np.eye(3)

        T_w_0 = tf.getTransFromRp(tf.getRotationYaw(np.pi/2),CoMtoL)
        T_0_1 = tf.getTransFromRp(tf.getRotationYaw(hipyaw),zero_v)
        T_1_2 = tf.getTransFromRp(tf.getRotationRoll(np.pi/2),zero_v).dot(tf.getTransFromRp(tf.getRotationYaw(hiproll),zero_v))
        T_2_3 = tf.getTransFromRp(tf.getRotationRoll(np.pi/2),zero_v).dot(tf.getTransFromRp(tf.getRotationYaw(hippitch),zero_v))
        T_3_4 = tf.getTransFromRp(I3, np.array([-L1,0,0])).dot(tf.getTransFromRp(tf.getRotationYaw(knee),zero_v))
        T_4_5 = tf.getTransFromRp(I3, np.array([-L2,0,0])).dot(tf.getTransFromRp(tf.getRotationYaw(anklepitch),zero_v))
        T_5_6 = tf.getTransFromRp(tf.getRotationRoll(-np.pi/2), zero_v).dot(tf.getTransFromRp(tf.getRotationYaw(ankleroll),zero_v))

        T_w_1 = T_w_0.dot(T_0_1)
        T_w_2 = T_w_1.dot(T_1_2)
        T_w_3 = T_w_2.dot(T_2_3)
        T_w_4 = T_w_3.dot(T_3_4)
        T_w_5 = T_w_4.dot(T_4_5)
        T_w_6 = T_w_5.dot(T_5_6)

        return T_w_0, T_w_1, T_w_2, T_w_3, T_w_4, T_w_5, T_w_6



    def forwardKinematics(self, jointPositions):

        L1 = 0.18
        L2 = 0.18
        T_w_0 = tf.getTransFromRp(tf.getRotationYaw(np.pi/2),self.COMtoL)
        zero_v = np.zeros(3)
        I3 = np.eye(3)

        T_w_6 = getLegTrans(jointPositions)[6]


        return tf.getRotationAndPositionFromT(T_w_6)





    def jacobian(self, q):
        Tw = self.getLegTrans(q)
        a = np.array([0,0,1])
        zero_v = np.zeros(3)

        R = [tf.getRotationFromT(Tw[i]) for i in range(len(Tw))]
        p = [tf.getPositionFromT(Tw[i]) for i in range(len(Tw))]


        wa = [R[i].dot(a) for i in range(len(R))]


        Jp = np.vstack((  np.hstack((np.cross( wa[i],(p[6]-p[i])), wa[i])) for i in range(1,len(wa)-1) ))
        J = np.vstack(( Jp, np.hstack((zero_v, wa[6])) )).T

        return J


    

    def getJointPositions(self):
        jointStates = pb.getJointStates(self._robotId, jointIndices=self._jointIdList)
        jointPositions = [jointStates[i][0] for i in range(self.numJoint)]
        return jointPositions

