import numpy as np
import sympy as sy
import scipy.linalg as la
#3DoF Inverse Dynamics
class InverseDynamics:
    def __init__(self,jointVectorList,linkInertiaList, comVectorList,linkMassList, positionGain=900,velocityGain=5):
        #set to matrix
        self.b1=jointVectorList[0]
        self.b2=jointVectorList[1]
        self.b3=jointVectorList[2]
        self.b4=jointVectorList[3]
        self.m = linkMassList
        self.S = comVectorList
        self.Hhat=[]
        self._g = np.matrix([0.,0.,-9.8,0.]).T 
        for i in range(3):
            self.Hhat.append(self.HhatMatrix(Ihat=linkInertiaList[i],S=comVectorList[i],m=linkMassList[i]))

        self.Kp = positionGain
        self.Kv = velocityGain



    def forward(self,jointPosition,jointAcceleration, jointVelocity):
        M = self.M(jointPosition)
        h = self.h(jointPosition, jointVelocity)
        g = self.g(theta)

        return M*jointAcceleration + h + g

    def solve(self,acceleration_ref,position,position_ref,velocity_ref, jointVelocity,jointPosition,jacobian,diffJacobian): #全部リスト
        M = self.M(jointPosition)
        h = self.h(jointPosition, jointVelocity)
        g = self.g(jointPosition)
        x = np.matrix(position).T
        xd = np.matrix(position_ref).T
        vd = np.matrix(velocity_ref).T
        a_ref = np.matrix(acceleration_ref).T
        dtheta = np.matrix(jointVelocity).T

        v= jacobian*dtheta

        #u = a_ref + Kv*(vd-v)+Kp*(xd-x)
        #tau = M * la.inv(jacobian)*(u-diffJacobian*dtheta)+h+g

        feedForwardTau = M * la.inv(jacobian)*(a_ref-diffJacobian*dtheta)+h+g
        feedBackTau = jacobian.T*(self.Kv*(vd-v)+self.Kp*(xd-x))
        tau = feedBackTau + feedForwardTau

        

        return [tau[0,0],tau[1,0],tau[2,0]]



    def M(self,theta):
        M = np.matrix(np.zeros((3,3)))

        q1 = theta[0]
        q2 = theta[1]
        q3 = theta[2]

        dT01dq1 = np.matrix(np.zeros((4,4)))
        dT01dq1[1,1] = -np.sin(q1)
        dT01dq1[1,2] = -np.cos(q1)
        dT01dq1[2,1] = np.cos(q1)
        dT01dq1[2,2] = -np.sin(q1)
        dT02dq1 = np.matrix(np.zeros((4,4)))
        dT02dq1[1,0] = np.cos(q1)*np.sin(q2)
        dT02dq1[1,1] = -np.sin(q1)
        dT02dq1[1,2] = -np.cos(q1)*np.cos(q2)
        dT02dq1[1,3] = -self.b2[2]*np.cos(q1)-self.b2[1]*np.sin(q1)
        dT02dq1[2,0] = np.sin(q1)*np.sin(q2)
        dT02dq1[2,1] = np.cos(q1)
        dT02dq1[2,2] = -np.cos(q2)*np.sin(q1)
        dT02dq1[2,3] = self.b2[1]*np.cos(q1)-self.b2[2]*np.sin(q1)
        

        dT02dq2 = np.matrix(np.zeros((4,4)))
        dT02dq2[0,0] = -np.sin(q2)
        dT02dq2[0,2] = np.cos(q2)
        dT02dq2[1,0] = np.cos(q2)*np.sin(q1)
        dT02dq2[1,2] = np.sin(q1)*np.sin(q2)
        dT02dq2[2,0] = -np.cos(q1)*np.cos(q2)
        dT02dq2[2,2] = -np.cos(q1)*np.sin(q2)
        
        dT03dq1 = np.matrix(np.zeros((4,4)))
        dT03dq1[1,0] = np.sin(q2+q3)*np.cos(q1)
        dT03dq1[1,1] = -np.sin(q1)
        dT03dq1[1,2] = -np.cos(q2+q3)*np.cos(q1)
        dT03dq1[1,3] = self.b3[0]*np.cos(q1)*np.sin(q2) - self.b2[1]*np.sin(q1) - self.b3[1]*np.sin(q1) - self.b3[2]*np.cos(q1)*np.cos(q2) - self.b2[2]*np.cos(q1)
        dT03dq1[2,0] = np.sin(q2+q3)*np.sin(q1)
        dT03dq1[2,1] = np.cos(q1)
        dT03dq1[2,2] = -np.cos(q2+q3)*np.sin(q1)
        dT03dq1[2,3] = self.b2[1]*np.cos(q1) - self.b3[1]*np.cos(q1) - self.b2[2]*np.sin(q1) - self.b3[2]*np.cos(q2)*np.sin(q1) + self.b3[0]*np.sin(q1)*np.sin(q2)

        dT03dq2 = np.matrix(np.zeros((4,4)))
        dT03dq2[0,0] = -np.sin(q2+q3)
        dT03dq2[0,2] = np.cos(q2+q3)
        dT03dq2[0,3] = self.b3[2]*np.cos(q2)-self.b3[0]*np.sin(q2)
        dT03dq2[1,0] = np.cos(q2+q3)*np.sin(q1)
        dT03dq2[1,2] = np.sin(q2+q3)*np.sin(q1)
        dT03dq2[1,3] = np.sin(q1)*(self.b3[0]*np.cos(q2)+self.b3[2]*np.sin(q2))
        dT03dq2[2,0] = -np.cos(q2+q3)*np.cos(q1)
        dT03dq2[2,2] = -np.sin(q2+q3)*np.cos(q1)
        dT03dq2[2,3] = -np.cos(q1)*(self.b3[0]*np.cos(q2)+self.b3[2]*np.sin(q2))

        dT03dq3 = np.matrix(np.zeros((4,4)))
        dT03dq3[0,0] = -np.sin(q2+q3)
        dT03dq3[0,2] = np.cos(q2+q3)
        dT03dq3[1,0] = np.cos(q2+q3)*np.sin(q1)
        dT03dq3[1,2] = np.sin(q2+q3)*np.sin(q1)
        dT03dq3[2,0] = -np.cos(q2+q3)*np.cos(q1)
        dT03dq3[2,2] = -np.sin(q2+q3)*np.cos(q1)


        M[0,0] = np.trace(dT01dq1 * self.Hhat[0] * dT01dq1.T) + np.trace(dT02dq1 * self.Hhat[1] * dT02dq1.T) + np.trace(dT03dq1 * self.Hhat[2] * dT03dq1.T) #k=1 i=1,j=1
        M[1,1] = np.trace(dT02dq2 * self.Hhat[1] * dT02dq2.T) + np.trace(dT03dq2 * self.Hhat[2] * dT03dq2.T) 
        M[2,2] = np.trace(dT03dq3 * self.Hhat[2] * dT03dq3.T) 

        M[0,1] = M[1,0] = np.trace(dT02dq2 * self.Hhat[1] * dT02dq1.T) + np.trace(dT03dq2 * self.Hhat[2] * dT03dq1.T) 
        M[0,2] = M[2,0] = np.trace(dT03dq3 * self.Hhat[2] * dT03dq1.T) 
        M[1,2] = M[2,1] = np.trace(dT03dq3 * self.Hhat[2] * dT03dq2.T)

        return M

    def h(self,theta,dtheta):
        h = np.array([0.,0.,0.])

        q1 = theta[0]
        q2 = theta[1]
        q3 = theta[2]

        dT01dq1 = np.matrix(np.zeros((4,4)))
        dT01dq1[1,1] = -np.sin(q1)
        dT01dq1[1,2] = -np.cos(q1)
        dT01dq1[2,1] = np.cos(q1)
        dT01dq1[2,2] = -np.sin(q1)
        dT02dq1 = np.matrix(np.zeros((4,4)))
        dT02dq1[1,0] = np.cos(q1)*np.sin(q2)
        dT02dq1[1,1] = -np.sin(q1)
        dT02dq1[1,2] = -np.cos(q1)*np.cos(q2)
        dT02dq1[1,3] = -self.b2[2]*np.cos(q1)-self.b2[1]*np.sin(q1)
        dT02dq1[2,0] = np.sin(q1)*np.sin(q2)
        dT02dq1[2,1] = np.cos(q1)
        dT02dq1[2,2] = -np.cos(q2)*np.sin(q1)
        dT02dq1[2,3] = self.b2[1]*np.cos(q1)-self.b2[2]*np.sin(q1)
        

        dT02dq2 = np.matrix(np.zeros((4,4)))
        dT02dq2[0,0] = -np.sin(q2)
        dT02dq2[0,2] = np.cos(q2)
        dT02dq2[1,0] = np.cos(q2)*np.sin(q1)
        dT02dq2[1,2] = np.sin(q1)*np.sin(q2)
        dT02dq2[2,0] = -np.cos(q1)*np.cos(q2)
        dT02dq2[2,2] = -np.cos(q1)*np.sin(q2)
        
        dT03dq1 = np.matrix(np.zeros((4,4)))
        dT03dq1[1,0] = np.sin(q2+q3)*np.cos(q1)
        dT03dq1[1,1] = -np.sin(q1)
        dT03dq1[1,2] = -np.cos(q2+q3)*np.cos(q1)
        dT03dq1[1,3] = self.b3[0]*np.cos(q1)*np.sin(q2) - self.b2[1]*np.sin(q1) - self.b3[1]*np.sin(q1) - self.b3[2]*np.cos(q1)*np.cos(q2) - self.b2[2]*np.cos(q1)
        dT03dq1[2,0] = np.sin(q2+q3)*np.sin(q1)
        dT03dq1[2,1] = np.cos(q1)
        dT03dq1[2,2] = -np.cos(q2+q3)*np.sin(q1)
        dT03dq1[2,3] = self.b2[1]*np.cos(q1) - self.b3[1]*np.cos(q1) - self.b2[2]*np.sin(q1) - self.b3[2]*np.cos(q2)*np.sin(q1) + self.b3[0]*np.sin(q1)*np.sin(q2)

        dT03dq2 = np.matrix(np.zeros((4,4)))
        dT03dq2[0,0] = -np.sin(q2+q3)
        dT03dq2[0,2] = np.cos(q2+q3)
        dT03dq2[0,3] = self.b3[2]*np.cos(q2)-self.b3[0]*np.sin(q2)
        dT03dq2[1,0] = np.cos(q2+q3)*np.sin(q1)
        dT03dq2[1,2] = np.sin(q2+q3)*np.sin(q1)
        dT03dq2[1,3] = np.sin(q1)*(self.b3[0]*np.cos(q2)+self.b3[2]*np.sin(q2))
        dT03dq2[2,0] = -np.cos(q2+q3)*np.cos(q1)
        dT03dq2[2,2] = -np.sin(q2+q3)*np.cos(q1)
        dT03dq2[2,3] = -np.cos(q1)*(self.b3[0]*np.cos(q2)+self.b3[2]*np.sin(q2))

        dT03dq3 = np.matrix(np.zeros((4,4)))
        dT03dq3[0,0] = -np.sin(q2+q3)
        dT03dq3[0,2] = np.cos(q2+q3)
        dT03dq3[1,0] = np.cos(q2+q3)*np.sin(q1)
        dT03dq3[1,2] = np.sin(q2+q3)*np.sin(q1)
        dT03dq3[2,0] = -np.cos(q2+q3)*np.cos(q1)
        dT03dq3[2,2] = -np.sin(q2+q3)*np.cos(q1)


        dT01dq1q1 = np.matrix(np.zeros((4,4)))
        dT01dq1[1,1] = -np.cos(q1)
        dT01dq1[1,2] = np.sin(q1)
        dT01dq1[2,1] = -np.sin(q1)
        dT01dq1[2,2] = -np.cos(q1)


        dT02dq1q1 = np.matrix(np.zeros((4,4)))
        dT02dq1[1,0] = -np.sin(q1)*np.sin(q2)
        dT02dq1[1,1] = -np.cos(q1)
        dT02dq1[1,2] = np.cos(q2)*np.sin(q1)
        dT02dq1[1,3] = self.b2[2]*np.sin(q1)-self.b2[1]*np.cos(q1)
        dT02dq1[2,0] = np.cos(q1)*np.sin(q2)
        dT02dq1[2,1] = -np.sin(q1)
        dT02dq1[2,2] = -np.cos(q1)*np.cos(q2)
        dT02dq1[2,3] = -self.b2[2]*np.cos(q1)-self.b2[1]*np.sin(q1)


        dT02dq1q2 = dt02dq2q1 =np.matrix(np.zeros((4,4)))
        dT02dq1q2[1,0] = np.cos(q1)*np.cos(q2)
        dT02dq1q2[1,2] = np.cos(q1)*np.sin(q2)
        dT02dq1q2[2,0] = np.cos(q2)*np.sin(q1)
        dT02dq1q2[2,2] = np.sin(q1)*np.sin(q2)

        dT02dq2q2 = np.matrix(np.zeros((4,4)))
        dT02dq2q2[0,0] = -np.cos(q2)
        dT02dq2q2[0,2] = -np.sin(q2)
        dT02dq2q2[1,0] = -np.sin(q1)*np.sin(q2)
        dT02dq2q2[1,2] = np.cos(q2)*np.sin(q1)
        dT02dq2q2[2,0] = np.cos(q1)*np.sin(q2)
        dT02dq2q2[2,2] = -np.cos(q1)*np.cos(q2)

        dT03dq1q1 = np.matrix(np.zeros((4,4)))
        dT03dq1[1,0] = -np.sin(q2+q3)*np.sin(q1)
        dT03dq1[1,1] = -np.cos(q1)
        dT03dq1[1,2] = np.cos(q2+q3)*np.sin(q1)
        dT03dq1[1,3] = self.b2[2]*np.sin(q1) - self.b3[1]*np.cos(q1) - self.b2[1]*np.cos(q1) + self.b3[2]*np.cos(q2)*np.sin(q1) - self.b3[0]*np.sin(q1)*np.sin(q2)
        dT03dq1[2,0] = np.sin(q2+q3)*np.cos(q1)
        dT03dq1[2,1] = -np.sin(q1)
        dT03dq1[2,2] = -np.cos(q2+q3)*np.cos(q1)
        dT03dq1[2,3] = self.b3[0]*np.cos(q1)*np.sin(q2) - self.b2[1]*np.sin(q1) - self.b3[1]*np.sin(q1) - self.b3[2]*np.cos(q1)*np.cos(q2) - self.b2[2]*np.cos(q1)

        dT03dq1q2 = dt03dq2q1 = np.matrix(np.zeros((4,4)))
        dT03dq1q2[1,0] = np.cos(q2+q3)*np.cos(q1)
        dT03dq1q2[1,2] = np.sin(q2+q3)*np.cos(q1)
        dT03dq1q2[1,3] = np.cos(q1)*(self.b3[0]*np.cos(q2)+self.b3[2]*np.sin(q2))
        dT03dq1q2[2,0] = np.cos(q2+q3)*np.sin(q1)
        dT03dq1q2[2,2] = np.sin(q2+q3)*np.sin(q1)
        dT03dq1q2[2,3] = np.sin(q1)*(self.b3[0]*np.cos(q2)+self.b3[2]*np.sin(q2))

        dT03dq2q2 = np.matrix(np.zeros((4,4)))
        dT03dq2q2[0,0] = -np.cos(q2+q3)
        dT03dq2q2[0,2] = -np.sin(q2+q3)
        dT03dq2q2[0,3] = -self.b3[0]*np.cos(q2) - self.b3[2]*np.sin(q2)
        dT03dq2q2[1,0] = -np.sin(q2+q3)*np.sin(q1)
        dT03dq2q2[1,2] = np.cos(q2+q3)*np.sin(q1)
        dT03dq2q2[1,3] = np.sin(q1)*(self.b3[2]*np.cos(q2)-self.b3[0]*np.sin(q2))
        dT03dq2q2[2,0] = np.sin(q2+q3)*np.cos(q1)
        dT03dq2q2[2,2] = -np.cos(q2+q3)*np.cos(q1)
        dT03dq2q2[2,3] = -np.cos(q1)*(self.b3[2]*np.cos(q2)-self.b3[0]*np.sin(q2))

        dT03dq3q2 = dt03dq2q3 = np.matrix(np.zeros((4,4)))
        dT03dq3q2[0,0] = -np.cos(q2+q3) #un
        dT03dq3q2[0,2] = -np.sin(q2+q3)
        dT03dq3q2[1,0] = -np.sin(q2+q3)*np.sin(q1)
        dT03dq3q2[1,2] = np.cos(q2+q3)*np.sin(q1)
        dT03dq3q2[2,0] = np.sin(q2+q3)*np.cos(q1)
        dT03dq3q2[2,2] = -np.cos(q2+q3)*np.cos(q1)

        dT03dq3q3 = np.matrix(np.zeros((4,4)))
        dT03dq3q3[0,0] = -np.cos(q2+q3)
        dT03dq3q3[0,2] = -np.sin(q2+q3)
        dT03dq3q3[1,0] = -np.sin(q2+q3)*np.sin(q1)
        dT03dq3q3[1,2] = np.cos(q2+q3)*np.sin(q1)
        dT03dq3q3[2,0] = np.sin(q2+q3)*np.cos(q1)
        dT03dq3q3[2,2] = -np.cos(q2+q3)*np.cos(q1)


        dT03dq3q1 = dt03dq1q3 = np.matrix(np.zeros((4,4)))
        dT03dq3q3[1,0] = np.cos(q2+q3)*np.cos(q1)
        dT03dq3q3[1,2] = np.sin(q2+q3)*np.cos(q1)
        dT03dq3q3[2,0] = np.cos(q2+q3)*np.sin(q1)
        dT03dq3q3[2,2] = np.sin(q2+q3)*np.sin(q1)


        h[0] = np.trace(dT01dq1q1 * self.Hhat[0] * dT01dq1.T)*dtheta[0]*dtheta[0] \
                +np.trace(dT02dq1q1 * self.Hhat[1] * dT02dq1.T)*dtheta[0]*dtheta[0] \
                +np.trace(dT03dq1q1 * self.Hhat[2] * dT03dq1.T)*dtheta[0]*dtheta[0] \
                +np.trace(dT02dq1q2 * self.Hhat[1] * dT02dq1.T) * dtheta[0]*dtheta[1] \
                +np.trace(dT03dq1q2 * self.Hhat[2] * dT03dq1.T) * dtheta[0]*dtheta[1] \
                +np.trace(dt03dq1q3 * self.Hhat[2] * dT03dq1.T) * dtheta[0]*dtheta[2] \
                +np.trace(dt02dq2q1 * self.Hhat[1] * dT02dq1.T)*dtheta[1]*dtheta[0] \
                +np.trace(dt03dq2q1 * self.Hhat[2] * dT03dq1.T)*dtheta[1]*dtheta[0] \
                +np.trace(dT02dq2q2 * self.Hhat[1] * dT02dq1.T)*dtheta[1]*dtheta[1] \
                +np.trace(dT03dq2q2 * self.Hhat[2] * dT03dq1.T)*dtheta[1]*dtheta[1] \
                +np.trace(dt03dq2q3 * self.Hhat[2] * dT03dq1.T) * dtheta[1]*dtheta[2] \
                +np.trace(dT03dq3q1 * self.Hhat[2] * dT03dq1.T)*dtheta[2]*dtheta[0] \
                +np.trace(dT03dq3q2 * self.Hhat[2] * dT03dq1.T)*dtheta[2]*dtheta[1] \
                +np.trace(dT03dq3q3 * self.Hhat[2] * dT03dq1.T)*dtheta[2]*dtheta[2] 

        h[1] = +np.trace(dT02dq1q1 * self.Hhat[1] * dT02dq2.T)*dtheta[0]*dtheta[0] \
                +np.trace(dT03dq1q1 * self.Hhat[2] * dT03dq2.T)*dtheta[0]*dtheta[0] \
                +np.trace(dT02dq1q2 * self.Hhat[1] * dT02dq2.T) * dtheta[0]*dtheta[1] \
                +np.trace(dT03dq1q2 * self.Hhat[2] * dT03dq2.T) * dtheta[0]*dtheta[1] \
                +np.trace(dt03dq1q3 * self.Hhat[2] * dT03dq2.T) * dtheta[0]*dtheta[2]      \
                +np.trace(dt02dq2q1 * self.Hhat[1] * dT02dq2.T)*dtheta[1]*dtheta[0] \
                +np.trace(dt03dq2q1 * self.Hhat[2] * dT03dq2.T)*dtheta[1]*dtheta[0] \
                +np.trace(dT02dq2q2 * self.Hhat[1] * dT02dq2.T)*dtheta[1]*dtheta[1] \
                +np.trace(dT03dq2q2 * self.Hhat[2] * dT03dq2.T)*dtheta[1]*dtheta[1] \
                +np.trace(dt03dq2q3 * self.Hhat[2] * dT03dq2.T) * dtheta[1]*dtheta[2] \
                +np.trace(dT03dq3q1 * self.Hhat[2] * dT03dq2.T)*dtheta[2]*dtheta[0] \
                +np.trace(dT03dq3q2 * self.Hhat[2] * dT03dq2.T)*dtheta[2]*dtheta[1] \
                +np.trace(dT03dq3q3 * self.Hhat[2] * dT03dq2.T)*dtheta[2]*dtheta[2] 

        h[2] = np.trace(dT03dq1q1 * self.Hhat[2] * dT03dq3.T)*dtheta[0]*dtheta[0] \
                +np.trace(dT03dq1q2 * self.Hhat[2] * dT03dq3.T) * dtheta[0]*dtheta[1] \
                +np.trace(dt03dq1q3 * self.Hhat[2] * dT03dq3.T) * dtheta[0]*dtheta[2] \
                +np.trace(dt03dq2q1 * self.Hhat[2] * dT03dq3.T)*dtheta[1]*dtheta[0] \
                +np.trace(dT03dq2q2 * self.Hhat[2] * dT03dq3.T)*dtheta[1]*dtheta[1] \
                +np.trace(dt03dq2q3 * self.Hhat[2] * dT03dq3.T) * dtheta[1]*dtheta[2] \
                +np.trace(dT03dq3q1 * self.Hhat[2] * dT03dq3.T)*dtheta[2]*dtheta[0] \
                +np.trace(dT03dq3q2 * self.Hhat[2] * dT03dq3.T)*dtheta[2]*dtheta[1] \
                +np.trace(dT03dq3q3 * self.Hhat[2] * dT03dq3.T)*dtheta[2]*dtheta[2] 

        return np.matrix(h).T

    def g(self,theta):
        q1 = theta[0]
        q2 = theta[1]
        q3 = theta[2]

        dT01dq1 = np.matrix(np.zeros((4,4)))
        dT01dq1[1,1] = -np.sin(q1)
        dT01dq1[1,2] = -np.cos(q1)
        dT01dq1[2,1] = np.cos(q1)
        dT01dq1[2,2] = -np.sin(q1)
        dT02dq1 = np.matrix(np.zeros((4,4)))
        dT02dq1[1,0] = np.cos(q1)*np.sin(q2)
        dT02dq1[1,1] = -np.sin(q1)
        dT02dq1[1,2] = -np.cos(q1)*np.cos(q2)
        dT02dq1[1,3] = -self.b2[2]*np.cos(q1)-self.b2[1]*np.sin(q1)
        dT02dq1[2,0] = np.sin(q1)*np.sin(q2)
        dT02dq1[2,1] = np.cos(q1)
        dT02dq1[2,2] = -np.cos(q2)*np.sin(q1)
        dT02dq1[2,3] = self.b2[1]*np.cos(q1)-self.b2[2]*np.sin(q1)
        

        dT02dq2 = np.matrix(np.zeros((4,4)))
        dT02dq2[0,0] = -np.sin(q2)
        dT02dq2[0,2] = np.cos(q2)
        dT02dq2[1,0] = np.cos(q2)*np.sin(q1)
        dT02dq2[1,2] = np.sin(q1)*np.sin(q2)
        dT02dq2[2,0] = -np.cos(q1)*np.cos(q2)
        dT02dq2[2,2] = -np.cos(q1)*np.sin(q2)
        
        dT03dq1 = np.matrix(np.zeros((4,4)))
        dT03dq1[1,0] = np.sin(q2+q3)*np.cos(q1)
        dT03dq1[1,1] = -np.sin(q1)
        dT03dq1[1,2] = -np.cos(q2+q3)*np.cos(q1)
        dT03dq1[1,3] = self.b3[0]*np.cos(q1)*np.sin(q2) - self.b2[1]*np.sin(q1) - self.b3[1]*np.sin(q1) - self.b3[2]*np.cos(q1)*np.cos(q2) - self.b2[2]*np.cos(q1)
        dT03dq1[2,0] = np.sin(q2+q3)*np.sin(q1)
        dT03dq1[2,1] = np.cos(q1)
        dT03dq1[2,2] = -np.cos(q2+q3)*np.sin(q1)
        dT03dq1[2,3] = self.b2[1]*np.cos(q1) - self.b3[1]*np.cos(q1) - self.b2[2]*np.sin(q1) - self.b3[2]*np.cos(q2)*np.sin(q1) + self.b3[0]*np.sin(q1)*np.sin(q2)

        dT03dq2 = np.matrix(np.zeros((4,4)))
        dT03dq2[0,0] = -np.sin(q2+q3)
        dT03dq2[0,2] = np.cos(q2+q3)
        dT03dq2[0,3] = self.b3[2]*np.cos(q2)-self.b3[0]*np.sin(q2)
        dT03dq2[1,0] = np.cos(q2+q3)*np.sin(q1)
        dT03dq2[1,2] = np.sin(q2+q3)*np.sin(q1)
        dT03dq2[1,3] = np.sin(q1)*(self.b3[0]*np.cos(q2)+self.b3[2]*np.sin(q2))
        dT03dq2[2,0] = -np.cos(q2+q3)*np.cos(q1)
        dT03dq2[2,2] = -np.sin(q2+q3)*np.cos(q1)
        dT03dq2[2,3] = -np.cos(q1)*(self.b3[0]*np.cos(q2)+self.b3[2]*np.sin(q2))

        dT03dq3 = np.matrix(np.zeros((4,4)))
        dT03dq3[0,0] = -np.sin(q2+q3)
        dT03dq3[0,2] = np.cos(q2+q3)
        dT03dq3[1,0] = np.cos(q2+q3)*np.sin(q1)
        dT03dq3[1,2] = np.sin(q2+q3)*np.sin(q1)
        dT03dq3[2,0] = -np.cos(q2+q3)*np.cos(q1)
        dT03dq3[2,2] = -np.sin(q2+q3)*np.cos(q1)

        g1 = -self.m[0] * self._g.T * dT01dq1 * np.matrix(np.hstack((self.S[0],1.))).T -self.m[1] * self._g.T * dT02dq1 * np.matrix(np.hstack((self.S[1],1.))).T -self.m[2] * self._g.T * dT03dq1 * np.matrix(np.hstack((self.S[2],1.))).T
        g2 =  -self.m[1] * self._g.T * dT02dq2 * np.matrix(np.hstack((self.S[1],1.))).T -self.m[2] * self._g.T * dT03dq2 * np.matrix(np.hstack((self.S[2],1.))).T
        g3 = -self.m[2] * self._g.T * dT03dq3 * np.matrix(np.hstack((self.S[2],1.))).T

        return np.matrix(np.array([g1,g2,g3])).T



    def HhatMatrix(self,Ihat, S, m):
        Hhat = np.matrix(np.zeros((4,4)))
        Hhat[0:3,0:3] = -Ihat
        Hhat[0,0] = (-Ihat[0,0]+Ihat[1,1]+Ihat[2,2])/2
        Hhat[1,1] = (Ihat[0,0]-Ihat[1,1]+Ihat[2,2])/2
        Hhat[2,2] = (Ihat[0,0]+Ihat[1,1]-Ihat[2,2])/2
        Hhat[0,3] = Hhat[3,0] = m * S[0]
        Hhat[1,3] = Hhat[3,1] = m * S[1]
        Hhat[2,3] = Hhat[3,2] = m * S[2]
        Hhat[3,3] = m

        return Hhat
