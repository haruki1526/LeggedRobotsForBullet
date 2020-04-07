import numpy as np
import tform as tf
import scipy.linalg as la
import control
import swing_trajectory as st

class PreviewControl:
    def __init__(self, dt=1./240., Tsup_time=0.5, Tdl_time=0.1, CoMheight=0.45, g=9.8, previewStepNum=240, stride=0.1, initialTargetZMP=np.array([0.,0.]), initialFootPrint=np.array([[[0.,0.065],[0.,-0.065]]]), R=np.matrix([1.]), Q=np.matrix([[7000,0,0,0],
                                                                                                                                                                                                                                                                [0,1,0,0],
                                                                                                                                                                                                                                                                [0,0,1,0],
                                                                                                                                                                                                                                                                [0,0,0,1]])):
        self._RIGHT_LEG = 1
        self._LEFT_LEG = 0
        self.dt = dt
        self.previewStepNum = previewStepNum
        self.A = np.matrix([[1, dt, (dt**2)/2],
                    [0, 1, dt],
                    [0, 0, 1]])
        self.B = np.matrix([(dt**3)/6, (dt**2)/2, dt]).T
        self.C = np.matrix([1, 0, -CoMheight/g])
        self.CoMheight = CoMheight

        self.G = np.vstack((-self.C*self.B, self.B))
        self.Gr= np.matrix([1., 0., 0., 0.]).T
        #state vector
        self.x = np.matrix(np.zeros(3)).T
        self.y = np.matrix(np.zeros(3)).T

        self.footPrints = np.array([[[0.,0.065],[0.,-0.065]],
                                        [[0.,0.065],[0.,-0.065]],
                                        [[0.,0.065],[0.,-0.065]]])
        self.Tsup = int(Tsup_time/dt)
        self.Tdl = int(Tdl_time/dt)

        self.px_ref = np.full((self.Tsup+self.Tdl)*3,initialTargetZMP[0])
        self.py_ref = np.full((self.Tsup+self.Tdl)*3,initialTargetZMP[1])
        self.px = np.array([0.0]) #zmp
        self.py = np.array([0.0])

        self.phi = np.hstack( (np.matrix([1,0,0,0]).T,  np.vstack((-self.C*self.A, self.A))  ) )
        P, _, _ = control.dare(self.phi,self.G,Q,R)
        zai = (np.eye(4) - self.G * la.inv(R + self.G.T*P*self.G) * self.G.T * P )*self.phi
        self.Fr=np.array([])
        for j in range(1,previewStepNum+1):
            self.Fr= np.append(self.Fr, -la.inv(R + self.G.T*P*self.G)*self.G.T*((zai.T)**(j-1))*P*self.Gr)
        
        self.F=-la.inv(R + self.G.T*P*self.G)*self.G.T*P*self.phi

        self.px_ref_log = self.px_ref[:(self.Tsup+self.Tdl)*2]
        self.py_ref_log = self.py_ref[:(self.Tsup+self.Tdl)*2]

        self.xdu = 0
        self.ydu = 0

        self.xu = 0
        self.yu = 0

        self.dx=np.matrix(np.zeros(3)).T
        self.dy=np.matrix(np.zeros(3)).T


        self.swingLeg = self._RIGHT_LEG
        self.supportLeg = self._LEFT_LEG

        self.targetZMPold = np.array([initialTargetZMP])

        self.currentFootStep = 0



    def footPrintAndCOMtrajectoryGenerator(self, inputTargetZMP,inputFootPrint):
        currentFootStep = 0

        self.footPrints = self.footOneStep(self.footPrints,inputFootPrint, self.supportLeg)

        input_px_ref, input_py_ref = self.targetZMPgenerator(inputTargetZMP, self.targetZMPold[-1], self.Tsup,self.Tdl)


        self.px_ref = self.fifo(self.px_ref, input_px_ref, len(input_px_ref))
        self.py_ref = self.fifo(self.py_ref, input_py_ref, len(input_py_ref))

        self.px_ref_log = np.append(self.px_ref_log, input_px_ref)
        self.py_ref_log = np.append(self.py_ref_log, input_py_ref)

        CoMTrajectory = np.empty((0,3), float)
        startRobotVelocity = np.array([self.x[1],self.y[1]])
        for k in range(len(input_px_ref)):
            dpx_ref = self.px_ref[k+1] - self.px_ref[k]
            dpy_ref = self.py_ref[k+1] - self.py_ref[k]

            xe = self.px_ref[k] - self.C * self.x
            ye = self.py_ref[k] - self.C * self.y

            X=self.phi * np.vstack((xe, self.dx)) + self.G*self.xdu + self.Gr*dpx_ref
            Y=self.phi * np.vstack((ye, self.dy)) + self.G*self.ydu + self.Gr*dpy_ref

            xsum=ysum=0
            for j in range(1,self.previewStepNum+1):
                xsum +=self.Fr[j-1]*(self.px_ref[k+j]-self.px_ref[k+j-1])
                ysum +=self.Fr[j-1]*(self.py_ref[k+j]-self.py_ref[k+j-1])
        
            self.xdu=self.F*X+xsum
            self.ydu=self.F*Y+ysum
            
            self.xu+=self.xdu
            self.yu+=self.ydu
        
            old_x=self.x
            old_y=self.y

            self.x=self.A*self.x+self.B*self.xu
            self.y=self.A*self.y+self.B*self.yu

            self.dx=self.x-old_x
            self.dy=self.y-old_y

            CoMTrajectory = np.vstack((CoMTrajectory, [self.x[0,0], self.y[0,0], self.CoMheight]))
        
            self.px = np.append(self.px, self.C*self.x)
            self.py = np.append(self.py, self.C*self.y) 

        robotEndVelocity = np.array([self.x[1],self.y[1],0.])

        leftTrj,rightTrj = self.footTrajectoryGenerator(np.hstack((self.footPrints[currentFootStep,self.swingLeg], 0.)),
                                                        np.hstack((self.footPrints[currentFootStep+1,self.swingLeg], 0.)),
                                                        np.array([0.,0.,0.]),
                                                        np.array([0.,0.,0.]),
                                                        np.hstack((self.footPrints[currentFootStep,self.supportLeg],0.)),
                                                        self.swingLeg)

        
        self.swingLeg, self.supportLeg = self.changeSupportLeg(self.swingLeg, self.supportLeg)
        self.targetZMPold = np.vstack((self.targetZMPold, inputTargetZMP))
        

        return CoMTrajectory, leftTrj, rightTrj

    def targetZMPgenerator(self,targetZMP,targetZMPold, Tsup, Tdl):
        tdl_t = np.arange(0,Tdl)
        x_a = (targetZMPold[0]-targetZMP[0])/(0-Tdl)
        x_b = targetZMPold[0]
        y_a = (targetZMPold[1]-targetZMP[1])/(0-Tdl)
        y_b = targetZMPold[1]

        px_ref = np.hstack(( x_a * tdl_t + x_b, np.full(Tsup, targetZMP[0])  ))
        py_ref = np.hstack(( y_a * tdl_t + y_b, np.full(Tsup, targetZMP[1])  ))
        
        return px_ref, py_ref

    def footTrajectoryGenerator(self,swingStartPointV,swingEndPointV, startRobotVelocityV_xy,endRobotVelocityV,supportPointV,swingLeg,zheight=0.04):
        supportTrajectory = np.vstack((np.full(self.Tdl+self.Tsup,supportPointV[0]),
                                    np.full(self.Tdl+self.Tsup,supportPointV[1]),
                                    np.full(self.Tdl+self.Tsup,supportPointV[2]))).T

        swingTrajectoryForTdl = np.vstack((np.full(self.Tdl,swingStartPointV[0]),
                                            np.full(self.Tdl,swingStartPointV[1]),
                                            np.full(self.Tdl,swingStartPointV[2]))).T

        if np.array_equal(swingStartPointV, swingEndPointV):
            swingTrajectoryForTsup = np.vstack((np.full(self.Tsup,swingEndPointV[0]),
                                            np.full(self.Tsup,swingEndPointV[1]),
                                            np.full(self.Tsup,swingEndPointV[2]))).T
        
        else:
            swingTrajectoryForTsup = st.swingTrajectoryGenerator(swingStartPointV, swingEndPointV, -startRobotVelocityV_xy, -endRobotVelocityV ,zheight, 0.,self.Tsup*self.dt,self.dt) 

        if swingLeg is self._RIGHT_LEG:
            trjR = np.vstack((swingTrajectoryForTdl,swingTrajectoryForTsup))
            trjL = supportTrajectory
        elif swingLeg is self._LEFT_LEG:
            trjL = np.vstack((swingTrajectoryForTdl,swingTrajectoryForTsup))
            trjR = supportTrajectory

        return trjL, trjR

    def fifo(self, p, in_p, range, vstack=False):
        if vstack:
            return np.vstack(( np.delete(p, range, 0), in_p ))

        else:
            return np.append( np.delete(p, slice(range), None), in_p )

    def footOneStep(self,footPrints,supportPoint,supportLeg):
        step = len(footPrints)
        if supportLeg is self._LEFT_LEG:
            newFootPrint = np.vstack((footPrints, [np.vstack((supportPoint,footPrints[-1,1]))] ))

        elif supportLeg is self._RIGHT_LEG:
            newFootPrint = np.vstack((footPrints, [np.vstack((footPrints[-1,0], supportPoint))] ))

        return np.delete(newFootPrint, 0, 0)
    

    def changeSupportLeg(self, swingLeg, supportLeg):
        return supportLeg, swingLeg
