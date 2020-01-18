import numpy as np

class CPG:
    def __init__(self, cpgId, uo=5, ue=1, uf=0, ve=0.0, vf=0.0, dt=0.01, wfe = -1.5, wef = -1.5):


        self.uo = uo
        self.ye = 1
        self.yf = 0
        self.tau = 1
        self.dtau = 6
        self.beta = 1
        self.y = 0.0

        self.wef = wef
        self.wfe = wfe

        self.ue = ue
        self.uf = uf
        self.due = 0.0
        self.duf = 0.0

        self.ve = ve
        self.vf = vf
        self.dve = 0.0
        self.dvf = 0.0
        self.feede = 0.0
        self.feedf = 0.0

        self.dt = dt
        
    def onestep(self):

        u_matrix = np.array([self.ue, self.uf])

        w_matrix = np.array([[self.wfe, 0], 
                            [0, self.wef]])
        
        y_matrix = np.array([self.ye, self.yf])
        v_matrix = np.array([self.ve, self.vf])
        feed_matrix = np.array([self.feede, self.feedf])



        du = -u_matrix + w_matrix.dot(np.array([y_matrix[1], y_matrix[0]])) -self.beta * v_matrix + self.uo + feed_matrix        

        du = du * (1/self.tau) 

        dv = -v_matrix + y_matrix

        dv = dv * (1/self.dtau)

        u_matrix = u_matrix + du * self.dt
        v_matrix = v_matrix + dv * self.dt


        self.ue, self.uf = u_matrix
        self.ve, self.vf = v_matrix

        self.ye = np.max([self.ue, 0])
        self.yf = np.max([self.uf, 0])

        self.y = self.ye - self.yf
        print(self.y)

        return self.y


if __name__ == "__main__":
    cpg = CPG(cpgId=1)
    for i in range(60000):
        cpg.onestep()


        



