import numpy as np
class TwoWheeledRobot():
    def __init__(self, Dw,Lr):
        self.Dw = Dw
        self.Lr = Lr 
    def get_parametres(self):
        return self.Dw, self.Lr
    def kinematic(self, state,control,t):
        y = np.copy(state)
        A = np.array([[ 0, 0,0 ], [0, 0, 0], [0,0,0]])
        B11 = self.Dw/4*np.cos(y[2])
        B12 = B11
        B21 = self.Dw/4*np.sin(y[2])
        B22 = B21
        B31 = self.Dw/(2*self.Lr)
        B32 = -B31
        B = np.array([[B11, B12], [B21, B22], [B31, B32]])
        x = A@y + B@control
        return x


