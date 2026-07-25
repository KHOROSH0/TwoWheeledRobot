from Modeling import Solver
from Kinematic import TwoWheeledRobot
import numpy as np
import matplotlib.pyplot as p
def main():
    #constants for system
    Dw = 0.05 # shape of wheel
    Lr = 0.4 # distance from center of robot to wheel
    # statr conditon
    state0 = np.array([1,1,np.pi/4])
    # time for modeling
    t_span = [0, 500]
    control = np.array([0.5, 0.1])
    system = TwoWheeledRobot(Dw, Lr)
    sol = Solver(system=system, action = control, dt = 0.01)
    t, x = sol.modeling(state0 = state0, t_span = t_span, control= control)
    fig, ax = p.subplots()
    ax.plot(x[:,0], x[:,1])
    ax.set_title('x and y')
    p.grid(True)
    p.show()

main()