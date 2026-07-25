from Kinematic import TwoWheeledRobot
import numpy as np
class Solver:
    def __init__(self, system: TwoWheeledRobot,action, dt: float):
        self.system = system
        self.action = action
        self.dt = dt

    # runge_kutta 4 
    def _step_rk4(self, t, y, dt, control):
        k1 = self.system.kinematic(t = t, state = y,control = control)
        k2 = self.system.kinematic(t = t + dt/2,state = y + dt/2 * k1, control = control)
        k3 = self.system.kinematic(t = t + dt/2,state = y + dt/2 * k2, control=control)
        k4 = self.system.kinematic(t = t + dt,state = y + dt * k3, control=control)
        return y + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    def modeling(self, state0, t_span, control):
        state = state0.copy()
        dt = self.dt
        trajectory = []
        time = []
        t0,t_final = t_span
        t = t0
        time.append(t0)
        trajectory.append(state)
        while (t < t_final):
             t  = t + dt
             state = self._step_rk4(t, state, dt, control)
             trajectory.append(state)
             time.append(t)
        return np.array(time), np.array(trajectory)

