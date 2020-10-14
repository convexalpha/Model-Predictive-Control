import numpy as np
from sim.sim1d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['FULL_RECALCULATE'] = False

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference = [50, 0, 0]

    def plant_model(self, prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        v_t = prev_state[3] # m/s
        a_t = pedal
        
        x_t_1 = x_t + v_t * dt
        v_t_1 = v_t + a_t * dt - v_t / 15.0
        return [x_t_1, 0, 0, v_t_1]

    def cost_function(self,u, *args):

        state = args[0]
        ref = args[1]
        cost = 0.0
        ideal_speed = 10.0
        for k in range(self.horizon):
            state = self.plant_model(state, self.dt, u[k*2], u[k*2+1])
            cost += (ref[0]-state[0])**2
            if ((state[3] * 3.6) > ideal_speed):
                cost += 1000*(1 - (1/state[3]))
            if state[0]>ref[0]:
                cost += 1000

        return cost

sim_run(options, ModelPredictiveControl)
