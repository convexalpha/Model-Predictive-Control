import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = False

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference1 = [10, 10, 0]
        self.reference2 = [10, 2, 3.14/2]

    def plant_model(self,prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]
        a_t = pedal

        x_t_1 = x_t + v_t*dt*np.cos(psi_t)
        y_t_1 = y_t + v_t*dt*np.sin(psi_t)
        v_t_1 = v_t + a_t*dt
        psi_t_1 = psi_t + np.tan(steering)/2.5 * v_t * dt

        return [x_t_1, y_t_1, psi_t_1, v_t_1]

    def cost_function(self,u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0
        for i in range(self.horizon):
            state = self.plant_model(state, self.dt, u[i], u[2*i+1])
            cost += abs(ref[0]-state[0])
            cost += abs(ref[1]-state[1])
            
            cost += abs(ref[2]-state[2])

            # if state[3]*3.6 > 10:
            #     cost += (1 - 1/state[3])*100

            # cost += ((state[3] - v_start)**2)
            # accelaration cost - cost for changing accelaration quickly so passengers don't get motion sickness

            # cost += (u[i*2+1]**2)*self.dt
            # steering input cost - cost for moving the steering wheel
            
        return cost

sim_run(options, ModelPredictiveControl)
