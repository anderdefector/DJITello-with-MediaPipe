import numpy as np
class controlDrone():
    def __init__(self, obj, k, zm):
        self.obj = obj
        self.k = k
        self.zm = zm
        self.e_ant = 0.0

    def estimacion_vel_zm(self, s_a, e_ant):
        e = self.obj - s_a
        if abs(e) <= self.zm:
            e_ant = e
            v_sat = 0
        else:
            v = self.k[0] * e + self.k[1] *(e - e_ant)
            e_ant = e    
            v_sat = int(np.clip(v, -100, 100))

        return v_sat, e_ant