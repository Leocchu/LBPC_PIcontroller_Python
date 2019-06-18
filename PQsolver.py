def PQ_solver(self, c37):
    V_mag = c37['phasorChannels'][0]['data'][-1]['magnitude']
    V_ang = c37['phasorChannels'][0]['data'][-1]['angle']
    I_mag = c37['phasorChannels'][?]['data'][-1]['magnitude']
    I_ang = c37['phasorChannels'][?]['data'][-1]['angle']

    theta = V_ang - I_ang
    self.Pact = V_mag * I_mag * (math.cos(math.radians(theta)))
    self.Qact = V_mag * I_mag * (math.sin(math.radians(theta)))
