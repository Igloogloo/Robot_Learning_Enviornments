#!/usr/bin/env python

from balance_toy import BalanceToy
import numpy as np 

env = BalanceToy(stack_size=3)

#for i in range(1000):
#    env.render()

#obs = env.reset()
#print(obs.shape)
done = False
i=-1
while done is False:
    action = i*np.array([10,10,10])
    observation, reward, done, info = env.step(action)
    print("OBS", observation.shape)
    print("REWARD", reward)
    print("DONE", done)
    print("TOTAL time", info)
    #i = -1*i
