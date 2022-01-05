#!/usr/bin/env python

from color_reacher import ColorReacher

env = ColorReacher()

obs = env.reset()
print(obs)
action = [.1,.1]
new_state, reward, done, info = env.step(action)
print("State:", new_state, " Reward:", reward, " Done:", done, " info: ", info )