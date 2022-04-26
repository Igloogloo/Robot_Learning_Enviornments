#!/usr/bin/env python

from sphero_push import SpheroPush

env = SpheroPush(home_arm=False)

obs = env.reset()
print(obs)
action = [.1,.1]
new_state, reward, done, info = env.step(action)
print("State:", new_state, " Reward:", reward, " Done:", done, " info: ", info )