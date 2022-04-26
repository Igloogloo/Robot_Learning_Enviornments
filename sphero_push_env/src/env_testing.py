#!/usr/bin/env python

import imp
from sphero_push import SpheroPush
from sphero_push_env.msg import Observation
from kinova_msgs.msg import ArmPoseAction

env = SpheroPush(home_arm=False)

obs = env.reset()
print(obs)
action = [.1,.1]
new_state, reward, done, info = env.step(action)
print("State:", new_state, " Reward:", reward, " Done:", done, " info: ", info )