#!/usr/bin/env python

import imp
import rospy
from sphero_push import SpheroPush
from kinova_msgs.msg import ArmPoseAction
from sphero_interface.msg import ObsMessage
import numpy as np
rospy.init_node("aaaa")
observation = rospy.wait_for_message("/rl_observation", ObsMessage)
observation = np.array(observation.obs)
print(observation)
# env = SpheroPush(home_arm=True)

# obs = env.reset()
# print(obs)
# action = [.1,.1]
# new_state, reward, done, info = env.step(action)
# print("State:", new_state, " Reward:", reward, " Done:", done, " info: ", info )