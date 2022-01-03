#!/usr/bin/env python

import numpy as np
import rospy 
from sac_torch import Agent
from hlpr_manipulation_utils.msg import RLTransitionPlusReward
from hlpr_manipulation_utils.srv import GetActionFromObs, GetActionFromObsResponse

global agent
agent = Agent(alpha=0.001, beta=0.001, input_dims=env.observation_space[0], env=env, batch_size=128,
            tau=.02, max_size=100000, layer1_size=256, layer2_size=256, n_actions=env.action_space, reward_scale=1, auto_entropy=False)

def learn():
    pass

def store_transition(data):
    rate = rospy.Rate(1)
    rate.sleep()
    print(data.action)

def action_callback(observation):
    print(observation)
    print("ActionSERVICE", observation.observation)
    return GetActionFromObsResponse(observation.observation)
    #pass
    
def learning_listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)


    action_service = rospy.Service('get_action', GetActionFromObs, action_callback)
    print("Ready to recieve actions")
    rospy.Subscriber("rl_transition", RLTransitionPlusReward, store_transition)
    
    learn()

    # spin() simply keeps python from exiting until this node is stopped

    rospy.spin()

if __name__ == '__main__':
    learning_listener()
