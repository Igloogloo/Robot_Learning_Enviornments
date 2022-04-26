#!/usr/bin/env python

import numpy as np
import rospy 
import pickle
from sac_torch import Agent
from hlpr_manipulation_utils.msg import RLTransitionPlusReward
from hlpr_manipulation_utils.srv import GetActionFromObs, GetActionFromObsResponse

# TODO: make agent parameters passable
global agent
agent = Agent(alpha=0.001, beta=0.001, input_dims=16, batch_size=128,
            tau=.02, max_size=100000, layer1_size=256, layer2_size=256, n_actions=2, reward_scale=2, 
            max_action=1, auto_entropy=True)

#def learn():
#    agent.learn

def store_transition_and_learn(data):
    old_observation = data.old_observation
    action = data.action
    reward = data.reward
    observation = data.observation
    done = data.done
    global agent
    agent.remember(old_observation, action, reward[0], observation, done[0])
    agent.learn()

def action_callback(observation):
    observation = observation.observation
    action = agent.choose_action(observation)
    return GetActionFromObsResponse(action)


# def save_callback():
#     global agent
#     pickle.dump(agent, open( "color_reacher_agent.p", "wb" ) )
#     return 
    
def learning_listener():

    rospy.init_node('listener', anonymous=True)

    action_service = rospy.Service('get_action', GetActionFromObs, action_callback)
    print("Ready to recieve actions")
    #rospy.Service('save_agent', GetActionFromObs, action_callback)
    rospy.Subscriber("rl_transition", RLTransitionPlusReward, store_transition_and_learn)
    
    # learn()

    rospy.spin()

if __name__ == '__main__':
    learning_listener()
