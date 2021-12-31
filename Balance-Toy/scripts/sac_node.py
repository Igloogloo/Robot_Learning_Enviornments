#!/usr/bin/env python

import numpy as np
import rospy 
from hlpr_manipulation_utils.msg import RLTransitionPlusReward
from hlpr_manipulation_utils.srv import GetActionFromObs, GetActionFromObsResponse

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
