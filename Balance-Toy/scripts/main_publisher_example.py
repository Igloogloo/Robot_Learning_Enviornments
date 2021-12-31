#!/usr/bin/env python

import rospy
import numpy as np
from hlpr_manipulation_utils.msg import RLTransitionPlusReward
from hlpr_manipulation_utils.srv import GetActionFromObs
from rospy import service

# Function that will qeuery the model for an action
def get_action(observation):
    print("Obs, ", observation)
    rospy.wait_for_service('get_action')
    try:
        service = rospy.ServiceProxy('get_action', GetActionFromObs)
        resp1 = service(observation)
        print(resp1.action)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def main_loop():
    pub = rospy.Publisher('rl_transition', RLTransitionPlusReward, queue_size=10)
    rospy.init_node('transition', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    i = 1
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        transition = RLTransitionPlusReward()
        old_obs = [.4, .5]
        action = [i]
        new_obs = [.99, .44]
        reward = [.33]
        done = [0]
        transition.old_observation = old_obs
        transition.action = action
        transition.observation = new_obs
        transition.reward = reward
        transition.done = done
        i+=1

        print("Action ", get_action(list(new_obs)))
        
        #rospy.loginfo(hello_str)
        pub.publish(transition)
        rate.sleep()

if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass