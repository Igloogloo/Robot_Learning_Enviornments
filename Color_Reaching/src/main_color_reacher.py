#!/usr/bin/env python

import rospy
import numpy as np
from hlpr_manipulation_utils.msg import RLTransitionPlusReward
from hlpr_manipulation_utils.srv import GetActionFromObs
from color_reacher import ColorReacher
from kinova_msgs.srv import ClearTrajectories

# Function that will qeuery the model for an action
def get_action(observation):
    # print("Obs, ", observation)
    rospy.wait_for_service('get_action')
    try:
        service = rospy.ServiceProxy('get_action', GetActionFromObs)
        resp1 = service(observation)
        return resp1.action
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def main_loop():

    n_games = 100
    rewards = []
    score_history = []
    load_checkpoint = False
    best_score = -1000

    env_interacts = 0

    transition_publisher = rospy.Publisher('rl_transition', RLTransitionPlusReward, queue_size=10)
    
    try:
        rospy.init_node('transition', anonymous=True)
    except:
        print("Transition publisher not started")
    rate = rospy.Rate(100) # 10hz

    print("Initiating Enviornment")
    env = ColorReacher()
    
    for episode in range(n_games):
        rospy.wait_for_service("/j2s7s300_driver/in/clear_trajectories")
        rospy.ServiceProxy('/j2s7s300_driver/in/clear_trajectories', ClearTrajectories).call()
        observation = env.reset()
        observation = np.array(observation)
        done = False
        score = 0
        rospy.sleep(1)
        while not rospy.is_shutdown() and not done:
            env_interacts += 1
            action = get_action(observation)
            print(action)
            new_observation, reward, done, info = env.step(action)
            new_observation = np.array(new_observation)
            score += reward
            print(reward, "REWARD")
            print(observation, "OBS")
            print(done, "DONE")
            transition = RLTransitionPlusReward()
            old_obs = observation
            action = action
            new_obs = new_observation
            reward = reward
            done = done
            transition.old_observation = list(old_obs)
            transition.action = action
            transition.observation = list(new_obs)
            transition.reward = [reward]
            transition.done = [done]
            transition_publisher.publish(transition)
            
            observation = new_observation

            #rate.sleep()
        
        score_history.append(score)
        avg_score = np.mean(score_history[-100:])

        if avg_score > best_score:
            best_score = avg_score

        rewards.append(score)

        print('episode ', episode, 'score %.1f' % score, 'avg_score %.1f' % avg_score)

if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass