#!/usr/bin/env python

"""
This code mainly follows a Soft-Actor Critic YouTube tutorial found at:
https://www.youtube.com/watch?v=ioidsRlf79o&t=2649s
Channel name: Machine Learning with Phil

Any modifiations are made by the AABL Lab.
"""

from balance_toy import BalanceToy
import numpy as np
from sac_torch import Agent
import time
from scipy.special import expit
from sklearn.preprocessing import normalize

if __name__ == '__main__':
    env = env = BalanceToy(stack_size=3, with_pixels=False, max_action=1, max_action_true=40)
    agent = Agent(alpha=0.001, beta=0.001, input_dims=env.observation_space[0], env=env, batch_size=128,
            tau=.02, max_size=100000, layer1_size=256, layer2_size=256, n_actions=env.action_space, reward_scale=1, auto_entropy=False)
    n_games = 100
    rewards = []

    best_score = -10000
    score_history = []
    load_checkpoint = False

    env_interacts = 0

    for i in range(n_games):
        observation = env.reset()
        observation = observation.astype('float32')
        observation = observation/np.linalg.norm(observation)
        print(observation)
        done = False
        score = 0
        while not done:
            env_interacts+=1
            action = agent.choose_action(observation)
            observation_, reward, done, info = env.step(action)
            observation_ = observation_.astype('float32')
            observation_ = observation_/np.linalg.norm(observation_)
            score += reward
            print(reward, "REWARD")
            print(observation, "OBS")
            agent.remember(observation, action, reward, observation_, done)
            agent.learn(update_params=True)
            observation = observation_
            env.render()

        score_history.append(score)
        avg_score = np.mean(score_history[-100:])

        if avg_score > best_score:
            best_score = avg_score

        rewards.append(score)

        print('episode ', i, 'score %.1f' % score, 'avg_score %.1f' % avg_score)
    
    np.save("BP_sac_2000", rewards)
