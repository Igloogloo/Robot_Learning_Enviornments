#!/usr/bin/env python

"""
This code mainly follows a Soft-Actor Critic YouTube tutorial found at:
https://www.youtube.com/watch?v=ioidsRlf79o&t=2649s
Channel name: Machine Learning with Phil

Any modifiations are made by the AABL Lab.
"""

import numpy as np
import time
import os
from datetime import datetime

class ReplayBuffer():
    def __init__(self, max_size, input_shape, n_actions, periodically_save=[False, 0]): # periodically_save: [bool, save_period]
        self.mem_size = max_size
        self.mem_cntr = 0
        self.memory = np.zeros(max_size)
        self.state_memory = np.zeros((self.mem_size, input_shape))
        self.new_state_memory = np.zeros((self.mem_size, input_shape))
        self.action_memory = np.zeros((self.mem_size, n_actions))
        self.reward_memory = np.zeros(self.mem_size)
        self.terminal_memory = np.zeros(self.mem_size, dtype=np.bool)
        
        # For saving to file
        self.periodically_save, self.save_period = periodically_save
        if (self.save_period > self.mem_size):
            print("ERROR: trying to save more than is buffered. Don't do this.")
        self.timestamp_memory = np.zeros(self.mem_size) # Don't assume anything about the environment and record the timestamps we 
        self.save_path = "../data/transitions/" + datetime.now().strftime("%d_%H_%M_%S_%m_%Y") +"/" # yeah its weird but day-hour seems the most helpful for deleting bad data.


    def store_transition(self, state, action, reward, state_, done, timestamp=None):
        index = self.mem_cntr % self.mem_size

        self.state_memory[index] = state
        self.new_state_memory[index] = state_
        self.action_memory[index] = action
        self.reward_memory[index] = reward
        self.terminal_memory[index] = done
        self.timestamp_memory[index] = time.time() if not timestamp else timestamp 

        self.mem_cntr += 1

        if (self.periodically_save and self.mem_cntr % self.save_period == 0): self.save_buffer() # save {save_period} samples to file

    def sample_buffer(self, batch_size):
        max_mem = min(self.mem_cntr, self.mem_size)

        batch = np.random.choice(max_mem, batch_size)

        states = self.state_memory[batch]
        states_ = self.new_state_memory[batch]
        actions = self.action_memory[batch]
        rewards = self.reward_memory[batch]
        dones = self.terminal_memory[batch]

        return states, actions, rewards, states_, dones

    def save_buffer(self):
        '''
        Save the transition history to a file
        TODO: save the index of the action instead of a one-hot vector
        '''
        if not os.path.isdir(self.save_path): os.mkdir(self.save_path)

        from_idx = (self.mem_cntr - self.save_period) % self.mem_size
        to_idx = self.mem_cntr if (self.mem_cntr % self.mem_size)==0 else (self.mem_cntr) % self.mem_size # Catch that last sample
        print(f"Saving {from_idx}, {to_idx}")
        np.savez(self.save_path+datetime.now().strftime("%H_%M_%S_%f")[:-5], # 1 decimal point ms
                state=self.state_memory[from_idx:to_idx], 
                new_state=self.new_state_memory[from_idx:to_idx], 
                action=self.action_memory[from_idx: to_idx], 
                reward=self.reward_memory[from_idx: to_idx], 
                terminal=self.terminal_memory[from_idx: to_idx], 
                timestamp=self.timestamp_memory[from_idx: to_idx]
                )

    def load_buffer(self, filename):
        '''
        Order and store n_samples from the read file into the buffer
        TODO: There are definitely faster ways of doing this.
        '''
        # this is unfortunate statefullness :(
        if (self.periodically_save): 
            print("WARN: Loading into a buffer in save mode. Turning off save mode to prevent saving loaded data. ReplayBuffer will no longer save.")
            self.periodically_save = False

        to_load = np.load(self.save_path+filename)
        for s, s_n, a, r, d, ts in zip(to_load["state"], to_load["new_state"], to_load["action"], to_load["reward"], to_load["terminal"], to_load["timestamp"]):
            self.store_transition(s, a, r, s_n, d, ts)


if __name__ == "__main__":
    import random
    shape, n_actions, save_params = 8, 5, [True, 10]
    buffer = ReplayBuffer(100, shape, n_actions, save_params)

    for i in range(120):
        buffer.store_transition(np.ones(shape), i, random.random(), -1*np.ones(shape), False)
        time.sleep(0.01)

    saved_files = sorted(os.listdir(buffer.save_path))
    buffer.load_buffer(saved_files[0])
