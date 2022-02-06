import gym
import gym_a1
from random import random
import numpy as np

env = gym.make('a1-v0')

#Call env.step(action) in a loop here to take actions
try:
    print("Running simulation")
    for i in range(1000):
        pos = np.array([1 if random() < 0.5 else -1 for i in range(12) ])
        action = np.array([random()*2 for i in range(12)])
        obs, reward, done, info = env.step(pos * action)
    print("Completed simulation")
    env.close()
except:
    print("Force quit simulation")
    env.close()
print("Stopped simulation")