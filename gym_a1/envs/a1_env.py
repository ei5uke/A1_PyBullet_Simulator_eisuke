#from args import parse_args

import gym
from gym import error, spaces, utils
from gym.utils import seeding
import random
import numpy as np
import time as time

#Use PyBullet to simulate the A1 robot
import pybullet as p
import pybullet_data

class A1Env(gym.Env):
	metadata = {'render.modes': ['human']}

	def __init__(self):
		print("In init")
		self.physics_client = p.connect(p.GUI)
		#self.physics_client = p.connect(p.DIRECT)
		
		p.setAdditionalSearchPath(pybullet_data.getDataPath())

		p.setGravity(0,0,-9.807)
		self.start_pos = np.array([0,0,0.2])
		self.og_angles = np.array([0,0.9,-1.8]*4)
		self.plane_id = p.loadURDF("plane.urdf",)
		self.robot = p.loadURDF("gym_a1/envs/a1.urdf", self.start_pos)
		self.dt = 1./240.
		p.setTimeStep(self.dt)
		self.max_forces = [5, 5, 10]*4
		self.num_steps = 20
		self.flip = False
		self.init_pos, self.init_ori = p.getBasePositionAndOrientation(self.robot)

		## we can see all the joints of the A1 robot with this
		# for j in range (p.getNumJoints(self.robot)):
		# 	joint_info = p.getJointInfo(self.robot,j)
		# 	name = joint_info[1].decode('utf-8')
		# 	idx = joint_info[0]
		# 	print(name, idx)

		self.joint_idxs = np.array([2, 4, 5, 7, 9, 10, 12, 14, 15, 17, 19, 20]) # these are the indeces of the joints of our robot
		#self.contact_idxs = [6, 11, 16, 21] # these are the indeces of the fixed feet

		for j in range(0, 12):
			joint_id = self.joint_idxs[j]
			p.resetJointState(self.robot, joint_id, self.og_angles[j])

		print("Completed init")

	def step(self, action):
		done = False
		p.setJointMotorControlArray(self.robot, self.joint_idxs, p.POSITION_CONTROL, action, forces=self.max_forces)
		for step in range(self.num_steps):
			p.stepSimulation()
			if self.isFlipped(): self.reset() # we could put this out of the loop if we want
		return [], 0, done, []

	def reset(self):
		print("Resetting")
		p.resetBasePositionAndOrientation(self.robot, self.init_pos, self.init_ori)
		for j in range(0, 12):
			joint_id = self.joint_idxs[j]
			p.resetJointState(self.robot, joint_id, self.og_angles[j])
		print("Finished reset")

	def isFlipped(self):
		contact_points = p.getContactPoints(self.robot, self.plane_id)

		#if there are no contact points, assume the robot is aerial so don't reset
		if len(contact_points) == 0: 
			#print("aerial")
			return False

		# check all contact points
		for contact_point in contact_points:
			joint = contact_point[3]
			#print(joint)
			if (joint == 0): # if one of the contact points is the imu sensor (which I assume is on the head or back) reset
				return True
			# elif (joint in self.contact_idxs):
			# 	return False
		return False

	def close(self):
		print("Closing")
		p.disconnect()
		print("Closed")

if __name__ == "__main__":
	bob = A1Env()