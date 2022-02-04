import gym
from gym import error, spaces, utils
from gym.utils import seeding

#Use PyBullet to simulate the A1 robot
import pybullet as p
import pybullet_data

from args import parse_args

args = parse_args()

class A1Env(gym.Env):

	def __init__(self):
		print("In init")
		#p.GUI for visual version, getCameraImage api
		self.physics_client = p.connect(p.DIRECT) if args.direct else p.connect(p.GUI) 
		p.setAdditionalSearchPath(pybullet_data.getDataPath())
		p.setGravity(*args.gravity)
		self.plane_id = p.loadURDF(args.plane)
		self.robot = p.loadURDF(args.robot)
		self.start_pos = args.start_pos
		print(f"{p.getJointInfo(self.robot, 0)}")
		print("Completed init")

	def step(self, action):
		print("In Step")
		#p.stepSimulation()
		print("Completed Step")

	def reset(self):
		print("In reset")
		#self.start_pos = [0, 0, 5]

if __name__ == "__main__":
	bob = A1Env()