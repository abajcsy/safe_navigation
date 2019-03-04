#!/usr/bin/env python
# license removed for brevity
import numpy as np
import matplotlib.pyplot as plt

# ROS imports.
import rospy
from nav_msgs.msg import OccupancyGrid
from safe_navigation_msgs.msg import Trajectory

class DummyNNPlannerNode(object):
	"""
	This class acts as a dumb "NN planner" and just spoofs trajectories and 
	occupancy maps so we can test the MATLAB side of things.
	"""

	def __init__(self):

		# create ROS node
		rospy.init_node('dummy_nn_planner', anonymous=True)

		# load all the params and setup subscriber/publishers
		self.register_callbacks()

		# publish the big ground-truth initial map.
		gt_occu_map_msg = self.get_gt_occu_map_msg()
		self.map_pub.publish(gt_occu_map_msg)

		# make some fake states and controls.
		xinit = [2.0, 2.5, np.pi/2, 0.0]
		replan_rate = 10 # seconds
		states = []
		controls = [[0.0, 0.4], [0.0, 0.2], [0.0, 0.0]]
		traj_msg = self.to_traj_msg(states, controls)
		self.plan_pub.publish(traj_msg)
		prev_plan_t = rospy.Time.now()

		rate = rospy.Rate(100) 
		while not rospy.is_shutdown():
			# fake replanning at a fixed frequency
			dt = (rospy.Time.now() - prev_plan_t).to_sec()
			print dt
			if dt >= replan_rate:
				prev_plan_t = rospy.Time.now()
				traj_msg = self.to_traj_msg(states, controls)
				self.plan_pub.publish(traj_msg)

			rate.sleep()


	def register_callbacks(self):
		"""
		Sets up all the publishers/subscribers needed.
		"""

		verifiedTopicName = '/verified_traj'
		occuMapTopicName = '/occu_map'
		planTopicName = '/planned_traj'

		# publisher sending out trajectory & occupancy map
		self.plan_pub = rospy.Publisher(planTopicName, Trajectory, queue_size=1)
		self.map_pub = rospy.Publisher(occuMapTopicName, OccupancyGrid, queue_size=1)

		# subscriber for state info
		self.verified_sub = rospy.Subscriber(verifiedTopicName, Trajectory, 
			self.verified_callback, queue_size=1)

	def verified_callback(self, msg):
		"""
		Does operations on the state.
		"""

		print "Got a verified trajectory of length: ", len(msg.controls)
		

	def to_traj_msg(self, states, controls):
		"""
		Converts state and control trajectories into Trajectory ROS message.
		"""
		traj_msg = Trajectory()
		traj_msg.states = []
		traj_msg.controls = []

		for (s,c) in zip(states, controls):
			state_msg = State()
			control_msg = Control()
			state_msg.x = s
			control_msg.u = c

			# Store the state and control messages in trajectory
			traj_msg.states.append(state_msg)
			traj_msg.controls.append(control_msg)

		return traj_msg

	def get_gt_occu_map_msg(self):
		"""
		Fake ground-truth occupancy grid from SBPD.
		"""
		h = 30
		w = 40
		dx = 0.05
		numR = int(h/dx)
		numC = int(w/dx)
		map2D = np.ones((numR, numC))
		map2D[300:600, 200:400] = -1;

		occu_map = OccupancyGrid()
		occu_map.info.resolution = dx
		occu_map.info.width = w
		occu_map.info.height = h
		occu_map.data = map2D.flatten()
		return occu_map


if __name__ == '__main__':
	plannerNode = DummyNNPlannerNode()