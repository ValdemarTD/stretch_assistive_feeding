#!/usr/bin/env python3
# Import the FollowJointTrajectoryGoal from the control_msgs.msg package to
# control the Stretch robot
import rospy
from control_msgs.msg import FollowJointTrajectoryGoal
import time


# Import JointTrajectoryPoint from the trajectory_msgs package to define
# robot trajectories
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm
class MultiPointCommand(hm.HelloNode):
	"""
	A class that sends multiple joint trajectory goals to the stretch robot.
	"""

	def __init__(self):
		"""
		Function that initializes the inhereted hm.HelloNode class.
		:param self: The self reference.
		"""
		hm.HelloNode.__init__(self)
		converted_shape = []
		time = 10

	
	def createPath(self, shape, velocities = None, accelerations = None):
		converted_shape = []
		for point in shape:
			trajectory = JointTrajectoryPoint()
			trajectory.positions = [point[0], point[1],point[2], point[3], point[4]]
			if velocities is not None:
				trajectory.velocities = velocities
			if accelerations is None:
				pass
			else:
				trajectory.accelerations = accelerations
			converted_shape.append(trajectory)
		return converted_shape


	def issue_multipoint_command(self, shape, velocities = None, accelerations = None):
		"""
		Function that makes an action call and sends multiple joint trajectory goals
		to the joint_lift, wrist_extension, and joint_wrist_yaw.
		:param self: The self reference.
		"""

		# Set trajectory_goal as a FollowJointTrajectoryGoal and define
		# the joint names as a list
		
		trajectory_goal = FollowJointTrajectoryGoal()
		trajectory_goal.trajectory.joint_names = ['wrist_extension','joint_lift', 'joint_wrist_yaw', 'joint_wrist_roll', 'joint_wrist_pitch']
		# Then trajectory_goal.trajectory.points is defined by a list of the joint
		# trajectory points
		trajectory_goal.trajectory.points = self.createPath(shape=shape, velocities=velocities, accelerations=accelerations)

		# Specify the coordinate frame that we want (base_link) and set the time to be now
		trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
		trajectory_goal.trajectory.header.frame_id = 'base_link'

		# Make the action call and send the goal. The last line of code waits
		# for the result before it exits the python script
		self.trajectory_client.send_goal(trajectory_goal)
		#rospy.loginfo('Sent list of goals = {0}'.format(trajectory_goal))
		self.trajectory_client.wait_for_result()
		


	def single_joint(self, joint_name, points, velcoities= None, accelerations = None):
		trajectory_goal = FollowJointTrajectoryGoal()
		trajectory_goal.trajectory.joint_names = [joint_name]
		# Then trajectory_goal.trajectory.points is defined by a list of the joint
		
		traj_points = []
		for point in points:
			trajectory = JointTrajectoryPoint()
			trajectory.positions = [point]
			if accelerations is not None:
				trajectory.accelerations = accelerations
			if velcoities is not None:
				trajectory.velocities = velcoities
			traj_points.append(trajectory)

		trajectory_goal.trajectory.points = traj_points

		# Specify the coordinate frame that we want (base_link) and set the time to be now
		trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
		trajectory_goal.trajectory.header.frame_id = 'base_link'

		# Make the action call and send the goal. The last line of code waits
		# for the result before it exits the python script
		self.trajectory_client.send_goal(trajectory_goal)
		rospy.loginfo('Sent list of goals = {0}'.format(trajectory_goal))
		#self.trajectory_client.wait_for_result()
		#time.sleep()
	


if __name__ == '__main__':
	pi = 3.14
	length = 0.06
	try:
        # Instanstiate a `MultiPointCommand()` object and execute the main() method
		pointlist = [[0.3,0.5,0], [0.1, 0.9, 0], [0.4, 0.5,0]]
		node = MultiPointCommand(shape=pointlist)
		node.main()	
		node.issue_multipoint_command(pointlist)

	except KeyboardInterrupt:
		rospy.loginfo('interrupt received, so shutting down')
