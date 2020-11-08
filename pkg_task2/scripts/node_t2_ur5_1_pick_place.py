#! /usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from pkg_vb_sim.srv import vacuumGripper


class Ur5Moveit:
	'''
		Class used for all ur5 arm movement
	'''
	# Constructor
	def __init__(self):

		rospy.init_node('node_eg3_set_joint_angles', anonymous=True)

		self._planning_group = "ur5_1_planning_group"
		moveit_commander.roscpp_initialize(sys.argv)
		self._robot = moveit_commander.RobotCommander()
		self._scene = moveit_commander.PlanningSceneInterface()
		self._group = moveit_commander.MoveGroupCommander(self._planning_group)
		self._display_trajectory_publisher = rospy.Publisher(
			'/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

		self._exectute_trajectory_client = actionlib.SimpleActionClient(
			'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
		self._exectute_trajectory_client.wait_for_server()

		self._planning_frame = self._group.get_planning_frame()
		self._eef_link = self._group.get_end_effector_link()
		self._group_names = self._robot.get_group_names()
		self._box_name = ''

		self._vacuum_gripper = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)

		# Current State of the Robot is needed to add box to planning scene
		self._curr_state = self._robot.get_current_state()

		rospy.loginfo('\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')

		rospy.loginfo('\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')

		rospy.loginfo('\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

		rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

	def go_to_pose(self, arg_pose):
		'''
			Move EE to position arg_pose
		'''

		pose_values = self._group.get_current_pose().pose
		rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
		rospy.loginfo(pose_values)

		self._group.set_pose_target(arg_pose)
		flag_plan = self._group.go(wait=True)  # wait=False for Async Move

		pose_values = self._group.get_current_pose().pose
		rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
		rospy.loginfo(pose_values)

		list_joint_values = self._group.get_current_joint_values()
		rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
		rospy.loginfo(list_joint_values)

		if flag_plan:
			rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
		else:
			rospy.logerr('\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

		return flag_plan

	def go_to_predefined_pose(self, arg_pose_name):
		'''
			Move EE to predefined position arg_pose_name
		'''

		rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
		self._group.set_named_target(arg_pose_name)
		plan = self._group.plan()
		goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
		goal.trajectory = plan
		self._exectute_trajectory_client.send_goal(goal)
		self._exectute_trajectory_client.wait_for_result()
		rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

	def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
		'''
			Ensuring Collision Updates Are Received
			^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
			If the Python node dies before publishing a collision object update message, the message
			could get lost and the box will not appear. To ensure that the updates are
			made, we wait until we see the changes reflected in the
			``get_attached_objects()`` and ``get_known_object_names()`` lists.
			We call this function after adding,removing, attaching or detaching an object
			in the planning scene. We then wait until the updates have been made or
			``timeout`` seconds have passed
		'''
		start = rospy.get_time()
		seconds = rospy.get_time()
		while (seconds - start < timeout) and not rospy.is_shutdown():
			# Test if the box is in attached objects
			attached_objects = self._scene.get_attached_objects([self._box_name])
			is_attached = len(attached_objects.keys()) > 0
			# Test if the box is in the scene.
			# Note that attaching the box will remove it from known_objects
			is_known = self._box_name in self._scene.get_known_object_names()
			# Test if we are in the expected state
			if (box_is_attached == is_attached) and (box_is_known == is_known):
				return True
			# Sleep so that we give other threads time on the processor
			rospy.sleep(0.1)
			seconds = rospy.get_time()
			# If we exited the while loop without returning then we timed out
		return False
		## END_SUB_TUTORIAL


	def add_box(self, box_name, box_size, box_pose, timeout=4):	
		'''
			Adding Objects to the Planning Scene
			First, we will create a box in the planning scene at box_pose
		'''

		box_pose.header.frame_id = self._planning_frame
		self._box_name = box_name
		self._scene.add_box(self._box_name, box_pose, size=box_size)
		return self.wait_for_state_update(box_is_known=True, timeout=timeout)

	def attach_box(self, timeout=4):
		'''
			Attaching Objects to the Robot
		'''

		#touch_links = self._robot.get_link_names(group=self._planning_group)
		self._scene.attach_box(self._eef_link, self._box_name, touch_links=[self._eef_link])

		self._vacuum_gripper(1)
		# We wait for the planning scene to update.
		return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

	def detach_box(self, timeout=4):
		'''
		BEGIN_SUB_TUTORIAL detach_object
		Detaching Objects from the Robot
		We can also detach and remove the object from the planning scene:
		'''

		self._scene.remove_attached_object(self._eef_link, name=self._box_name)
		self._vacuum_gripper(0)
		# We wait for the planning scene to update.
		return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

	def pick_box(self, pick_pose):
		'''
			 Picks the box in both gazebo and rviz from position pick_pose 
		'''
		self.go_to_pose(pick_pose)
		self.attach_box()

	def place_box(self, place_pose):
		'''
			Places the box in both gazebo and rviz to position pick_pose
		'''
		self.go_to_pose(place_pose)
		self.detach_box()

	# Destructor
	def __del__(self):
		moveit_commander.roscpp_shutdown()
		rospy.loginfo('\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


class ReqInfo:
	'''
		Class which contains all required pose and size
	'''
	PickPose = geometry_msgs.msg.Pose()
	PickPose.position.x = 0 # centre of box
	PickPose.position.y = 0.31 - 0.15/2 + 0.02 # box_position - half of box length + vaccum range
	PickPose.position.z = 1.97 - 0.15/2 + 0.02 # box_position - half of box length + vaccum range
	PickPose.orientation.x = 0
	PickPose.orientation.y = 0
	PickPose.orientation.z = 0
	PickPose.orientation.w = 1

	PlacePose = geometry_msgs.msg.Pose()
	PlacePose.position.x = -0.8 # center of box
	PlacePose.position.y = -0.1 # somewhere near miiddle
	PlacePose.position.z = 0.8 # just below walls of bin
	PlacePose.orientation.x = -0.49999984 # wrist 2 facing the ground
	PlacePose.orientation.y = -0.49999984
	PlacePose.orientation.z = 0.49999984
	PlacePose.orientation.w = 0.50039816

	BoxPose = geometry_msgs.msg.PoseStamped()
	BoxPose.pose.position.x = 0.04 - 0.03 # (+offset of self) + (-box pose)
	BoxPose.pose.position.y = 0.76 - 0.31 # (+offset of self) + (-box pose)
	BoxPose.pose.position.z = 1.97 - 0.05 # (+offset of self) + (+box pose)
	BoxPose.pose.orientation.w = 1.0

	BoxSize = (0.15, 0.15, 0.15) # cube of sides 0.15 units

def main():

	rospy.sleep(5) # delay need when it is launched from launch file
	ur5 = Ur5Moveit()
	box_name = 'task2_box'
	ur5.add_box(box_name, ReqInfo.BoxSize, ReqInfo.BoxPose)
	ur5.pick_box(ReqInfo.PickPose)
	ur5.place_box(ReqInfo.PlacePose)
	ur5.go_to_predefined_pose('allZeros')
	del ur5


if __name__ == '__main__':
	main()
