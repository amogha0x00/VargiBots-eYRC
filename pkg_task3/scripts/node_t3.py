#! /usr/bin/env python

import rospy
import sys
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.msg import LogicalCameraImage
from pkg_vb_sim.srv import conveyorBeltPowerMsg
import tf2_ros
import tf2_msgs.msg

class ReqInfo:

	RedBinPose = geometry_msgs.msg.Pose()
	RedBinPose.position.x = 0.11 # center of box
	RedBinPose.position.y = 0.65 # somewhere near miiddle
	RedBinPose.position.z = 1 # just above walls of bin
	RedBinPose.orientation.x = -0.5
	RedBinPose.orientation.y = -0.5
	RedBinPose.orientation.z =  0.5
	RedBinPose.orientation.w =  0.5

	GreenBinPose = geometry_msgs.msg.Pose()
	GreenBinPose.position.x = 0.75 # center of box
	GreenBinPose.position.y = 0.03 # somewhere near miiddle
	GreenBinPose.position.z = 1 # just above walls of bin
	GreenBinPose.orientation.x = -0.5
	GreenBinPose.orientation.y = -0.5
	GreenBinPose.orientation.z = 0.5
	GreenBinPose.orientation.w = 0.5
	
	BlueBinPose = geometry_msgs.msg.Pose()
	BlueBinPose.position.x = 0.04 # center of box
	BlueBinPose.position.y = -0.65 # somewhere near miiddle
	BlueBinPose.position.z = 1 # just above walls of bin
	BlueBinPose.orientation.x = -0.5
	BlueBinPose.orientation.y = -0.5
	BlueBinPose.orientation.z = 0.5
	BlueBinPose.orientation.w = 0.5

	VacuumGripperWidth = 0.115    # Vacuum Gripper Width
	BoxSize = (0.15, 0.15, 0.15) # cube of sides 0.15 units
	Delta = VacuumGripperWidth + (BoxSize[0]/2) + 0.01 # 0.19
	
	HomePose = geometry_msgs.msg.Pose()
	HomePose.position.x = -0.8
	HomePose.position.y = 0
	HomePose.position.z = 1 + Delta
	# This to keep EE parallel to Ground Plane
	HomePose.orientation.x = -0.5
	HomePose.orientation.y = -0.5
	HomePose.orientation.z = 0.5
	HomePose.orientation.w = 0.5

class CartesianPath:

	# Constructor
	def __init__(self):

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

		rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')


	def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
		# 1. Create a empty list to hold waypoints
		waypoints = []

		# 2. Add Current Pose to the list of waypoints
		waypoints.append(self._group.get_current_pose().pose)

		# 3. Create a New waypoint
		wpose = geometry_msgs.msg.Pose()
		wpose.position.x = waypoints[0].position.x + (trans_x)  
		wpose.position.y = waypoints[0].position.y + (trans_y)  
		wpose.position.z = waypoints[0].position.z + (trans_z)
		# This to keep EE parallel to Ground Plane
		wpose.orientation.x = -0.5
		wpose.orientation.y = -0.5
		wpose.orientation.z = 0.5
		wpose.orientation.w = 0.5


		# 4. Add the new waypoint to the list of waypoints
		waypoints.append(copy.deepcopy(wpose))


		# 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
		(plan, fraction) = self._group.compute_cartesian_path(
			waypoints,   # waypoints to follow
			0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
			0.0)         # Jump Threshold
		rospy.loginfo("Path computed successfully. Moving the arm.")

		# The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
		# https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
		num_pts = len(plan.joint_trajectory.points)
		if (num_pts >= 3):
			del plan.joint_trajectory.points[0]
			del plan.joint_trajectory.points[1]

		# 6. Make the arm follow the Computed Cartesian Path
		self._group.execute(plan)

	
	def go_to_pose(self, arg_pose):

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

		if (flag_plan == True):
			rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
		else:
			rospy.logerr(
				'\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

		return flag_plan


	# Destructor
	def __del__(self):
		moveit_commander.roscpp_shutdown()
		rospy.loginfo(
			'\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')

class PickPlace:

		# Constructor
	def __init__(self):
		self._tansition_x = 0
		self._planning_group = "ur5_1_planning_group"
		#moveit_commander.roscpp_initialize(sys.argv)
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

		self._tfBuffer = tf2_ros.Buffer()
		self._listener = tf2_ros.TransformListener(self._tfBuffer)
		self._vacuum_gripper = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
		self._convear_belt = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
		self._flag_pickable = 0
		self._flag_take_pose = 0
		self._pickable_name = ''
		self._picked_box_name = []
		self._ur5 = CartesianPath()
		self._current_power = 0

		# Current State of the Robot is needed to add box to planning scene
		self._curr_state = self._robot.get_current_state()
		rospy.loginfo('\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')

		rospy.loginfo('\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')

		rospy.loginfo('\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

		rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

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


	def add_box(self, box_name, box_size, box_pose, timeout=4):	
		""""
			Adding Objects to the Planning Scene
			First, we will create a box in the planning scene at box_pose
		"""

		box_pose.header.frame_id = self._planning_frame
		self._box_name = box_name
		self._scene.add_box(self._box_name, box_pose, size=box_size)
		return self.wait_for_state_update(box_is_known=True, timeout=timeout)

	def attach_box(self, timeout=4):
		'''
			Attaching Objects to the Robot
		'''

		#touch_links = self._robot.get_link_names(group=self._planning_group)
		self._scene.attach_box(self._eef_link, self._pickable_name, touch_links=[self._eef_link])

		self._vacuum_gripper(1)
		# We wait for the planning scene to update.
		return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

	def detach_box(self, timeout=4):
		"""
			detach last picked box from planing scene
		"""

		self._scene.remove_attached_object(self._eef_link, name=self._picked_box_name[-1])
		self._vacuum_gripper(0)
		# We wait for the planning scene to update.
		return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

	def remove_box(self, timeout=4):
		"""
			Remove last picked box from planing scene
		"""
		self._scene.remove_world_object(self._picked_box_name[-1])

		# We wait for the planning scene to update.
		return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

	def pick_box(self):
		'''
			 Picks the box in both gazebo and rviz from position pick_pose 
		'''
		self._ur5.go_to_pose(ReqInfo.HomePose)
		self.set_convear_power(90)
		while not self._flag_take_pose:
			#print('Yep Yep')
			rospy.sleep(0.1)
		if abs(self._tansition_x) > 0:
			self._ur5.ee_cartesian_translation(self._tansition_x, 0, 0)
		self._flag_take_pose = 0
		print('done transition')
		while not self._flag_pickable:
			pass

		rospy.sleep(0.1)
		box_pose = self.func_tf(self._planning_frame, 'logical_camera_2_' + self._pickable_name+ '_frame')
		self.add_box(self._pickable_name, ReqInfo.BoxSize, box_pose)

		print('Time to pick!!!')
		self._picked_box_name.append(self._pickable_name)
		self.attach_box()
		self._flag_pickable == 0
		self._flag_take_pose = 0

	def place_box(self):
		"""
			Places the box in both gazebo and rviz to position pick_pose
		"""

		if self._picked_box_name[-1][-1] == '1':
			self._ur5.go_to_pose(ReqInfo.RedBinPose)
		elif self._picked_box_name[-1][-1] == '2':
			self._ur5.go_to_pose(ReqInfo.GreenBinPose)
		else:
			self._ur5.go_to_pose(ReqInfo.BlueBinPose)

		self.detach_box()
		self.remove_box()
		#self.set_convear_power(90)


	def cam_callback(self, cam):
		"""
			Logical camera 2 callback function
		"""
		for model in cam.models:
			if model.type[:-2] == 'package':
				package_name = model.type
				if package_name in self._picked_box_name:
					continue
				self._tansition_x = round(model.pose.position.z, 2)
				self._flag_take_pose = 1

				print(self._tansition_x, round(model.pose.position.y, 2), package_name)
				if abs(model.pose.position.y) <= 0.1:
					#print('PICKING!!!!!!!!!')
					self._pickable_name = package_name
					self._flag_pickable = 1
					self.set_convear_power(0)
				else:
					self._flag_pickable = 0
				break

	def func_tf(self, arg_frame_1, arg_frame_2):
		"""
			Function to get transformation from reference frame 'arg_frame1' to 'arg_frame2'
		"""
		try:
			trans = self._tfBuffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time(0))
			trans_pose = geometry_msgs.msg.PoseStamped()
			trans_pose.pose.position.x = round(trans.transform.translation.x, 3)
			trans_pose.pose.position.y = round(trans.transform.translation.y, 3)
			trans_pose.pose.position.z = round(trans.transform.translation.z, 3)
			trans_pose.pose.orientation.x = round(trans.transform.rotation.x, 3)
			trans_pose.pose.orientation.y = round(trans.transform.rotation.y, 3)
			trans_pose.pose.orientation.z = round(trans.transform.rotation.z, 3)
			trans_pose.pose.orientation.w = round(trans.transform.rotation.w, 3)
			#print(trans_pose)
			return trans_pose

		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			rospy.logerr("TF error")
	
	def set_convear_power(self, power):
		if self._current_power == power:
			return
		self._current_power = power
		self._convear_belt(self._current_power)

	def __del__(self):

		del self._ur5


def main():
	rospy.init_node('node_t3', anonymous=True)
	rospy.sleep(5)
	ur5 = CartesianPath()
	gazebo = PickPlace()
	rospy.Subscriber('eyrc/vb/logical_camera_2', LogicalCameraImage, gazebo.cam_callback)
	i = 0
	while not rospy.is_shutdown() and i < 3:
		gazebo.pick_box()
		gazebo.place_box()
		i+=1
	ur5.go_to_pose(ReqInfo.HomePose)

	del ur5
	del gazebo

if __name__ == '__main__':
	main()
