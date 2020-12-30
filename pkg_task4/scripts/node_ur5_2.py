#!/usr/bin/env python
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.msg import LogicalCameraImage

import yaml
import sys

class Ur5Moveit:

	# Constructor
	def __init__(self, arg_robot_name, packages_info):

		self._robot_ns = '/'  + arg_robot_name
		self._planning_group = "manipulator"
		
		self._commander = moveit_commander.roscpp_initialize(sys.argv)
		self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
		self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
		self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
		self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
		self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
		self._exectute_trajectory_client.wait_for_server()

		self._planning_frame = self._group.get_planning_frame()
		self._eef_link = self._group.get_end_effector_link()
		self._group_names = self._robot.get_group_names()
		self._box_name = ''
		self._picked_box_name = []
		self._packages_info = packages_info
		self._convear_belt = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
		self._vacuum_gripper = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
		self._current_power = 0
		self._pickable_name = ''
		self._flag_pickable = 0


		# Current State of the Robot is needed to add box to planning scene
		self._curr_state = self._robot.get_current_state()

		rospy.loginfo(
			'\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
		rospy.loginfo(
			'\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
		rospy.loginfo(
			'\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


		rp = rospkg.RosPack()
		self._pkg_path = rp.get_path('pkg_task4')
		self._file_path = self._pkg_path + '/config/saved_trajectories/'
		rospy.loginfo( "Package Path: {}".format(self._file_path) )


		rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

	def clear_octomap(self):
		clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
		return clear_octomap_service_proxy()
	
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

	def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
		file_path = arg_file_path + arg_file_name
		
		with open(file_path, 'r') as file_open:
			loaded_plan = yaml.load(file_open)
		
		ret = self._group.execute(loaded_plan)
		# rospy.logerr(ret)
		return ret

	def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
		number_attempts = 0
		flag_success = False

		while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
			number_attempts += 1
			flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
			rospy.logwarn("attempts: {}".format(number_attempts) )
			# self.clear_octomap()

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

	def add_box(self, box_name, box_pose, timeout=4):	
		""""
			Adding Objects to the Planning Scene
			First, we will create a box in the planning scene at box_pose
		"""
		box_size = (0.15, 0.15, 0.15)
		box_pose.header.frame_id = self._planning_frame
		self._box_name = box_name
		self._scene.add_box(self._box_name, box_pose, size=box_size)
	
		return self.wait_for_state_update(box_is_known=True, timeout=timeout)

	def attach_box(self,timeout=4):
		'''
			Attaching Objects to the Robot
		'''
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
		self.go_to_pose(ReqInfo.HomePose)
		self.set_convear_power(90)
		while not self._flag_pickable:
			rospy.sleep(0.1)

		self.add_box(self._pickable_name, ReqInfo.BoxPose)

		print('Time to pick!!!')
		self.attach_box()
		self._picked_box_name.append(self._pickable_name)
		self._flag_pickable == 0
	
	def place_box(self):
		"""
			Places the box in both gazebo and rviz to position pick_pose
		"""

		if self._packages_info[self._picked_box_name[-1]] == 'red':
			self.go_to_pose(ReqInfo.RedBinPose)
		elif self._packages_info[self._picked_box_name[-1]] == 'green':
			self._ur5.go_to_pose(ReqInfo.GreenBinPose)
		else:
			self.go_to_pose(ReqInfo.BlueBinPose)

		self.detach_box()
		self.remove_box()

	def cam_callback(self, cam):
		"""
			Logical camera 2 callback function
		"""
		for model in cam.models:
			if model.type[:-2] == 'packagen':
				package_name = model.type
				if package_name in self._picked_box_name:
					continue

				if abs(model.pose.position.y) <= 0.1:
					#print('PICKING!!!!!!!!!')
					self._pickable_name = package_name
					self._flag_pickable = 1
					self.set_convear_power(0)
				else:
					self._flag_pickable = 0
				break
	def set_convear_power(self, power):
		if self._current_power == power:
			return
		self._current_power = power
		self._convear_belt(self._current_power)

	# Destructor
	def __del__(self):
		moveit_commander.roscpp_shutdown()
		rospy.loginfo(
			'\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

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

	BoxPose = geometry_msgs.msg.PoseStamped()
	BoxPose.pose.position.x = -0.8 # (+offset of self) + (-box pose)
	BoxPose.pose.position.y = 0 # (+offset of self) + (-box pose)
	BoxPose.pose.position.z = 1 # (+offset of self) + (+box pose)
	BoxPose.pose.orientation.w = 1.0

def main():
	rospy.init_node('node_ur5_2', anonymous=True)
	try:
		while not rospy.has_param('/packages_color_info'):
			rospy.sleep(0.1)
	except KeyboardInterrupt:
		print('Quiting')
	info = rospy.get_param('/packages_color_info')
	print(info)
	ur5_2 = Ur5Moveit('ur5_2', info)
	rospy.Subscriber('eyrc/vb/logical_camera_2', LogicalCameraImage, ur5_2.cam_callback)
	i = 0
	while not rospy.is_shutdown() and i < 3:
		ur5_2.pick_box()
		ur5_2.place_box()
		i+=1
	ur5_2.go_to_pose(ReqInfo.HomePose)

	del ur5_2

if __name__ == '__main__':
	main()
