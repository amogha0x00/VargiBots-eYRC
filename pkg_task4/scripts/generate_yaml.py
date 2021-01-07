#! /usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import conveyorBeltPowerMsg

import yaml
import os
import math
import time
import sys
import copy

from std_srvs.srv import Empty

class Ur5Moveit:

	# Constructor
	def __init__(self, arg_robot_name):

		rospy.init_node('node_moveit_eg6', anonymous=True)

		self._robot_ns = '/'  + arg_robot_name
		self._planning_group = "manipulator"
		
		moveit_commander.roscpp_initialize(sys.argv)
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

		# Attribute to store computed trajectory by the planner	
		self._computed_plan = ''
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
		self._file_path = self._pkg_path + '/config/saved_trajectories{}/'.format(self._robot_ns)
		rospy.loginfo( "Package Path: {}".format(self._file_path) )


		rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

	def clear_octomap(self):
		clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
		return clear_octomap_service_proxy()

	def set_joint_angles(self, arg_list_joint_angles):

		#list_joint_values = self._group.get_current_joint_values()
		# rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
		# rospy.loginfo(list_joint_values)

		self._group.set_joint_value_target(arg_list_joint_angles)
		self._computed_plan = self._group.plan()
		#print(self._computed_plan)
		st_time = rospy.get_time()
		if self._computed_plan.joint_trajectory.points:
			flag_plan = self._group.execute(self._computed_plan, wait=True)
		else:
			flag_plan = False
		self._computed_time = rospy.get_time() - st_time
		# list_joint_values = self._group.get_current_joint_values()
		# rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
		# rospy.loginfo(list_joint_values)

		# pose_values = self._group.get_current_pose().pose
		# rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
		# rospy.loginfo(pose_values)

		if (flag_plan == True):
			pass
			# rospy.loginfo(
			#     '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
		else:
			pass
			# rospy.logerr(
			#     '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

		return flag_plan

	def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

		number_attempts = 0
		flag_success = False
		
		while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
			number_attempts += 1
			flag_success = self.set_joint_angles(arg_list_joint_angles)
			rospy.logwarn("attempts: {} : {}".format(number_attempts, flag_success))
			#self.clear_octomap()

	def go_to_pose(self, arg_pose):

		pose_values = self._group.get_current_pose().pose
		rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
		rospy.loginfo(pose_values)

		self._group.set_pose_target(arg_pose)
		self._computed_plan = self._group.plan()
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
			# # self.clear_octomap()
		
		return True
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

	def add_box(self, box_name, box_pose, timeout=4):	
		""""
			Adding Objects to the Planning Scene
			First, we will create a box in the planning scene at box_pose
		"""
		box_size = (0.16,0.16,0.16)
		box_pose.header.frame_id = self._planning_frame
		self._box_name = box_name
		self._scene.add_box(self._box_name, box_pose, size=box_size)
		return self.wait_for_state_update(box_is_known=True, timeout=timeout)
	# Destructor
	def attach_box(self,timeout=4):
		'''
			Attaching Objects to the Robot
		'''

		#touch_links = self._robot.get_link_names(group=self._planning_group)
		self._scene.attach_box(self._eef_link, self._box_name, touch_links=[self._eef_link])

		# We wait for the planning scene to update.
		return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)
	def detach_box(self, timeout=4):
		"""
			detach last picked box from planing scene
		"""

		self._scene.remove_attached_object(self._eef_link, name=self._box_name)
		# We wait for the planning scene to update.
		return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

	def __del__(self):
		moveit_commander.roscpp_shutdown()
		rospy.loginfo(
			'\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
	def remove_box(self, timeout=4):
		"""
			Remove last picked box from planing scene
		"""
		self._scene.remove_world_object(self._box_name)

		# We wait for the planning scene to update.
		return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

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
def main():

	ur5 = Ur5Moveit('ur5_1')

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

	RedBinPose = geometry_msgs.msg.Pose()
	RedBinPose.position.x = 0 # center of box
	RedBinPose.position.y = 0.65 # somewhere near miiddle
	RedBinPose.position.z = 1 # just above walls of bin
	RedBinPose.orientation.x = -0.5
	RedBinPose.orientation.y = -0.5
	RedBinPose.orientation.z =  0.5
	RedBinPose.orientation.w =  0.5

	YellowBinPose = geometry_msgs.msg.Pose()
	YellowBinPose.position.x = 0.75 # center of box
	YellowBinPose.position.y = 0 # somewhere near miiddle
	YellowBinPose.position.z = 1 # just above walls of bin
	YellowBinPose.orientation.x = -0.5
	YellowBinPose.orientation.y = -0.5
	YellowBinPose.orientation.z = 0.5
	YellowBinPose.orientation.w = 0.5
	
	GreenBinPose = geometry_msgs.msg.Pose()
	GreenBinPose.position.x = 0 # center of box
	GreenBinPose.position.y = -0.65 # somewhere near miiddle
	GreenBinPose.position.z = 1 # just above walls of bin
	GreenBinPose.orientation.x = -0.5
	GreenBinPose.orientation.y = -0.5
	GreenBinPose.orientation.z = 0.5
	GreenBinPose.orientation.w = 0.5

	BoxPose = geometry_msgs.msg.Pose()
	BoxPose.position.x = 0.28
	BoxPose.position.y = (6.59 + Delta) - 7
	BoxPose.position.z = 1.2
	BoxPose.orientation.x = 0
	BoxPose.orientation.y = 0
	BoxPose.orientation.z = -1
	BoxPose.orientation.w = 0.011

	# BoxPose = geometry_msgs.msg.PoseStamped()
	# BoxPose.pose.position.x = -0.8
	# BoxPose.pose.position.y = 0 
	# BoxPose.pose.position.z = 1
	# BoxPose.pose.orientation.w = 1.0

	home_angles = [math.radians(172),
					  math.radians(-41),
					  math.radians(58),
					  math.radians(-107),
					  math.radians(-90),
					  math.radians(0)]
	
	angles = {
	'packagen00_angles' : [2.815168624110804, -1.957159323610826, -0.06813568406921622, -1.1187510933325484, 0.34789107043833223, 0],
	'packagen01_angles' : [2.117601887680756, -1.6382689583102579, -0.3272104479374356, -1.1750402503112065, 1.0467933400552374, 0],
	'packagen02_angles' : [0.9608205816835991, -1.9643683773390794, -0.05335973294242358, -1.1235666740561543, 2.201852020101356, 0],
	'packagen10_angles' : [-0.9607833663019525, -1.4493642323248688, 0.6771322814996603, 0.7714856866781101, 2.159462390060801, 0],
	'packagen11_angles' : [-2.1175658663703265, -2.0387327602213547, 1.7091042218282437, -2.812494654243083, -1.0013358771387182, 0],
	'packagen12_angles' : [-2.8152535337704023, -1.690683517685268, 1.4742655543459708, -2.9226066842135623, -0.3048721471355975, 0],
	'packagen20_angles' : [-0.9605764672406538, -1.643735681421231, 2.029592653976728, 2.756541135143788, -2.159385980446242, 0],
	'packagen21_angles' : [-2.1173231602835703, -2.060962957306092, 2.315577997893504, 2.886384193245158, -1.0024981825093517, 0],
	'packagen22_angles' : [-2.8152248676466956, -1.6434319637221213, 2.0297073347102135, 2.7559627506021194, -0.3035895625726175, 0],
	'packagen30_angles' : [-0.9607709954967598, -1.252967133724633, 2.2894529340625276, 2.1039636047946395, -2.158576423925589, 0]
	}
	a = ['packagen00_angles','packagen01_angles','packagen02_angles','packagen10_angles','packagen11_angles','packagen12_angles','packagen20_angles','packagen21_angles','packagen22_angles']
	redBinAngles = [-1.4021989827315986, -2.419851300424532, -1.6751688763338333, -0.6181097611262976, 1.5708253066354905, 0]
	greenBinAngles = [-1.7394663777934447, -0.7215250654445988, 1.674853829440881, -2.523859580495726, -1.571644619132984, 0]
	yellowBinAngles = [-0.1460482615680263, -0.5637933949566794, 1.3095397138479834, -2.315863329865439, -1.5710326593084032, 0]

	vacuum_gripper = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
	convear_belt = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
	convear_belt(90)

	ur5._box_name = 'packagen30'
	ur5.hard_set_joint_angles(home_angles, 20)
	ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'home_to_packagen30.yaml', 5)
	vacuum_gripper(1)
	ur5.attach_box()
	ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'packagen30_to_home.yaml', 5)

	#ur5.go_to_predefined_pose('allZeros')
	#ur5.go_to_pose(YellowBinPose)
	# ur5.attach_box()
	# ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'packagen01_to_home.yaml'.format(ur5._box_name), 5)
	# for i in a:
	# 	ur5._box_name = i[:-7]
	# 	if ur5._box_name == 'packagen00':
	# 		ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'allZeros_to_packagen00.yaml', 5)
	# 	else:
	# 		ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'home_to_{}.yaml'.format(ur5._box_name), 5)
	# 	vacuum_gripper(1)
	# 	ur5.attach_box()
	# 	ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '{}_to_home.yaml'.format(ur5._box_name), 5)
	# 	vacuum_gripper(0)
	# 	ur5.detach_box()
	# 	ur5.remove_box()

	# x = {}
	# y = {}
	# #ur5.go_to_predefined_pose('allZeros')
	# ur5.hard_set_joint_angles(home_angles, 20)
	# for i in range(3):
	# 	rospy.sleep(1)
	# 	ur5.add_box('box', BoxPose)
	# 	rospy.sleep(1)
	# 	ur5.attach_box()
	# 	ur5.hard_set_joint_angles(greenBinAngles, 20)
	# 	y[ur5._computed_plan] = ur5._computed_time
	# 	print(ur5._computed_time)
	# 	ur5.detach_box()
	#  	ur5.remove_box()
	# 	ur5.hard_set_joint_angles(home_angles, 20)
	# 	x[ur5._computed_plan] = ur5._computed_time
	# 	print(ur5._computed_time)

	# y = {}
	# box_name = 'packagen22'
	# for i in range(20):
	# 	ur5.hard_set_joint_angles(home_angles, 20)
	# 	ur5.hard_set_joint_angles(angles['{}_angles'.format(box_name)], 100)
	# 	y[ur5._computed_plan] = ur5._computed_time
	# 	print(ur5._computed_time)

	# file_path = ur5._file_path + 'home_to_{}_new.yaml'.format(box_name)	
	# plan = sorted(y.items(),key=lambda l: l[1])[0][0]
	# with open(file_path, 'w') as file_save:
	# 	yaml.dump(plan, file_save, default_flow_style=True)


	# ur5.hard_set_joint_angles(home_angles, 2)
	# for i in a[8:]:
	# 	ur5._box_name = i[:-7]
	# 	print('GOING TO {}'.format(ur5._box_name))
	# 	ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'home_to_{}.yaml'.format(ur5._box_name), 5)
	# 	vacuum_gripper(1)
	# 	ur5.attach_box()
	# 	ur5.hard_set_joint_angles(home_angles, 1000)
	# 	file_path = ur5._file_path + '{}_to_home_new.yaml'.format(ur5._box_name)
	# 	with open(file_path, 'w') as file_save:
	# 		yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
		
	# 	vacuum_gripper(0)
	# 	ur5.detach_box()
	# 	ur5.remove_box()

		
	#ur5.hard_set_joint_angles(angles, 5)
	# pose_values = ur5._group.get_current_pose().pose
	# rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
	# rospy.loginfo(pose_values)

	del ur5



if __name__ == '__main__':
	main()

