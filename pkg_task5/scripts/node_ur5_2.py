#!/usr/bin/env python

"""
	Team ID: VB#1823
	Theme : Vargi Bots

	This node is used to,
	* Push shiped details to IMS(send goal to action server).
	* Ship the Orders based on their color.
"""

import datetime
import sys
import threading

import actionlib
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospkg
import rospy
import yaml
from pkg_ros_iot_bridge.msg import msgRosIotAction  # Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotGoal  # Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult  # Message Class that is used for Result Messages
from pkg_vb_sim.msg import LogicalCameraImage
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import vacuumGripper


class Ur5Moveit:

	# Constructor
	def __init__(self, arg_robot_name, packages_info):

		self._robot_ns = '/' + arg_robot_name
		self._planning_group = "manipulator"

		moveit_commander.roscpp_initialize(sys.argv)
		self._robot = moveit_commander.RobotCommander(robot_description=self._robot_ns + "/robot_description",
													  ns=self._robot_ns)
		self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
		self._group = moveit_commander.MoveGroupCommander(self._planning_group,
														  robot_description=self._robot_ns + "/robot_description",
														  ns=self._robot_ns)
		self._display_trajectory_publisher = rospy.Publisher(self._robot_ns + '/move_group/display_planned_path',
															 moveit_msgs.msg.DisplayTrajectory, queue_size=1)
		self._exectute_trajectory_client = actionlib.SimpleActionClient(self._robot_ns + '/execute_trajectory',
																		moveit_msgs.msg.ExecuteTrajectoryAction)
		self._exectute_trajectory_client.wait_for_server()

		self._planning_frame = self._group.get_planning_frame()
		self._eef_link = self._group.get_end_effector_link()
		self._group_names = self._robot.get_group_names()
		self._box_name = ''
		self._picked_box_name = []
		self._packages_info = packages_info
		self._conveyor_belt = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
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
		self._pkg_path = rp.get_path('pkg_task5')
		self._file_path = self._pkg_path + '/config/saved_trajectories{}/'.format(self._robot_ns)
		rospy.loginfo("Package Path: {}".format(self._file_path))

		rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

	def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
		"""
			This function is used to load and execute pre-planed trajectories 
		"""
		file_path = arg_file_path + arg_file_name
		#for _ in range(5):
		#	before_joints = self._group.get_current_joint_values()

		with open(file_path, 'r') as file_open:
			loaded_plan = yaml.load(file_open)

		ret = self._group.execute(loaded_plan)

		#for _ in range(5):
		#	after_joints = self._group.get_current_joint_values()

		# This needed only because even if arm completed the trejectory 
		# It was returning Flase with error ABORTED 
		# so hard_play version of this function was trying again and again unnessarlly
		#squared_error = sum([(i - j) ** 2 for i, j in zip(before_joints, after_joints)])
		#if squared_error > 10:  # this means arm has moved enough so trajectory was executed
		#	return True  # calculated by measuring difference square of all the angles
		return ret

	def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
		number_attempts = 0
		flag_success = False

		while (number_attempts <= arg_max_attempts) and (flag_success is False):
			number_attempts += 1
			flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
			rospy.logwarn("attempts: {}".format(number_attempts))
		# self.clear_octomap()

	def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
		"""
			Ensuring Collision Updates Are Received
			^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
			If the Python node dies before publishing a collision object update message, the message
			could get lost and the box will not appear. To ensure that the updates are
			made, we wait until we see the changes reflected in the
			``get_attached_objects()`` and ``get_known_object_names()`` lists.
			We call this function after adding,removing, attaching or detaching an object
			in the planning scene. We then wait until the updates have been made or
			``timeout`` seconds have passed
		"""
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
		box_size = ReqInfo.BoxSize
		box_pose.header.frame_id = self._planning_frame
		self._box_name = box_name
		self._scene.add_box(self._box_name, box_pose, size=box_size)

		return self.wait_for_state_update(box_is_known=True, timeout=timeout)

	def attach_box(self, timeout=4):
		"""
			Attaching Objects to the Robot
		"""
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
		"""
			 Picks the box in both gazebo and rviz from position pick_pose
		"""
		# self.go_to_pose(ReqInfo.HomePose)
		if not self._flag_pickable:
			self.set_conveyor_power(90)
		while not self._flag_pickable:
			rospy.sleep(0.1)
		self.add_box(self._pickable_name, ReqInfo.BoxPose)
		self.attach_box()
		self._flag_pickable = 0
		return self._pickable_name

	def place_box(self):
		"""
			Places the box in both gazebo and rviz to position pick_pose
		"""
		threading.Thread(name="worker", target=self.set_conveyor_power, args=(90, 1)).start()
		pkg_color = self._packages_info[self._picked_box_name[-1]]
		self.moveit_hard_play_planned_path_from_file(self._file_path, 'home_to_{}Bin.yaml'.format(pkg_color), 5)
		self.detach_box()
		self.remove_box()

	def set_conveyor_power(self, power, delay=-1):
		"""
			Sets conveyor belt power to `power` after delay of `delay`
		"""
		rospy.sleep(delay)
		for _ in range(5):
			try:
				self._conveyor_belt(power)
				break
			except rospy.service.ServiceException as error:
				rospy.logerr(error)
				rospy.logwarn("Retrying")

	def cam_callback(self, cam):
		"""
			Logical camera 2 callback function
		"""
		for model in cam.models:
			if model.type[:-2] == 'packagen':
				package_name = model.type
				if package_name in self._picked_box_name:
					break

				if abs(model.pose.position.y) <= 0.05:
					self._pickable_name = package_name
					self._picked_box_name.append(self._pickable_name)
					self.set_conveyor_power(0)
					self._flag_pickable = 1
				else:
					self._flag_pickable = 0
				break

	# Destructor
	def __del__(self):
		moveit_commander.roscpp_shutdown()
		rospy.loginfo(
			'\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


class ActionClientRosIoTBridge:

	# Constructor
	def __init__(self):

		# Initialize Action Client
		self._ac = actionlib.ActionClient('/action_ros_iot',
										  msgRosIotAction)

		# list to Store all the goal handels
		self._goal_handles = []

		# Store the MQTT Topic on which to Publish in a variable
		param_config_pyiot = rospy.get_param('config_pyiot')
		self._config_mqtt_pub_topic = param_config_pyiot['mqtt']['topic_pub']
		self._config_spread_sheet_id = param_config_pyiot['google_apps']['spread_sheet_id']
		self._config_submission_spread_sheet_id = param_config_pyiot['google_apps']['submission_spread_sheet_id']
		# Wait for Action Server that will use the action - '/action_ros_iot' to start
		self._ac.wait_for_server()
		rospy.loginfo("Action server ROS Iot Bridge is up, we can send goals.")

	def on_transition(self, goal_handle):
		"""
			This function will be called when there is a change of state in the Action Client State Machine
		"""
		# from on_goal() to on_transition(). goal_handle generated by send_goal() is used here.

		result = msgRosIotResult()

		index = self._goal_handles.index(goal_handle)
		rospy.loginfo("Transition Callback. Client Goal Handle #: " + str(index))
		rospy.loginfo("Comm. State: " + str(goal_handle.get_comm_state()))
		rospy.loginfo("Goal Status: " + str(goal_handle.get_goal_status()))

		# Comm State - Monitors the State Machine of the Client which is different from Server's
		# Comm State = 2 -> Active
		# Comm State = 3 -> Wating for Result
		# Comm State = 7 -> Done

		# if (Comm State == ACTIVE)
		if goal_handle.get_comm_state() == 2:
			rospy.loginfo(str(index) + ": Goal just went active.")

		# if (Comm State == DONE)
		if goal_handle.get_comm_state() == 7:
			rospy.loginfo(str(index) + ": Goal is DONE")
			rospy.loginfo(goal_handle.get_terminal_state())

			# get_result() gets the result produced by the Action Server
			result = goal_handle.get_result()
			rospy.loginfo(result.flag_success)

			if result.flag_success == True:
				rospy.loginfo("Goal successfully completed. Client Goal Handle #: " + str(index))
			# self._goal_handles.remove(goal_handle)
			# rospy.loginfo("Client Goal Handle #: {} removed".format(index))
			else:
				rospy.loginfo("Goal failed. Client Goal Handle #: " + str(index))

	def send_goal(self, arg_protocol, arg_mode, arg_topic, arg_message):
		"""
			This function is used to send Goals to Action Server
		"""
		# Create a Goal Message object
		goal = msgRosIotGoal()

		goal.protocol = arg_protocol
		goal.mode = arg_mode
		goal.topic = arg_topic
		goal.message = arg_message

		rospy.loginfo("Send goal.")

		# self.on_transition - It is a function pointer to a function which will be called when 
		#                       there is a change of state in the Action Client State Machine
		goal_handle = self._ac.send_goal(goal,
										 self.on_transition,
										 None)
		self._goal_handles.append(goal_handle)
		return goal_handle


class ReqInfo:
	VacuumGripperWidth = 0.115  # Vacuum Gripper Width
	BoxSize = (0.15, 0.15, 0.15)  # cube of sides 0.15 units
	Delta = VacuumGripperWidth + (BoxSize[0] / 2) + 0.01  # 0.19

	BoxPose = geometry_msgs.msg.PoseStamped()
	BoxPose.pose.position.x = -0.8
	BoxPose.pose.position.y = 0
	BoxPose.pose.position.z = 1
	BoxPose.pose.orientation.w = 1.0


def main():
	rospy.init_node('node_ur5_2', anonymous=True)
	try:
		while not rospy.has_param('/packages_color_info'):
			rospy.sleep(0.4)
	except KeyboardInterrupt:
		print('Quiting loop')
	packages_color_info = rospy.get_param('/packages_color_info')
	print(packages_color_info)
	ur5_2 = Ur5Moveit('ur5_2', packages_color_info)
	ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, 'allZeros_to_home.yaml', 5)
	rospy.Subscriber('eyrc/vb/logical_camera_2', LogicalCameraImage, ur5_2.cam_callback)
	iot_client = ActionClientRosIoTBridge()

	i = 0
	while not rospy.is_shutdown() and i < 9:
		shipping_package = ur5_2.pick_box()
		ur5_2.place_box()
		dispatched_order = rospy.get_param("/packages_dispatch_info/{}".format(shipping_package))

		datetime_stamp = datetime.datetime.now()
		dispatched_order["Dispatch Date and Time"] = datetime_stamp.strftime("%Y-%m-%d %H:%M:%S")

		if dispatched_order["Priority"] == "HP":
			est = datetime_stamp.date() + datetime.timedelta(1)
		elif dispatched_order["Priority"] == "MP":
			est = datetime_stamp.date() + datetime.timedelta(3)
		else:
			est = datetime_stamp.date() + datetime.timedelta(5)
		dispatched_order["Estimated Time of Delivery"] = est.strftime("%Y-%m-%d")

		shipped_order = str(dispatched_order).replace("Dispatch", "Shipped")

		iot_client.send_goal("http", "OrdersShipped", iot_client._config_spread_sheet_id, shipped_order)
		iot_client.send_goal("http", "OrdersShipped", iot_client._config_submission_spread_sheet_id, shipped_order)

		pkg_color = packages_color_info[shipping_package]
		ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, '{}Bin_to_home.yaml'.format(pkg_color), 5)
		i += 1

	del ur5_2


if __name__ == '__main__':
	main()
