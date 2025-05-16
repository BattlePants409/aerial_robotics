#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from enum import Enum
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from apriltag_ros.msg import AprilTagDetectionArray


class MissionState(Enum):
	IDLE = 0
	TAKEOFF = 1
	NAVIGATE = 2
	LANDING = 3
	COMPLETE = 4


class MissionManager:
	def __init__(self):
		rospy.init_node("mission_manager", anonymous=False)

		#ROS Setup
		self.state = None
		self.current_pose = None
		self.current_state = MissionState.IDLE

		# MAVROS
		rospy.Subscriber("mavros/state", State, self.state_cb)
		rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.pose_cb)
		self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
		self.vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
		rospy.wait_for_service("mavros/cmd/arming")
		rospy.wait_for_service("mavros/set_mode")
		self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
		self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

		# Tag detection
		rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_cb)
		self.tag_visible = False
		self.tag_error = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}

		#Parameters
		self.takeoff_height = rospy.get_param("~takeoff_height", 3.0)
		self.landing_height = rospy.get_param("~landing_height", 0.3)
		self.gps_position = tuple(rospy.get_param("~gps_position", [5.0, 5.0]))
		self.pid = rospy.get_param("~pid_gains", {"x": 0.7, "y": 0.7, "z": 0.7, "yaw": 0.4})
		self.rate = rospy.Rate(20)
		self.target_pose = PoseStamped()

		self.run_mission()

	#Callbacks
	def state_cb(self, msg):
		self.state = msg

	def pose_cb(self, msg):
		self.current_pose = msg.pose

	def tag_cb(self, msg):
		if len(msg.detections) == 0:
			self.tag_visible = False
			return

		detection = msg.detections[0]
		pose = detection.pose.pose.pose
		self.tag_visible = True
		self.tag_error["x"] = pose.position.x
		self.tag_error["y"] = pose.position.y
		self.tag_error["z"] = pose.position.z
		self.tag_error["yaw"] = pose.orientation.z  # Approximate

	#Helpers
	def publish_position(self, x, y, z):
		self.target_pose.header.stamp = rospy.Time.now()
		self.target_pose.header.frame_id = "map"
		self.target_pose.pose.position.x = x
		self.target_pose.pose.position.y = y
		self.target_pose.pose.position.z = z
		self.local_pos_pub.publish(self.target_pose)

	def publish_velocity(self):
		cmd = TwistStamped()
		cmd.header.stamp = rospy.Time.now()
		cmd.twist.linear.x = -self.pid["x"] * self.tag_error["x"]
		cmd.twist.linear.y = -self.pid["y"] * self.tag_error["y"]
		cmd.twist.linear.z = -self.pid["z"] * (self.tag_error["z"] - 0.5)
		cmd.twist.angular.z = -self.pid["yaw"] * self.tag_error["yaw"]
		self.vel_pub.publish(cmd)

	def reached_target(self, tol=0.5):
		if self.current_pose is None or self.target_pose is None:
			return False

		dx = self.current_pose.position.x - self.target_pose.pose.position.x
		dy = self.current_pose.position.y - self.target_pose.pose.position.y
		dz = self.current_pose.position.z - self.target_pose.pose.position.z

		# Squared distance compared to squared tolerance to avoid sqrt every time
		dist = dx * dx + dy * dy + dz * dz
		print("Distance to target:", dist, "Tolerance:", tol * tol)
		return dist < tol * tol

	def set_mode(self, mode):
		self.set_mode_client(base_mode=0, custom_mode=mode)

	#Main Loop
	def run_mission(self):
		rospy.loginfo("Waiting for FCU connection...")
		while not rospy.is_shutdown() and (self.state is None or not self.state.connected):
			self.rate.sleep()
		rospy.loginfo("Connected to FCU.")

		rospy.loginfo("Waiting for GPS fix...")
		for _ in range(100):
			self.publish_position(0, 0, self.takeoff_height)
			self.rate.sleep()
		rospy.loginfo("GPS fix acquired.")

		self.set_mode("OFFBOARD")
		self.arming_client(True)

		while not rospy.is_shutdown() and self.current_state != MissionState.COMPLETE:
			if self.current_state == MissionState.IDLE:
				rospy.loginfo("State: IDLE → TAKEOFF")
				self.current_state = MissionState.TAKEOFF

			elif self.current_state == MissionState.TAKEOFF:
				self.publish_position(0, 0, self.takeoff_height)
				if self.reached_target():
					rospy.loginfo("Reached takeoff height → NAVIGATE")
					self.current_state = MissionState.NAVIGATE

			elif self.current_state == MissionState.NAVIGATE:
				x, y = self.gps_position
				self.publish_position(x, y, 60)
				if self.reached_target(2):
					rospy.loginfo("Reached GPS target → LANDING")
					self.current_state = MissionState.LANDING

			elif self.current_state == MissionState.LANDING:
				if self.tag_visible:
					self.publish_velocity()
				else:
					rospy.loginfo("Tag lost — switching to QLAND")
					self.set_mode("QLAND")
					self.current_state = MissionState.COMPLETE

			self.rate.sleep()


if __name__ == "__main__":
	try:
		MissionManager()
	except rospy.ROSInterruptException:
		pass
