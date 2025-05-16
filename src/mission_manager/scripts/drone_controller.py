#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from apriltag_ros.msg import AprilTagDetectionArray

class DroneController:
	"""Handles ROS-based interactions with the drone hardware"""

	def __init__(self):
		"""Initialize ROS connections and drone interface variables"""
		# ROS setup
		self.state = None
		self.extended_state = None
		self.current_pose = None
		self.arm_time = None
		self.mode_set_time = None

		# MAVROS connections
		rospy.Subscriber("mavros/state", State, self.state_cb)
		rospy.Subscriber("mavros/extended_state", ExtendedState, self.extended_state_cb)
		rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.pose_cb)
		self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
		self.vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

		# AprilTag connections
		rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_cb)
		self.tag_visible = False
		self.tag_error = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}

		# Service clients
		rospy.loginfo("Waiting for MAVROS services...")
		rospy.wait_for_service("mavros/cmd/arming")
		rospy.wait_for_service("mavros/set_mode")
		rospy.wait_for_service("mavros/cmd/takeoff")
		self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
		self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
		self.takeoff_client = rospy.ServiceProxy("mavros/cmd/takeoff", CommandTOL)
		rospy.loginfo("MAVROS services available")

		# Message templates
		self.target_pose = PoseStamped()

	# Callbacks
	def state_cb(self, msg):
		"""Callback for MAVROS state messages"""
		self.state = msg

	def extended_state_cb(self, msg):
		"""Callback for MAVROS extended state messages"""
		self.extended_state = msg

	def pose_cb(self, msg):
		"""Callback for local position updates"""
		self.current_pose = msg.pose

	def tag_cb(self, msg):
		"""Callback for AprilTag detections"""
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

	# Control methods
	def limit(self, value, min_value, max_value):
		"""Limit a value between minimum and maximum bounds"""
		return max(min(value, max_value), min_value)

	def publish_position(self, x, y, z):
		"""Publish position setpoint to MAVROS"""
		self.target_pose.header.stamp = rospy.Time.now()
		self.target_pose.header.frame_id = "map"
		self.target_pose.pose.position.x = x
		self.target_pose.pose.position.y = y
		self.target_pose.pose.position.z = z
		self.local_pos_pub.publish(self.target_pose)

	def publish_velocity(self, vx=None, vy=None, vz=None, yaw_rate=0):
		"""Publish velocity commands to MAVROS"""
		cmd = TwistStamped()
		cmd.header.stamp = rospy.Time.now()

		# Use tag errors if velocities not explicitly provided
		if vx is None:
			vx = -self.pid["x"] * self.tag_error["x"]

		if vy is None:
			vy = -self.pid["y"] * self.tag_error["y"]

		if vz is None:
			vz = -self.pid["z"] * self.tag_error["z"]

		if yaw_rate == 0:
			yaw_rate = -self.pid["yaw"] * self.tag_error["yaw"]

		cmd.twist.linear.x = vx
		cmd.twist.linear.y = vy
		cmd.twist.linear.z = vz
		cmd.twist.angular.z = yaw_rate

		self.vel_pub.publish(cmd)

	def publish_velocity_to_target(self, tx, ty, tz, max_speed=1.0):
		"""Publish velocity commands to move toward target"""

		dx = tx - self.current_pose.position.x
		dy = ty - self.current_pose.position.y
		dz = tz - self.current_pose.position.z

		scale = (dx*dx + dy*dy + dz*dz)**0.5 / max_speed
		vx = self.limit(dx * scale, -max_speed, max_speed)
		vy = self.limit(dy * scale, -max_speed, max_speed)
		vz = self.limit(dz * scale, -max_speed, max_speed)

		self.publish_velocity(vx, vy, vz, 0)

	def set_mode(self, mode):
		"""Attempt to set the desired flight mode"""
		rospy.loginfo("Setting mode to %s..." % mode)
		self.mode_set_time = time.time()
		success = self.set_mode_client(base_mode=0, custom_mode=mode)
		return success.mode_sent

	def arm(self):
		"""Attempt to arm the vehicle"""
		rospy.loginfo("Arming vehicle...")
		self.arm_time = time.time()
		success = self.arming_client(True)
		return success.success

	def takeoff(self, altitude):
		"""Explicit takeoff command"""
		rospy.loginfo("Commanding takeoff to %.2fm..." % altitude)
		success = self.takeoff_client(min_pitch=0, yaw=0, latitude=0, longitude=0, altitude=altitude)
		return success.success

	def is_connected(self):
		"""Check if FCU is connected"""
		return self.state is not None and self.state.connected

	def is_armed(self):
		"""Check if drone is armed"""
		return self.state is not None and self.state.armed

	def get_current_altitude(self):
		"""Get current altitude of the drone"""
		if self.current_pose is not None:
			return self.current_pose.position.z
		return None

	def get_current_position(self):
		"""Get current position (x, y, z) of the drone"""
		if self.current_pose is not None:
			return (
				self.current_pose.position.x,
				self.current_pose.position.y,
				self.current_pose.position.z
			)
		return None

	def calculate_distance_to(self, x, y, z):
		"""Calculate distance to a target position"""
		if self.current_pose is None:
			return float('inf')

		dx = x - self.current_pose.position.x
		dy = y - self.current_pose.position.y
		dz = z - self.current_pose.position.z

		return (dx*dx + dy*dy + dz*dz) ** 0.5
