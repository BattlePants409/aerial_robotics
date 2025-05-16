#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from enum import Enum
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from apriltag_ros.msg import AprilTagDetectionArray

class MissionState(Enum):
	IDLE = 0
	ARMING = 1
	TAKEOFF = 2
	NAVIGATE = 3
	LANDING = 4
	COMPLETE = 5

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

		if yaw_rate is 0:
			yaw_rate = -self.pid["yaw"] * self.tag_error["yaw"]

		cmd.twist.linear.x = vx
		cmd.twist.linear.y = vy
		cmd.twist.linear.z = vz
		cmd.twist.angular.z = yaw_rate

		self.vel_pub.publish(cmd)

	def publish_velocity_to_target(self, dx, dy, dz, max_speed=5):
		"""Publish velocity commands to move toward target"""
		vx = self.limit(dx, 0, max_speed)
		vy = self.limit(dy, 0, max_speed)
		vz = self.limit(dz, 0, max_speed)
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


class MissionManager:
	"""Manages mission state and high-level flight behavior"""

	def __init__(self):
		"""Initialize mission manager"""
		rospy.init_node("mission_manager", anonymous=False)

		# Create the drone controller
		self.drone = DroneController()

		# Mission state variables
		self.current_state = MissionState.IDLE

		# Target tracking
		self.current_target = None
		self.target_reached = False
		self.target_tolerance = 0.5
		self.target_wait_start = None
		self.target_wait_duration = rospy.Duration(2.0)  # Wait 2 seconds at target

		# Parameters
		self.takeoff_height = rospy.get_param("~takeoff_height", 2.0)
		self.landing_height = rospy.get_param("~landing_height", 0.3)
		self.gps_position = tuple(rospy.get_param("~gps_position", [5.0, 5.0, 2.0]))
		self.pid = rospy.get_param("~pid_gains", {"x": 0.7, "y": 0.7, "z": 0.7, "yaw": 0.4})
		self.drone.pid = self.pid  # Pass PID values to drone controller

		# Attempt tracking
		self.arming_attempts = 0
		self.takeoff_attempts = 0
		self.max_attempts = 3

		self.rate = rospy.Rate(20)

		# Begin mission
		self.run_mission()

	def set_target_position(self, x, y, z, tolerance=0.5, wait_time=2.0):
		"""Set a new target position with specified tolerance and wait time"""
		self.current_target = (x, y, z)
		self.target_tolerance = tolerance
		self.target_reached = False
		self.target_wait_start = None
		self.target_wait_duration = rospy.Duration(wait_time)
		rospy.loginfo("New target set: (%.2f, %.2f, %.2f) with tolerance %.2f" % (x, y, z, tolerance))

	def update_target_status(self):
		"""Check if current target is reached and update status"""
		if self.current_target is None:
			return False

		tx, ty, tz = self.current_target

		# Get drone position
		drone_pos = self.drone.get_current_position()
		if drone_pos is None:
			return False

		dx, dy, dz = tx - drone_pos[0], ty - drone_pos[1], tz - drone_pos[2]

		# Squared distance compared to squared tolerance
		dist = dx * dx + dy * dy + dz * dz

		# Check if reached target position
		if dist < self.target_tolerance * self.target_tolerance:
			if not self.target_reached:
				self.target_reached = True
				self.target_wait_start = rospy.Time.now()
				rospy.loginfo("Target reached, holding position for %.1f seconds" %
					   self.target_wait_duration.to_sec())

			# Check if we've waited long enough at the target
			if (rospy.Time.now() - self.target_wait_start) >= self.target_wait_duration:
				return True
		else:
			# Reset target reached status if we drift away
			if self.target_reached:
				rospy.logwarn("Drifted from target, re-approaching")
				self.target_reached = False
				self.target_wait_start = None

			# Publish position and velocity commands to move toward target
			self.drone.publish_position(tx, ty, tz)

			# Calculate velocity commands for smoother approach
			self.drone.publish_velocity_to_target(dx, dy, dz)

		# Log distance periodically
		if rospy.Time.now().to_sec() % 5 < 0.1:  # Log every ~5 seconds
			rospy.loginfo("Distance to target: %.2f, Tolerance: %.2f" %
				   (dist ** 0.5, self.target_tolerance))

		return False

	def wait_for_fcu_connection(self):
		"""Wait for the flight controller to connect"""
		rospy.loginfo("Waiting for FCU connection...")
		timeout = rospy.Duration(30)  # 30 second timeout
		start_time = rospy.Time.now()

		while not rospy.is_shutdown():
			if self.drone.is_connected():
				rospy.loginfo("Connected to FCU.")
				return True

			if (rospy.Time.now() - start_time) > timeout:
				rospy.logerr("Timed out waiting for FCU connection")
				return False

			self.rate.sleep()

		return False

	def send_initial_setpoints(self):
		"""Send initial setpoints before attempting mode changes"""
		rospy.loginfo("Sending initial setpoints...")
		for _ in range(100):
			self.drone.publish_position(0, 0, 0)
			self.rate.sleep()

	def handle_idle_state(self):
		"""Handle the IDLE state - set the flight mode"""
		# Set mode to GUIDED first
		if self.drone.set_mode("GUIDED"):
			rospy.loginfo("GUIDED mode accepted, moving to ARMING")
			self.current_state = MissionState.ARMING
		else:
			rospy.logwarn("Failed to set GUIDED mode, retrying...")
			self.rate.sleep()

	def handle_arming_state(self):
		"""Handle the ARMING state - arm the vehicle"""
		if self.drone.is_armed():
			rospy.loginfo("Vehicle armed, proceeding to TAKEOFF")
			self.current_state = MissionState.TAKEOFF
		else:
			# Retry arming if needed
			if self.drone.arm_time is None or (time.time() - self.drone.arm_time) > 2.0:
				if self.arming_attempts < self.max_attempts:
					self.arming_attempts += 1
					self.drone.arm()
				else:
					rospy.logerr("Failed to arm after %d attempts" % self.max_attempts)
					self.current_state = MissionState.COMPLETE  # End mission if we can't arm

	def handle_takeoff_state(self):
		"""Handle the TAKEOFF state - takeoff to target altitude"""
		# First time in this state, set the target
		if self.current_target is None or self.current_target[2] != self.takeoff_height:
			self.set_target_position(0, 0, self.takeoff_height, tolerance=0.5, wait_time=1.0)

		if self.target_reached and (self.target_wait_start is None):
			# Target reached and wait completed
			rospy.loginfo("Takeoff complete, moving to NAVIGATE")
			self.current_state = MissionState.NAVIGATE
			self.current_target = None  # Clear target for next state
			return

		if not self.drone.is_armed():
			rospy.logwarn("Vehicle disarmed during takeoff! Returning to ARMING")
			self.current_state = MissionState.ARMING
			self.arming_attempts = 0
			self.current_target = None  # Clear target
			return

		# Send explicit takeoff command occasionally as backup
		if self.takeoff_attempts < self.max_attempts:
			if self.takeoff_attempts == 0 or rospy.Time.now().to_sec() % 5 < 0.1:
				self.takeoff_attempts += 1
				if self.drone.takeoff(self.takeoff_height):
					rospy.loginfo("Takeoff command accepted")
				else:
					rospy.logwarn("Takeoff command rejected, retrying...")

	def handle_navigate_state(self):
		"""Handle the NAVIGATE state - fly to GPS target"""
		# First time in this state, set the target
		if self.current_target is None or self.current_target != self.gps_position:
			x, y, z = self.gps_position
			self.set_target_position(x, y, z, tolerance=2.0, wait_time=2.0)

		if self.target_reached and (self.target_wait_start is None):
			# Target reached and wait completed
			rospy.loginfo("GPS target reached, moving to LANDING")
			self.current_state = MissionState.LANDING
			self.current_target = None  # Clear target for next state

	def handle_landing_state(self):
		"""Handle the LANDING state - land using tag or fallback"""
		if self.drone.tag_visible:
			self.drone.publish_velocity()  # Use tag-based velocity

			# Check if we've landed based on altitude
			altitude = self.drone.get_current_altitude()
			if altitude is not None and altitude < self.landing_height:
				rospy.loginfo("Landed successfully at height: %.2fm" % altitude)
				self.drone.set_mode("LAND")  # Ensure motors are off
				self.current_state = MissionState.COMPLETE
		else:
			rospy.loginfo("Tag lost — switching to QLAND")
			self.drone.set_mode("QLAND")
			self.current_state = MissionState.COMPLETE

	def run_mission(self):
		"""Main mission execution loop"""
		if not self.wait_for_fcu_connection():
			return

		self.send_initial_setpoints()

		while not rospy.is_shutdown() and self.current_state != MissionState.COMPLETE:
			# Update target tracking
			if self.current_target is not None:
				reached = self.update_target_status()
				if reached and self.target_wait_start is not None:
					rospy.loginfo("Target hold complete")
					self.target_wait_start = None  # Reset wait timer

			# State machine
			if self.current_state == MissionState.IDLE:
				self.handle_idle_state()
			elif self.current_state == MissionState.ARMING:
				self.handle_arming_state()
			elif self.current_state == MissionState.TAKEOFF:
				self.handle_takeoff_state()
			elif self.current_state == MissionState.NAVIGATE:
				self.handle_navigate_state()
			elif self.current_state == MissionState.LANDING:
				self.handle_landing_state()

			self.rate.sleep()

if __name__ == "__main__":
	try:
		MissionManager()
	except rospy.ROSInterruptException:
		pass
