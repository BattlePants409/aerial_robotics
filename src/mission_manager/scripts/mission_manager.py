#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from enum import Enum

# Import the drone controller class
from drone_controller import DroneController

class MissionState(Enum):
	IDLE = 0
	ARMING = 1
	TAKEOFF = 2
	NAVIGATE = 3
	LANDING = 4
	COMPLETE = 5

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

		# Parameters
		self.takeoff_height = rospy.get_param("~takeoff_height", 2.0)
		self.landing_height = rospy.get_param("~landing_height", 0.3)
		self.gps_position = tuple(rospy.get_param("~gps_position", [5.0, 5.0, 2.0]))
		self.pid = rospy.get_param("~pid_gains", {"x": 0.7, "y": 0.7, "z": 0.7, "yaw": 0.4})
		self.drone.pid = self.pid  # Pass PID values to drone controller

		self.rate = rospy.Rate(20)

		# Begin mission
		self.run_mission()

	def set_target_position(self, x, y, z, tolerance=0.5):
		"""Set a new target position with specified tolerance"""
		self.current_target = (x, y, z)
		self.target_tolerance = tolerance
		self.target_reached = False

		rospy.loginfo("New target set: (%.2f, %.2f, %.2f) with tolerance %.2f" % (x, y, z, tolerance))

	def update_target_status(self):
		"""Check if current target is reached and update status"""
		if self.current_target is None or self.target_reached:
			return False

		# Get drone position
		drone_pos = self.drone.get_current_position()
		if drone_pos is None:
			return False

		tx, ty, tz = self.current_target

		dist = self.drone.calculate_distance_to(tx, ty, tz)

		# Check if reached target position
		if dist < self.target_tolerance:
			self.current_target = None
			self.target_reached = True

			rospy.loginfo("Target reached")
			return True

		self.drone.publish_position(tx, ty, tz)
		self.drone.publish_velocity_to_target(tx, ty, tz)

		# Log distance periodically
		current_time = rospy.Time.now().to_sec()
		if int(current_time) % 5 == 0 and current_time % 1 < 0.1:
			rospy.loginfo("Distance to target: %.2f, Tolerance: %.2f" %(dist ** 0.5, self.target_tolerance))

		return False

	def wait_for_fcu_connection(self):
		"""Wait for the flight controller to connect"""
		rospy.loginfo("Waiting for FCU connection...")

		timeout = rospy.Duration(30)
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

		if self.drone.set_mode("GUIDED"):
			rospy.loginfo("GUIDED mode accepted, moving to ARMING")
			self.current_state = MissionState.ARMING
		else:
			rospy.logwarn("Failed to set GUIDED mode, retrying...")
			self.rate.sleep()

	def handle_arming_state(self):
		"""Handle the ARMING state - arm the vehicle"""
		if not self.drone.is_armed():
			self.drone.arm()
			return

		rospy.loginfo("Vehicle armed, proceeding to TAKEOFF")
		self.current_state = MissionState.TAKEOFF

	def handle_takeoff_state(self):
		"""Handle the TAKEOFF state - takeoff to target altitude"""

		if not self.drone.is_armed():
			rospy.logwarn("Vehicle disarmed, returning to ARMING")
			self.current_state = MissionState.ARMING

		elif self.target_reached:
			rospy.loginfo("Takeoff complete, moving to NAVIGATE")
			self.current_state = MissionState.NAVIGATE

		elif self.current_target is None:
			self.set_target_position(0, 0, self.takeoff_height, tolerance=0.5)

			if self.drone.takeoff(self.takeoff_height):
				rospy.loginfo("Takeoff accepted")
			else:
				self.current_target = None
				rospy.logwarn("Takeoff rejected, retrying...")

	def handle_navigate_state(self):
		"""Handle the NAVIGATE state - fly to GPS target"""

		if self.target_reached:
			rospy.loginfo("GPS target reached, moving to LANDING")
			self.current_state = MissionState.LANDING

		elif self.current_target is None:
			x, y, z = self.gps_position
			self.set_target_position(x, y, z, tolerance=2.0)

	def handle_landing_state(self):
		"""Handle the LANDING state - land using tag or fallback"""

		if not self.drone.tag_visible:
			rospy.loginfo("Tag not visible - switching to QLAND")
			self.drone.set_mode("QLAND")
			self.current_state = MissionState.COMPLETE
			return

		altitude = self.drone.get_current_altitude()
		if altitude is None:
			rospy.logwarn("Unable to get altitude, using default landing behavior")
			self.drone.publish_velocity()
			return

		descend_rate = min(-0.2, -0.4 * (altitude / self.takeoff_height))
		self.drone.publish_velocity(None, None, descend_rate)

		if altitude < self.landing_height:
			rospy.loginfo("Landed successfully at height: %.2fm" % altitude)
			self.drone.set_mode("LAND")
			self.current_state = MissionState.COMPLETE

	def run_mission(self):
		"""Main mission execution loop"""
		if not self.wait_for_fcu_connection():
			return

		self.send_initial_setpoints()

		while not rospy.is_shutdown() and self.current_state != MissionState.COMPLETE:
			self.update_target_status()

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
