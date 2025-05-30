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

		self.drone.publish_position(x, y, z)
		#self.drone.publish_velocity_to_target(tx, ty, tz)

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

		# Log distance periodically
		current_time = rospy.Time.now().to_sec()
		if int(current_time) % 5 == 0 and current_time % 1 < 0.1:
			rospy.loginfo("Current target: (%.2f, %.2f, %.2f)" % (tx, ty, tz))
			rospy.loginfo("Distance to target: %.2fm" % dist)

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

	def change_state(self, new_state, msg=""):
		"""Change the current state and log the transition"""
		if new_state != self.current_state:
			rospy.loginfo(msg)
			self.current_state = new_state
			self.target_reached = False

	def handle_idle_state(self):
		"""Handle the IDLE state - set the flight mode"""

		if self.drone.set_mode("GUIDED"):
			self.change_state(MissionState.ARMING, "GUIDED mode set")
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
			self.change_state(MissionState.ARMING, "Disarmed, returning to ARMING")
			return

		elif self.target_reached:
			self.drone.set_mode("AUTO")
			self.change_state(MissionState.NAVIGATE, "Takeoff complete")

		elif self.current_target is None:
			self.set_target_position(0, 0, self.takeoff_height, tolerance=0.5)

			if self.drone.takeoff(self.takeoff_height):
				rospy.loginfo("Takeoff accepted")
			else:
				self.current_target = None
				rospy.logwarn("Takeoff rejected, retrying...")
				self.rate.sleep()

	def handle_navigate_state(self):
		"""Handle the NAVIGATE state - fly to GPS target"""

		if self.target_reached:
			self.drone.set_mode("QLAND")
			self.change_state(MissionState.LANDING, "Navigation complete")

		elif self.current_target is None:
			x, y, z = self.gps_position
			self.set_target_position(x, y, z, tolerance=2.0)

	def handle_landing_state(self):
		"""Handle the LANDING state - land using tag or fallback"""

		if not self.drone.tag_visible:
			x, y, z = self.drone.get_current_position()
			if z < self.landing_height:
				rospy.loginfo("Landing complete")
				self.current_state = MissionState.COMPLETE
				return
			return

		tag_offset = self.drone.get_tag_offset()
		if tag_offset is None:
			rospy.logwarn("Tag offset not available, hovering...")
			self.drone.publish_velocity(0, 0, 0)
			return

		x_err, y_err, z_err = tag_offset
		altitude = self.drone.get_current_altitude()

		if altitude is None:
			rospy.logwarn("Unable to get altitude, hovering...")
			self.drone.publish_velocity(0, 0, 0)
			return

    	# Calculate descent rate based on altitude
		descend_rate = min(-0.2, -0.4 * (altitude / self.takeoff_height))

    	# Simple proportional control to center over tag
		vx = -0.3 * x_err
		vy = -0.3 * y_err
		vz = descend_rate

		self.drone.publish_velocity(vx, vy, vz)

    	# Landing condition
		if abs(x_err) < 0.1 and abs(y_err) < 0.1 and altitude < self.landing_height:
			rospy.loginfo("Landed successfully over tag")
			self.drone.set_mode("QLAND")
			self.current_state = MissionState.COMPLETE

	def run_mission(self):
		"""Main mission execution loop"""
		if not self.wait_for_fcu_connection():
			return

		self.send_initial_setpoints()

		while not rospy.is_shutdown() and self.current_state != MissionState.COMPLETE:
			self.drone.disable_manual_input()
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
