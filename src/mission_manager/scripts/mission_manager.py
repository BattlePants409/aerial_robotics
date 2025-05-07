#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from enum import Enum  # PY2 COMPAT: enum34 back‑port
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class MissionState(Enum):
    IDLE = 0
    TAKEOFF = 1
    NAVIGATE = 2
    LANDING = 3
    COMPLETE = 4


class MissionManager:
    def __init__(self):
        # anonymous=False so node name is predictable
        rospy.init_node("mission_manager", anonymous=False)
        self.state = None
        self.current_state = MissionState.IDLE

        # ───── Publishers & Subscribers ─────
        #  NAMESPACE FIX → relative names (no leading slash)
        self.local_pos_pub = rospy.Publisher(
            "mavros/setpoint_position/local", PoseStamped, queue_size=10)
        rospy.Subscriber("mavros/state", State, self.state_cb)

        # ───── Services (relative) ─────
        rospy.wait_for_service("mavros/cmd/arming")
        rospy.wait_for_service("mavros/set_mode")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        # Pose message & timing
        self.target_pose = PoseStamped()
        self.rate = rospy.Rate(20)

        # Live pose subscriber for real feedback
        self.current_pose = None
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.pose_cb)

        # Mission targets
        self.takeoff_height = 3.0
        self.gps_position = (10.0, 5.0)  # Placeholder local‑frame target

        # Begin main loop
        self.run_mission()

    # ────────────────────────────────────────────────
    # Callbacks / helpers
    # ────────────────────────────────────────────────
    def state_cb(self, msg):
        self.state = msg

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def publish_position(self, x, y, z):
        self.target_pose.header.stamp = rospy.Time.now()
        self.target_pose.header.frame_id = "map"
        self.target_pose.pose.position.x = x
        self.target_pose.pose.position.y = y
        self.target_pose.pose.position.z = z
        self.local_pos_pub.publish(self.target_pose)

    def reached_altitude(self, target_z, tol=0.2):
        if self.current_pose is None:
            return False
        return abs(self.current_pose.position.z - target_z) < tol

    def reached_position(self, tx, ty, tol=0.5):
        if self.current_pose is None:
            return False
        cx, cy = self.current_pose.position.x, self.current_pose.position.y
        return abs(cx - tx) < tol and abs(cy - ty) < tol

    # ────────────────────────────────────────────────
    # Main mission loop
    # ────────────────────────────────────────────────
    def run_mission(self):
        rospy.loginfo("Waiting for FCU connection…")
        while not rospy.is_shutdown() and (self.state is None or not self.state.connected):
            self.rate.sleep()
        rospy.loginfo("Connected.")

        # Pre‑flight setpoints
        for _ in range(100):
            self.publish_position(0, 0, self.takeoff_height)
            self.rate.sleep()

        # Arm & enter OFFBOARD
        self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
        self.arming_client(True)

        while not rospy.is_shutdown() and self.current_state != MissionState.COMPLETE:
            if self.current_state == MissionState.IDLE:
                rospy.loginfo("State: IDLE ➜ TAKEOFF")
                self.current_state = MissionState.TAKEOFF

            elif self.current_state == MissionState.TAKEOFF:
                self.publish_position(0, 0, self.takeoff_height)
                if self.reached_altitude(self.takeoff_height):
                    rospy.loginfo("Reached takeoff height ➜ NAVIGATE")
                    self.current_state = MissionState.NAVIGATE

            elif self.current_state == MissionState.NAVIGATE:
                x, y = self.gps_position
                self.publish_position(x, y, self.takeoff_height)
                if self.reached_position(x, y):
                    rospy.loginfo("Reached target ➜ LANDING")
                    self.current_state = MissionState.LANDING

            elif self.current_state == MissionState.LANDING:
                # TODO: Replace with precision‑landing logic
                rospy.loginfo("Landing placeholder ➜ COMPLETE")
                self.current_state = MissionState.COMPLETE

            self.rate.sleep()


if __name__ == "__main__":
    try:
        MissionManager()
    except rospy.ROSInterruptException:
        pass
