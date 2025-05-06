#!/usr/bin/env python3

import rospy
from enum import Enum, auto
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class MissionState(Enum):
    IDLE = auto()
    TAKEOFF = auto()
    NAVIGATE = auto()
    LANDING = auto()
    COMPLETE = auto()

class MissionManager:
    def __init__(self):
        rospy.init_node("mission_manager", anonymous=True)
        self.state = None
        self.current_state = MissionState.IDLE

        # Publishers and Subscribers
        self.local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        rospy.Subscriber("/mavros/state", State, self.state_cb)

        # Services
        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.wait_for_service("/mavros/set_mode")
        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        # Pose message
        self.target_pose = PoseStamped()
        self.rate = rospy.Rate(20)

        # Mission targets
        self.takeoff_height = 3.0
        self.gps_position = (10.0, 5.0)  # Placeholder for GPS target in local frame

        # Begin main loop
        self.run_mission()

    def state_cb(self, msg):
        self.state = msg

    def run_mission(self):
        rospy.loginfo("Waiting for FCU connection...")
        while not rospy.is_shutdown() and (self.state is None or not self.state.connected):
            self.rate.sleep()
        rospy.loginfo("Connected.")

        # Send initial setpoints
        for _ in range(100):
            self.publish_position(0, 0, self.takeoff_height)
            self.rate.sleep()

        # Arm and set to OFFBOARD
        self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
        self.arming_client(True)

        while not rospy.is_shutdown() and self.current_state != MissionState.COMPLETE:
            if self.current_state == MissionState.IDLE:
                rospy.loginfo("State: IDLE -> TAKEOFF")
                self.current_state = MissionState.TAKEOFF

            elif self.current_state == MissionState.TAKEOFF:
                self.publish_position(0, 0, self.takeoff_height)
                if self.reached_altitude(self.takeoff_height):
                    rospy.loginfo("Reached takeoff height. Transitioning to NAVIGATE.")
                    self.current_state = MissionState.NAVIGATE

            elif self.current_state == MissionState.NAVIGATE:
                x, y = self.gps_position
                self.publish_position(x, y, self.takeoff_height)
                if self.reached_position(x, y):
                    rospy.loginfo("Reached target. Transitioning to LANDING.")
                    self.current_state = MissionState.LANDING

            elif self.current_state == MissionState.LANDING:
                # Placeholder: Replace with precision landing logic
                rospy.loginfo("Landing placeholder activated. Mission complete.")
                self.current_state = MissionState.COMPLETE

            self.rate.sleep()

    def publish_position(self, x, y, z):
        self.target_pose.header.stamp = rospy.Time.now()
        self.target_pose.header.frame_id = "map"
        self.target_pose.pose.position.x = x
        self.target_pose.pose.position.y = y
        self.target_pose.pose.position.z = z
        self.local_pos_pub.publish(self.target_pose)

    def reached_altitude(self, target_z, tolerance=0.2):
        current_z = self.target_pose.pose.position.z  # Ideally: get from actual topic
        return abs(current_z - target_z) < tolerance

    def reached_position(self, target_x, target_y, tolerance=0.5):
        current_x = self.target_pose.pose.position.x
        current_y = self.target_pose.pose.position.y
        return abs(current_x - target_x) < tolerance and abs(current_y - target_y) < tolerance

if __name__ == "__main__":
    try:
        MissionManager()
    except rospy.ROSInterruptException:
        pass
