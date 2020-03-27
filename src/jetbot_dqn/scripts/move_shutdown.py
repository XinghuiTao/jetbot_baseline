#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Float64MultiArray
import time


class MoveRobotStopOnShutdown(object):
    def __init__(self):
        self.pub_vel_left = rospy.Publisher('/robot/joint1_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_right = rospy.Publisher('/robot/joint2_velocity_controller/command', Float64, queue_size=1)
        self.msg = Float64MultiArray()

        rospy.on_shutdown(self.clean_shutdown)

        rospy.init_node('move_and_stop_robot')
        self.move_robot()
        rospy.spin()

    def publish(self, msg_type="move"):
        while self.pub_vel_left.get_num_connections() < 1 or self.pub_vel_right.get_num_connections() < 1:
            rospy.loginfo("Waiting for connection to publisher...")
            time.sleep(1)

        rospy.loginfo("Connected to publisher.")

        rospy.loginfo("Publishing %s message..." % msg_type)
        self.pub_vel_left.publish(self.msg.data[0])
        self.pub_vel_right.publish(self.msg.data[1])


    def move_robot(self):
        self.msg.data = [-20.0, -20.0]
        self.publish()
        time.sleep(5)
        self.msg.data = [10.0, 0.0]
        self.publish()
        time.sleep(3)
        self.msg.data = [0.0, 10.0]
        self.publish()
        time.sleep(6)
        self.msg.data = [10.0, 0.0]
        self.publish()
        time.sleep(3)

        rospy.signal_shutdown("Done!")

    def clean_shutdown(self):
        rospy.loginfo("System is shutting down. Stopping robot...")
        self.msg.data = [0.0, 0.0]
        self.publish("stop")

if __name__ == '__main__':
    MoveRobotStopOnShutdown()