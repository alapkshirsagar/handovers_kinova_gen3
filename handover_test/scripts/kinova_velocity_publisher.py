#!/usr/bin/env python

import rospy
from kinova_msgs.msg import PoseVelocity
from config import HandoverConfig


class KinovaCartesianVelocityPublisher():
    def __init__(self):
        # ######################### Fields #####################################
        self.config = HandoverConfig()
        self.target_velocity = PoseVelocity()

        # ################### Subscribers ####################################
        self.get_velocity_commands = rospy.Subscriber(self.config.target_velocity_pub_topic, PoseVelocity, self.arm_velocity_callback)

        # ################### Publishers ####################################
        self.arm_velocity_publisher = rospy.Publisher(self.config.arm_velocity_pub_topic, PoseVelocity, queue_size=10)

        # ################### Actions ######################################


        # ################### Services ####################################

        # ################## Initialization procedure #####################
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.arm_velocity_publisher.publish(self.target_velocity)
            rate.sleep()

    # 100Hz loop for cartesian velocity control
    def arm_velocity_callback(self,velocity):
        self.target_velocity = velocity


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('kinova_cartesian_velocity_publisher')
    try:
        velocity_publisher = KinovaCartesianVelocityPublisher()
    except rospy.ROSInterruptException:
        pass
