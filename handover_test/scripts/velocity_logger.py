#! /usr/bin/env python
"""A program to log commanded and actual robot velocities"""

import rospy
from kortex_driver.srv import *
from kortex_driver.msg import TwistCommand
from config import HandoverConfig
from geometry_msgs.msg import PoseStamped
import numpy as np
# Class to record trajectories from demonstration


class velocity_logger():
    def __init__(self):
        # #########################Fields#####################################
        self.config = HandoverConfig()
        self.commanded_velocity = []
        self.actual_velocity = [0,0,0]
        self.previous_position = []
        self.actual_position = []
        self.i = 0 #for initial timestep
        self.looprate = 100
        rate = rospy.Rate(self.looprate)

        # ################### Subscribers ####################################
        self.commanded_velocity_subscriber = rospy.Subscriber(self.config.arm_velocity_pub_topic, TwistCommand, self.commanded_velocity_callback)
        self.actual_velocity_subscriber = rospy.Subscriber("/mocap_node/kinova_gripper/pose", PoseStamped, self.actual_velocity_callback)


        while not rospy.is_shutdown():
            rate.sleep()
            


        # ################### Publishers ####################################

        # ################### Actions ######################################

        # ################### Services ####################################


    # ################### Action Clients ###################################

    # ################### Service Clients ##################################


    # ################### Callback Functions ###############################
    def commanded_velocity_callback(self, message):
        commanded_velocity = np.linalg.norm([message.twist.linear_x, message.twist.linear_y, message.twist.linear_z])
        actual_velocity = np.linalg.norm([self.actual_velocity[0], self.actual_velocity[1], self.actual_velocity[2]])
        print("cv",commanded_velocity)
        print("av", actual_velocity)
    # ################### Methods ###########################################
    def actual_velocity_callback(self, message):
        #use actual position as previous position for initial timestep
        if self.i==0:
            self.actual_position = [message.pose.position.x, message.pose.position.y, message.pose.position.z]
            self.previous_position = self.actual_position
            self.i = 1
        else:
            self.previous_position = self.actual_position
            self.actual_position = [message.pose.position.x, message.pose.position.y, message.pose.position.z]
        self.actual_velocity[0] = (self.actual_position[0] - self.previous_position[0])*self.looprate
        self.actual_velocity[1] = (self.actual_position[1] - self.previous_position[1])*self.looprate
        self.actual_velocity[2] = (self.actual_position[2] - self.previous_position[2])*self.looprate

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('velocity_tester', anonymous=False)

    try:
        velocity_log = velocity_logger()
    except rospy.ROSInterruptException:
        pass
