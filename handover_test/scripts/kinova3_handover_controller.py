#!/usr/bin/env python

import rospy
import actionlib
from kortex_driver.srv import *
from kortex_driver.msg import *
import tf
from std_msgs.msg import String
import math
import numpy as np
from config import HandoverConfig
from scipy.special import erf
from sklearn.svm import SVR
import matplotlib.pyplot as plt
import time
# import cvxpy
from scipy.linalg import toeplitz
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import Header
from geometry_msgs.msg import WrenchStamped, Point, Quaternion, PoseStamped
import cvxpy
from kinova_controller import KinovaController


class KinovaHandoverController:
    def __init__(self, control_type):
        # ######################### Fields #####################################
        self.config = HandoverConfig()
        self.robot_name = rospy.get_param('~robot_name', "my_gen3")
        self.kinova_controller = KinovaController()
        self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

        # ################### Subscribers ####################################

        # ################### Publishers ####################################

        # ################### Actions ######################################

        # ################### Services ####################################

        # ################## Initialization procedure #####################
        if rospy.has_param('pid_kp'):
            self.config.pid_kp = rospy.get_param('pid_kp')
        if rospy.has_param('dmp_kp'):
            self.config.dmp_kp = rospy.get_param('dmp_kp')
        if rospy.has_param('dmp_kd'):
            self.config.dmp_kd = rospy.get_param('dmp_kd')
        if rospy.has_param('dmp_mu'):
            self.config.dmp_mu = rospy.get_param('dmp_mu')
        if rospy.has_param('stl_reach_time'):
            self.config.stl_reach_time = rospy.get_param('stl_reach_time')
        if rospy.has_param('control_type'):
            self.config.control_type = rospy.get_param('control_type')
        print("Control Type:", control_type)

        # print(self.config.control_type == 'dmp')
        if control_type == 'pid':
            self.handover_controller = PIDVelocityController(config=self.config, kinova_controller=self.kinova_controller)
        # elif self.config.control_type is 'stl':
        #     self.stl_velocity_controller.perform_handover(1)
        elif control_type == 'dmp':
            self.handover_controller = DMPVelocityController(config=self.config, kinova_controller=self.kinova_controller)
        elif control_type == 'mpc':
            self.handover_controller = MPCVelocityController(config=self.config, kinova_controller=self.kinova_controller)

        else:
            print("Incorrect controller type. Change the parameter config.control_type to either pid or mpc or dmp")


    def pickup_object(self, pickup_location, pickup_orientation):
        success = self.kinova_controller.is_init_success
        if success:
            # Make sure to clear the robot's faults else it won't move if it's already in fault
            success &= self.kinova_controller.clear_faults_client()
            #*******************************************************************************

            #*******************************************************************************
            # Activate the action notifications
            success &= self.kinova_controller.subscribe_to_a_robot_notification()
            #*******************************************************************************

            #*******************************************************************************
            # Move the robot to the Home position with an Action
            # success &= self.kinova_controller.home_the_robot()
            #*******************************************************************************

            #*******************************************************************************
            # Set the reference frame to "Mixed"
            success &= self.kinova_controller.set_cartesian_reference_frame_client()


            # Hover over object
            rospy.sleep(1.0)

            ## Hover over pickup location
            self.kinova_controller.action_arm_pose(pickup_location + [0,0,0.1], self.config.object_orientation)

            ## Go down
            self.kinova_controller.action_arm_pose(pickup_location, self.config.object_orientation)

            ## Close gripper
            self.kinova_controller.send_gripper_command_client(self.config.gripper_close)

            ## Go up
            self.kinova_controller.action_arm_pose(pickup_location + [0,0,0.1], self.config.object_orientation)



class PIDVelocityController(object):
    def __init__(self, config=None, kinova_controller = None, **kwargs):
        super(PIDVelocityController, self).__init__(**kwargs)
        self.config = config
        self.kinova_controller = kinova_controller
        self.velocity_publisher = rospy.Publisher(self.config.arm_velocity_pub_topic, TwistCommand, queue_size=10)
        self.tf_listener = tf.TransformListener()
        self.soundHandle = SoundClient()
        rospy.sleep(1.0)
        self.soundHandle.stopAll()


    def robot_move_proportional(self, goal, pid_kp):
        reached = False
        rate = rospy.Rate(self.config.pid_loop_rate)

        while not rospy.is_shutdown() and not reached:
            try:
                (trans_robot, rot_robot) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_gripper, rospy.Time(0))
                # (trans_object, rot_object) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_object, rospy.Time(0))
                (trans_human, rot_human) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_human_hand, rospy.Time(0))
                # print("Human position =", trans_human)
                # print("Robot position =", trans_robot)
                self.check_human_safety_zone(trans_human)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print 'Could not find transform'
                continue

            if goal == 'human_hand':
                target = np.array((trans_human)) + self.config.offset_gripper_object #difference = math.sqrt((trans_object[0]-trans_robot[0])**2+(trans_object[1]-trans_robot[1])**2+(trans_object[2]-trans_robot[2])**2)
                kp = pid_kp
                threshold = self.config.pid_distance_threshold
            else:
                target = goal
                kp = self.config.retreat_kp
                threshold = self.config.retreat_threshold

            #print(target)
            difference = math.sqrt((target[0]-trans_robot[0])**2+(target[1]-trans_robot[1])**2+(target[2]-trans_robot[2])**2)
            #print difference
            velocity = min(kp*difference, self.config.max_velocity)
            # print(velocity)
            if difference < threshold:
                velocity = 0.0
                reached = True

            velocity_x = velocity*(target[0]-trans_robot[0])/difference
            velocity_y = velocity*(target[1]-trans_robot[1])/difference
            velocity_z = velocity*(target[2]-trans_robot[2])/difference
            #print(velocity_x, velocity_y, velocity_z)

            velocity_message = TwistCommand()
            velocity_message.twist.linear_x = velocity_x
            velocity_message.twist.linear_y = velocity_y
            velocity_message.twist.linear_z = velocity_z
            velocity_message.duration = 1.0/self.config.pid_loop_rate
            self.velocity_publisher.publish(velocity_message)
            rate.sleep()

    def perform_handover(self, pid_kp, iterations=1):
        # Go to home position
        # print('Going to home position')
        # self.robot_move_proportional(self.config.home_location_optitrack)
        # # self.action_arm_pose(self.config.home_location,self.config.home_orientation)
        # print("Reached home position, waiting for 2 seconds and then opening gripper")


        # Open Gripper, wait for 3 seconds and close gripper
        # rospy.sleep(2.0)
        # self.gripper_controller.gripper_move(self.config.gripper_open)  # fraction between [0,1]
        # print("Opened gripper, waiting for 3 seconds and then closing gripper")
        # rospy.sleep(3.0)
        # # self.gripper_controller.gripper_grasp(self.config.gripper_close, self.config.gripper_velocity, self.config.gripper_force)  # (distance in m [0.01,0.09], velocity in m/s (0,0.2))
        # self.gripper_controller.gripper_move(self.config.gripper_close)  # fraction between [0,1]

        print("Gripper closed, waiting for human to enter handover zone")
        # Go to handover location
        while (not self.check_human_handover_zone()):
            rospy.sleep(0.02)
        print('Human entered handover zone, moving towards human hand')
        self.robot_move_proportional('human_hand', pid_kp)

        # Open gripper
        print('Hand close to gripper, opening gripper')
        self.kinova_controller.send_gripper_command_client(self.config.gripper_open)  # (distance in m [0.01,0.09], velocity in m/s (0,0.2))

        # Go to home position
        print('Object grasped, moving to hover position')
        self.kinova_controller.action_arm_pose(self.config.hover_location,self.config.object_orientation)

        # Open gripper
        # print('Reached home position, opening gripper in 2 seconds')
        # rospy.sleep(2.0)
        # self.gripper_controller.gripper_move(self.config.gripper_open)  # (distance in m [0.01,0.09], velocity in m/s (0,0.2))
        # print('Opened gripper')

    def check_human_handover_zone(self):
        try:
            (trans_robot, rot_robot) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_gripper, rospy.Time(0))
            # (trans_object, rot_object) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_object, rospy.Time(0))
            (trans_human, rot_human) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_human_hand, rospy.Time(0))
            print(trans_human)
            self.check_human_safety_zone(trans_human)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print 'Could not find transform'


        # difference = math.sqrt((trans_human[0]-trans_robot[0])**2+(trans_human[1]-trans_robot[1])**2+(trans_human[2]-trans_robot[2])**2)
        # if difference > self.config.handover_zone_threshold:
        #     return False
        # else:
        #     return True
        if trans_human[0] > self.config.handover_zone_boundaries[0] and trans_human[0] < self.config.handover_zone_boundaries[1] and trans_human[1] > self.config.handover_zone_boundaries[2] and trans_human[1] < self.config.handover_zone_boundaries[3]:
            return True
        else:
            return False

    def check_human_safety_zone(self, trans_human):
        # print("here")
        if trans_human[0] > self.config.safety_zone_boundaries[0] and trans_human[0] < self.config.safety_zone_boundaries[1] and trans_human[1] > self.config.safety_zone_boundaries[2] and trans_human[1] < self.config.safety_zone_boundaries[3]:
            self.soundHandle.play(SoundRequest.NEEDS_PLUGGING)
            rospy.sleep(0.02)


class DMPVelocityController(object):
    def __init__(self, config=None, kinova_controller= None, **kwargs):
        super(DMPVelocityController, self).__init__(**kwargs)
        self.config = config
        self.kinova_controller = kinova_controller
        self.velocity_publisher = rospy.Publisher(self.config.arm_velocity_pub_topic, TwistCommand, queue_size=10)
        self.tf_listener = tf.TransformListener()
        self.clf_x = SVR(C=10.0, epsilon=0.1)
        self.clf_y = SVR(C=10.0, epsilon=0.1)
        self.clf_z = SVR(C=10.0, epsilon=0.1)
        self.soundHandle = SoundClient()
        rospy.sleep(1.0)
        self.soundHandle.stopAll()

        if self.config.dmp_record_trajectory:
            self.trajectory_file = open("human_trajectory.txt", "w")
            self.record_human_trajectory()
        with open("human_trajectory_good.txt") as textFile:
            trajectory = [[float(digit) for digit in line.split(",")] for line in textFile]
        trajectory = np.array(trajectory)
        self.learn_forcing_term(trajectory)

    def record_human_trajectory(self):
        rospy.sleep(2.0)
        rate = rospy.Rate(self.config.dmp_loop_rate)
        print("Start motion")
        t_end = rospy.get_time() + 3
        while rospy.get_time() < t_end:
            (trans_human, rot_human) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_human_hand, rospy.Time(0))
            self.trajectory_file.write(str(trans_human[0])+","+str(trans_human[1])+","+str(trans_human[2])+"\n")
            rate.sleep()
        self.trajectory_file.close()
        print("Stop motion")


    def learn_forcing_term(self, trajectory):
        f_s = np.zeros((trajectory.shape[0] - 1, 3))
        x = np.zeros((trajectory.shape[0] - 1, 3))
        v = np.zeros((trajectory.shape[0] - 1, 3))
        vdot = np.zeros((trajectory.shape[0] - 1, 3))
        target = trajectory[-1,:]
        s = np.ones(trajectory.shape[0] - 1)
        sdot = np.zeros(trajectory.shape[0] - 1)
        t = np.zeros(trajectory.shape[0] - 1)
        w_g = np.zeros(trajectory.shape[0] - 1)

        self.config.dmp_tau = 1.0*trajectory.shape[0]/self.config.dmp_loop_rate
        print("tau", self.config.dmp_tau)
        for timestep in range(0, trajectory.shape[0] - 2):
            t[timestep] = (timestep*1.0)/self.config.dmp_loop_rate
            x[timestep, :] = trajectory[timestep, :]
            v[timestep, :] = (trajectory[timestep + 1, :] - trajectory[timestep, :])*self.config.dmp_loop_rate
            v[timestep+1, :] = (trajectory[timestep + 2, :] - trajectory[timestep+1, :])*self.config.dmp_loop_rate
            vdot[timestep, :] = (v[timestep + 1, :] - v[timestep, :])*self.config.dmp_loop_rate
            sdot[timestep] = 1.0 / self.config.dmp_tau * (-self.config.dmp_alpha * s[timestep])
            w_g[timestep] = 0.5 * (1.0 + erf((t[timestep] - self.config.dmp_mu) / (np.sqrt(2) * self.config.dmp_sigma)))*(0.9+0.1*t[timestep]/self.config.dmp_tau)
            print("Time", t[timestep])
            print("w_g", w_g[timestep])
            f_s[timestep, :] = (1.0 / (1.0 - w_g[timestep])) * (
                        self.config.dmp_tau * vdot[timestep, :] - w_g[timestep] * (self.config.dmp_kp * (target - x[timestep, :])) + self.config.dmp_kd * v[timestep, :]) + x[timestep, :] - x[0, :]
            s[timestep+1] = s[timestep] + sdot[timestep]/self.config.dmp_loop_rate
        timestep = timestep + 1
        t[timestep] = (timestep*1.0)/self.config.dmp_loop_rate
        x[timestep, :] = trajectory[timestep, :]
        v[timestep, :] = (trajectory[timestep + 1, :] -trajectory[timestep, :])*self.config.dmp_loop_rate
        w_g[timestep] = 0.5 * (1.0 + erf((t[timestep] - self.config.dmp_mu) / (np.sqrt(2) * self.config.dmp_sigma)))*(0.9+0.1*t[timestep]/self.config.dmp_tau)

        print("t", t)
        print("x", x)

        # plt.close('all')
        # plt.plot(t, x[:,1])
        # plt.plot(t, v[:,0])
        # plt.plot(t, vdot[:,0])
        # plt.plot(t, sdot)
        # plt.plot(t, s)
        # plt.plot(t, w_g)
        # plt.plot(t, f_s[:,0])
        # plt.plot(s, f_s[:,0])
        print(f_s[:,0].reshape(-1, 1))
        self.clf_x.fit(s.reshape(-1, 1), f_s[:,0].reshape(-1, 1).ravel())
        self.clf_y.fit(s.reshape(-1, 1), f_s[:,1].reshape(-1, 1))
        self.clf_z.fit(s.reshape(-1, 1), f_s[:,2].reshape(-1, 1))
        self.clf_x.get_params()
        self.clf_y.get_params()
        self.clf_z.get_params()

        ##Sample values
        s_test = np.arange(0,1.1, 0.1)
        f_w = np.array([self.clf_x.predict(s_test.reshape(-1, 1)), self.clf_y.predict(s_test.reshape(-1, 1)), self.clf_z.predict(s_test.reshape(-1, 1))])
        print("f_w term", f_w )
        # plt.plot(s_test, f_w[0,:])
        # plt.show()

        # self.test_human_trajectory(trajectory)

    def test_human_trajectory(self, trajectory):
        self.config.dmp_tau = self.config.dmp_tau*2
        dt = 1.0/self.config.dmp_loop_rate
        x = [0.1, 0.3, 0.5]
        target = np.array([0.4, 0.5, 0.5])
        # x = trajectory[0,:]
        # target = trajectory[-1,:]

        x0 = x
        xdot = np.array([0,0,0])
        v = np.array([0,0,0])
        vdot = np.array([0,0,0])
        s = np.array([1.0])
        sdot = 0
        for timestep in range(0, 100):
            t = timestep * dt * 1.0
            w_g = 0.5 * (1.0 + erf((t - self.config.dmp_mu) / (np.sqrt(2.0) * self.config.dmp_sigma)))
            f_w = np.array([self.clf_x.predict(s.reshape(-1, 1)), self.clf_y.predict(s.reshape(-1, 1)), self.clf_z.predict(s.reshape(-1, 1))]).flatten()
            vdot = 1.0/self.config.dmp_tau*((1.0-w_g)*(f_w + x - x0) + w_g* (0.5 * self.config.dmp_kp * (target - x) - self.config.dmp_kd * v))
            xdot = 1.0 / self.config.dmp_tau * (v)
            sdot = 1.0 / self.config.dmp_tau * (-self.config.dmp_alpha * s)
            vdot[0] = min(vdot[0], self.config.max_velocity)
            vdot[1] = min(vdot[1], self.config.max_velocity)
            vdot[2] = min(vdot[2], self.config.max_velocity)

            v = v + vdot * dt
            x = x + xdot * dt
            s = s + sdot * dt
            print(target)
            plt.scatter(timestep * dt, v[0])
        plt.show()

    def robot_move_dmp(self, goal):
        reached = False
        rate = rospy.Rate(self.config.dmp_loop_rate)
        prev_target = 0.0
        velocity = 0.0
        time = 0.0
        s = np.array([1.0])
        state0 = np.array([0.0,0.0,0.0])
        self.config.dmp_tau = self.config.dmp_tau*2

        while not rospy.is_shutdown() and not reached:
            try:
                (trans_robot, rot_robot) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_gripper, rospy.Time(0))
                # (trans_object, rot_object) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_object, rospy.Time(0))
                (trans_human, rot_human) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_human_hand, rospy.Time(0))
                # print("Human position =", trans_human)
                # print("Robot position =", trans_robot)
                if time is 0.0:
                    state0 = trans_robot
                time = time + 1.0/self.config.dmp_loop_rate
                # print("Time", time)
                self.check_human_safety_zone(trans_human)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print 'Could not find transform'
                continue

            if goal == 'human_hand':
                # Compute velocity of human hand
                if prev_target is 0.0:
                    target_velocity = 0.0
                else:
                    target_velocity = [(trans_human[0]-prev_target[0])*self.config.dmp_loop_rate, (trans_human[1]-prev_target[1])*self.config.dmp_loop_rate, (trans_human[2]-prev_target[2])*self.config.dmp_loop_rate]
                prev_target = trans_human
                # Compute target position
                target = np.array((trans_human)) + self.config.offset_gripper_object #difference = math.sqrt((trans_object[0]-trans_robot[0])**2+(trans_object[1]-trans_robot[1])**2+(trans_object[2]-trans_robot[2])**2)
            else:
                target = goal
                target_velocity = [0,0,0]
            w_g = 0.5 * (1.0 + erf((time - self.config.dmp_mu) / (np.sqrt(2) * self.config.dmp_sigma)))
            f_w = np.array([self.clf_x.predict(s.reshape(-1, 1)), self.clf_y.predict(s.reshape(-1, 1)), self.clf_z.predict(s.reshape(-1, 1))]).flatten()
            acceleration = 1.0/self.config.dmp_tau*((1.0-w_g)*(f_w + trans_robot - state0) + w_g*(self.config.dmp_kp*(target-trans_robot) - self.config.dmp_kd*(velocity)))
            velocity = velocity + acceleration/self.config.dmp_loop_rate
            # print("Trans_robot", trans_robot)
            # print("f_w term", f_w + trans_robot - state0)
            # print("goal term", self.config.dmp_kp*(target-trans_robot) - self.config.dmp_kd*(velocity))
            # print("w_g", w_g)
            sdot = -1.0/self.config.dmp_tau*self.config.dmp_alpha*s
            s = s + sdot/self.config.dmp_loop_rate

            velocity_input = velocity/self.config.dmp_tau
            velocity_magnitude = math.sqrt(velocity_input[0]**2+velocity_input[1]**2+velocity_input[2]**2)
            difference = math.sqrt((target[0]-trans_robot[0])**2+(target[1]-trans_robot[1])**2+(target[2]-trans_robot[2])**2)
            # print difference
            velocity_magnitude_floor = min(velocity_magnitude, self.config.max_velocity)
            # print("Acceleration = ", acceleration)
            #print("Velocity =", velocity_input)
            plt.scatter(time, f_w[0])
            if difference < self.config.pid_distance_threshold:
                velocity_magnitude_floor = 0
                reached = True

            velocity_x = velocity_magnitude_floor*(velocity_input[0])/velocity_magnitude
            velocity_y = velocity_magnitude_floor*(velocity_input[1])/velocity_magnitude
            velocity_z = velocity_magnitude_floor*(velocity_input[2])/velocity_magnitude
            velocity = np.array([velocity_x, velocity_y, velocity_z])*self.config.dmp_tau
            print(velocity_x, velocity_y, velocity_z)
            velocity_message = TwistCommand()
            velocity_message.twist.linear_x = velocity_x
            velocity_message.twist.linear_y = velocity_y
            velocity_message.twist.linear_z = velocity_z

            self.velocity_publisher.publish(velocity_message)
            rate.sleep()

    def perform_handover(self, iterations=1):
        # # Go to home position
        # print('Going to home position')
        # self.robot_move_proportional(self.config.home_location_optitrack)
        # print("Reached home position, waiting for 2 seconds and then opening gripper")
        #
        #
        # # Open Gripper, wait for 3 seconds and close gripper
        # rospy.sleep(2.0)
        # self.gripper_controller.gripper_move(self.config.gripper_open)  # (distance in m [0.01,0.09], velocity in m/s (0,0.2))
        # print("Opened gripper, waiting for 3 seconds and then closing gripper")
        # rospy.sleep(3.0)
        # self.gripper_controller.gripper_grasp(self.config.gripper_close)  # (distance in m [0.01,0.09], velocity in m/s (0,0.2))

        print("Gripper closed, waiting for human to enter handover zone")
        # Go to handover location
        while (not self.check_human_handover_zone()):
            rospy.sleep(0.02)
        print('Human entered handover zone, moving towards human hand')
        self.robot_move_dmp('human_hand')

        # Open gripper
        # plt.show(block=False)
        print('Hand close to gripper, opening gripper')
        self.kinova_controller.send_gripper_command_client(self.config.gripper_open)  # (distance in m [0.01,0.09], velocity in m/s (0,0.2))

        # Go to home position
        print('Object grasped, moving to hover position')
        self.kinova_controller.action_arm_pose(self.config.hover_location,self.config.object_orientation)

        # Open gripper
        # print('Reached home position, opening gripper in 2 seconds')
        # rospy.sleep(2.0)
        # self.kinova_controller.send_gripper_command_client(self.config.gripper_open)  # (distance in m [0.01,0.09], velocity in m/s (0,0.2))
        # print('Opened gripper')

    def check_human_handover_zone(self):
        try:
            (trans_robot, rot_robot) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_gripper, rospy.Time(0))
            # (trans_object, rot_object) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_object, rospy.Time(0))
            (trans_human, rot_human) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_human_hand, rospy.Time(0))
            #print(trans_object)
            self.check_human_safety_zone(trans_human)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print 'Could not find transform'


        # # difference = math.sqrt((trans_human[0]-trans_robot[0])**2+(trans_human[1]-trans_robot[1])**2+(trans_human[2]-trans_robot[2])**2)
        # if difference > self.config.handover_zone_threshold:
        #     return False
        # else:
        #     return True
        if trans_human[0] > self.config.handover_zone_boundaries[0] and trans_human[0] < self.config.handover_zone_boundaries[1] and trans_human[1] > self.config.handover_zone_boundaries[2] and trans_human[1] < self.config.handover_zone_boundaries[3]:
            return True
        else:
            return False


    def check_human_safety_zone(self, trans_human):
        # print("here")
        if trans_human[0] > self.config.safety_zone_boundaries[0] and trans_human[0] < self.config.safety_zone_boundaries[1] and trans_human[1] > self.config.safety_zone_boundaries[2] and trans_human[1] < self.config.safety_zone_boundaries[3]:
            self.soundHandle.play(SoundRequest.NEEDS_PLUGGING)
            rospy.sleep(0.02)

class MPCVelocityController(object):
    def __init__(self, config=None, kinova_controller=None, **kwargs):
        super(MPCVelocityController, self).__init__(**kwargs)
        self.config = config
        self.kinova_controller = kinova_controller
        self.velocity_publisher = rospy.Publisher(self.config.arm_velocity_pub_topic, TwistCommand, queue_size=10)
        self.tf_listener = tf.TransformListener()

        self.nx = 3   # number of state
        self.nu = 3   # number of input
        self.delta_t = 0.25  # time tick
        self.L = 32 # Horizon length
        self.t_reach = self.config.stl_reach_time/self.delta_t*1.0
        self.T = 2*self.L + 1 # Receeding window
        self.u_max = self.config.max_velocity # Maximum velocity
        self.x0 = np.array([[0.0], [0.2], [0.3]]) # Initial position
        x_target = np.array([0.2, 0.5, 0.5]) # Target position
        self.epsilon = self.config.pid_distance_threshold # Maximum error in target position
        self.animation = True
        self.soundHandle = SoundClient()
        rospy.sleep(1.0)
        self.soundHandle.stopAll()

    def robot_move_mpc(self, goal, reach_time):
        reached = False
        rate = rospy.Rate(1.0/self.delta_t)
        tracking_available = False
        ## Initialize
        while not tracking_available:
            try:
                (trans_robot, rot_robot) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_gripper, rospy.Time(0))
                # (trans_object, rot_object) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_object, rospy.Time(0))
                (trans_human, rot_human) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_human_hand, rospy.Time(0))
                # print("Human position =", trans_human)
                # print("Robot position =", trans_robot)
                tracking_available = True
                self.check_human_safety_zone(trans_human)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print 'Could not find transform'
                continue

        self.x0 = np.array(trans_robot).reshape((self.nx,1))
        x_old = np.copy(self.x0)*np.ones((self.nx, self.L+1), dtype = float)
        u_old = np.zeros((self.nu, self.L+1), dtype = float)
        i = 0

        while not rospy.is_shutdown() and not reached:
            try:
                (trans_robot, rot_robot) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_gripper, rospy.Time(0))
                # (trans_object, rot_object) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_object, rospy.Time(0))
                (trans_human, rot_human) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_human_hand, rospy.Time(0))
                # print("Human position =", trans_human)
                # print("Robot position =", trans_robot)
                self.check_human_safety_zone(trans_human)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print 'Could not find transform'
                continue

            x_old[:,0:-2] = x_old[:,1:-1]
            x_old[:,-1] = np.array(trans_robot).reshape((self.nx,))


            if goal == 'human_hand':
                target = np.array((trans_human)) + self.config.offset_gripper_object #difference = math.sqrt((trans_object[0]-trans_robot[0])**2+(trans_object[1]-trans_robot[1])**2+(trans_object[2]-trans_robot[2])**2)
                t_reach = reach_time
                threshold = self.config.pid_distance_threshold
            else:
                target = goal
                t_reach = reach_time
                threshold = self.config.retreat_threshold

            print("Running MPC with reach time", t_reach)
            difference = math.sqrt((target[0]-trans_robot[0])**2+(target[1]-trans_robot[1])**2+(target[2]-trans_robot[2])**2)
            #print difference
            ox, oy, oz, oux, ouy, ouz = self.mpc_control(x_old, u_old, target, i)
            if difference < threshold :
                velocity_x = 0.0
                velocity_y = 0.0
                velocity_z = 0.0
                reached = True
            else:
                velocity_x = oux[0]
                velocity_y = ouy[0]
                velocity_z = ouz[0]


            #print(velocity_x, velocity_y, velocity_z)
            velocity_message = TwistCommand()
            velocity_message.twist.linear_x = velocity_x
            velocity_message.twist.linear_y = velocity_y
            velocity_message.twist.linear_z = velocity_z

            self.velocity_publisher.publish(velocity_message)
            u_old[:,0:-2] = u_old[:,1:-1]
            u_old[0,-1] = velocity_x
            u_old[1,-1] = velocity_y
            u_old[2,-1] = velocity_z
            i = i+1
            rate.sleep()

    def mpc_control(self, x_old, u_old, x_target, i):

        x = cvxpy.Variable((self.nx, self.L + 1))
        u = cvxpy.Variable((self.nu, self.L+1))

        cost = self.compute_cost(x,u,x_old, u_old, x_target, i)
        constr = self.compute_constr(x,u, x_old, u_old, x_target, i)
        constr += [x[:,0] == x_old[:,-1]]
        prob = cvxpy.Problem(cvxpy.Minimize(cost), constr)

        start = time.time()
        prob.solve(verbose=False)
        elapsed_time = time.time() - start
        print("calc time:{0} [sec]".format(elapsed_time))
        print("Problem status: %s", prob.status)

        if prob.status == cvxpy.OPTIMAL:
            ox = x.value[0, :]
            oy = x.value[1, :]
            oz = x.value[2, :]

            oux = u.value[0, :]
            ouy = u.value[1, :]
            ouz = u.value[2, :]
            # print(oux)
        else:
            ox = x_old[0,-1]*np.ones((self.T,1), dtype = float)
            oy =  x_old[1,-1]*np.ones((self.T,1), dtype = float)
            oz = x_old[2,-1]*np.ones((self.T,1), dtype = float)

            oux = np.zeros((self.T,), dtype = float)
            ouy = np.zeros((self.T,), dtype = float)
            ouz = np.zeros((self.T,), dtype = float)


        #print(self.x0)
        return ox, oy, oz, oux, ouy, ouz

    def compute_cost(self, x, u, x_old, u_old, x_target, i):
        cost = 0.0

        ## Minimum velocity cost
        # for t in range(T):
        #     cost += cvxpy.quad_form(u[:,t], np.identity(self.nu, dtype = float))
        # cost = cvxpy.max(cvxpy.norm(u,axis=0))

        ## Minimum jerk cost wrt x
        # _row = np.hstack((np.array([[-1.0, 3.0, -3.0, 1.0]]), np.zeros((1,self.T-3))))
        # _col = np.vstack((np.array([[-1.0]]), np.zeros((self.T-1,1))))
        # D = toeplitz(_col, _row)
        # # #jerk = cvxpy.vstack([D*x[0,:].T, D*x[1,:].T, D*x[2,:].T])
        # x = cvxpy.hstack([x_old, x])
        # cost = cvxpy.sum_squares(D*x[0,:]) + cvxpy.sum_squares(D*x[1,:]) + cvxpy.sum_squares(D*x[2,:])

        ## Minimum jerk cost wrt u
        _row = np.hstack((np.array([[1.0, -2.0, 1.0]]), np.zeros((1,self.L-1))))
        _col = np.vstack((np.array([[1.0]]), np.zeros((self.L+1,1))))
        D = toeplitz(_col, _row)
        u = cvxpy.hstack([u_old[:,-1].reshape((self.nu,1)), u])
        cost = cvxpy.sum_squares(D*u[0,:]) + cvxpy.sum_squares(D*u[1,:]) + cvxpy.sum_squares(D*u[2,:])



        return cost

    def compute_constr(self, x, u, x_old, u_old, x_target, i):
        constr = []
        A, B = self.get_model_matrix()
        for t in range(self.L):
            constr += [x[:, t + 1] == A * x[:, t] + B * u[:, t]]
            constr+= [cvxpy.quad_form(u[:,t], np.identity(self.nu, dtype = float)) <= self.u_max*self.u_max]
        if i == 0:
            constr += [u[:,0] == np.zeros((self.nu,))]
        if i < self.t_reach+1:
            constr+= [cvxpy.quad_form(x[:,self.t_reach-i-1]-x_target, np.identity(self.nx, dtype = float)) <= self.epsilon*self.epsilon]
            constr+= [cvxpy.quad_form(u[:,self.t_reach-i-1], np.identity(self.nx, dtype = float)) <= self.epsilon*self.epsilon]
        # print(self.t_reach)
        # for j in range(self.t_reach-i-1,self.L):
        #     constr+= [cvxpy.quad_form(u[:,j], np.identity(nx, dtype = float)) <= self.epsilon*self.epsilon]
        return constr

    def get_nparray_from_matrix(self, x):
        """
        get build-in list from matrix
        """
        return np.array(x).flatten()


    def get_model_matrix(self):

        # Model Parameter
        A = np.array([
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0]
        ])

        A = np.eye(self.nx) + self.delta_t * A

        B = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ])

        B = self.delta_t * B

        return A, B


    def flatten(self, a):
        return np.array(a).flatten()

    def perform_handover(self, reach_time, iterations=1):
        self.t_reach = reach_time/self.delta_t*1.0

        # # Go to home position
        # print('Going to home position')
        # self.robot_move_proportional(self.config.home_location_optitrack)
        # print("Reached home position, waiting for 2 seconds and then opening gripper")
        #
        #
        # # Open Gripper, wait for 3 seconds and close gripper
        # rospy.sleep(2.0)
        # self.gripper_controller.gripper_move(self.config.gripper_open, self.config.gripper_velocity)  # (distance in m [0.01,0.09], velocity in m/s (0,0.2))
        # print("Opened gripper, waiting for 3 seconds and then closing gripper")
        # rospy.sleep(3.0)
        # self.gripper_controller.gripper_grasp(self.config.gripper_close, 0.005, 0.005, self.config.gripper_velocity, self.config.gripper_force)  # (distance in m [0.01,0.09], velocity in m/s (0,0.2))

        print("Gripper closed, waiting for human to enter handover zone")
        # Go to handover location
        while (not self.check_human_handover_zone()):
            rospy.sleep(0.02)
        print('Human entered handover zone, moving towards human hand')
        self.robot_move_mpc('human_hand', reach_time)

        # Open gripper
        print('Hand close to gripper, opening gripper')
        self.kinova_controller.send_gripper_command_client(self.config.gripper_open)  # (distance in m [0.01,0.09], velocity in m/s (0,0.2))

        # Go to home position
        print('Object grasped, moving to hover position')
        self.kinova_controller.action_arm_pose(self.config.hover_location,self.config.object_orientation)

    def check_human_handover_zone(self):
        try:
            (trans_robot, rot_robot) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_gripper, rospy.Time(0))
            # (trans_object, rot_object) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_object, rospy.Time(0))
            (trans_human, rot_human) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_human_hand, rospy.Time(0))
            self.check_human_safety_zone(trans_human)

            #print(trans_human)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print 'Could not find transform'


        # difference = math.sqrt((trans_human[0]-trans_robot[0])**2+(trans_human[1]-trans_robot[1])**2+(trans_human[2]-trans_robot[2])**2)
        # if difference > self.config.handover_zone_threshold:
        #     return False
        # else:
        #     return True
        if trans_human[0] > self.config.handover_zone_boundaries[0] and trans_human[0] < self.config.handover_zone_boundaries[1] and trans_human[1] > self.config.handover_zone_boundaries[2] and trans_human[1] < self.config.handover_zone_boundaries[3]:
            return True
        else:
            return False

    def check_human_safety_zone(self, trans_human):
        # print("here")
        if trans_human[0] > self.config.safety_zone_boundaries[0] and trans_human[0] < self.config.safety_zone_boundaries[1] and trans_human[1] > self.config.safety_zone_boundaries[2] and trans_human[1] < self.config.safety_zone_boundaries[3]:
            self.soundHandle.play(SoundRequest.NEEDS_PLUGGING)
            rospy.sleep(0.02)

    def robot_move_proportional(self, goal):
        reached = False
        rate = rospy.Rate(self.config.pid_loop_rate)

        while not rospy.is_shutdown() and not reached:
            try:
                (trans_robot, rot_robot) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_gripper, rospy.Time(0))
                # (trans_object, rot_object) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_object, rospy.Time(0))
                (trans_human, rot_human) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_human_hand, rospy.Time(0))
                # print("Human position =", trans_human)
                # print("Robot position =", trans_robot)
                self.check_human_safety_zone(trans_human)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print 'Could not find transform'
                continue

            if goal == 'human_hand':
                target = np.array((trans_human)) + self.config.offset_gripper_object #difference = math.sqrt((trans_object[0]-trans_robot[0])**2+(trans_object[1]-trans_robot[1])**2+(trans_object[2]-trans_robot[2])**2)
                kp = self.config.pid_kp
                threshold = self.config.pid_distance_threshold
            else:
                target = goal
                kp = self.config.retreat_kp
                threshold = self.config.retreat_threshold

            #print(target)
            difference = math.sqrt((target[0]-trans_robot[0])**2+(target[1]-trans_robot[1])**2+(target[2]-trans_robot[2])**2)
            #print difference
            velocity = min(kp*difference, self.config.max_velocity)
            print(velocity)
            if difference < threshold:
                velocity = 0.0
                reached = True

            velocity_x = velocity*(target[0]-trans_robot[0])/difference
            velocity_y = velocity*(target[1]-trans_robot[1])/difference
            velocity_z = velocity*(target[2]-trans_robot[2])/difference
            #print(velocity_x, velocity_y, velocity_z)

            velocity_message = TwistCommand()
            velocity_message.twist_linear_x = velocity_x
            velocity_message.twist_linear_y = velocity_y
            velocity_message.twist_linear_z = velocity_z

            self.velocity_publisher.publish(velocity_message)
            rate.sleep()

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('kinova_handover_client')
    try:
        handover_client = KinovaHandoverController()
    except rospy.ROSInterruptException:
        pass
