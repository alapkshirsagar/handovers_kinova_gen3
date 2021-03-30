#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys
import tf
from config import HandoverConfig
import math 

class DataLogger():
    def __init__(self, participant_id):
        self.config = HandoverConfig()
        self.data_logger_subscriber = rospy.Subscriber('data_logger_signal', String, self.data_logger_callback)
        self.duration_table_publisher = rospy.Publisher('/duration_display', String, queue_size = 10)
        rospy.set_param('phase_number', 1)
        self.design_start_time = rospy.Time.now()
        self.design_end_time = rospy.Time.now()
        self.test_start_time = rospy.Time.now()
        self.test_end_time = rospy.Time.now()
        self.design_duration = self.design_end_time - self.design_start_time
        self.test_duration = self.test_end_time - self.test_start_time
        self.controller_type = 'mpc' #initialize
        self.participant_id = participant_id
        self.data_logging_file = open('src/handovers_kinova_gen3/handover_test/data/data_'+participant_id+".txt", "a")
        self.robot_idle_time = 0.0
        self.human_idle_time = 0.0
        self.number_of_handovers_design = 0
        self.tf_listener = tf.TransformListener()
        self.initialTime = rospy.Time.now()
        self.optitrack_callback()

    def data_logger_callback(self,signal):
        signal_list = signal.data.split(',')
        print(signal_list)
        self.controller_type = signal_list[1]
        timestamp = round((rospy.Time.now()-self.initialTime).to_nsec()/(1.0*10**9),3)
        self.data_logging_file.write(str(self.participant_id)+','+str(self.controller_type)+',' + signal_list[0]+ ',' + str(timestamp)+'\n')
        if signal_list[0] in ['design_start', 'design_end', 'test_start', 'test_end']:
            self.duration_callback(signal)

    def duration_callback(self, signal):
        signal_list = signal.data.split(',')
        self.controller_type = signal_list[1]
        if signal_list[0] == 'design_start':
            self.design_start_time = rospy.Time.now()
            self.design_duration = rospy.Duration.from_sec(0.0)
            self.robot_idle_time = 0.0
            self.human_idle_time = 0.0
            # self.data_logging_file.write(str(self.participant_id)+','+str(self.controller_type)+',' + 'design_duration'+ ',' + str(self.design_duration.to_sec())+'\n')
            
        elif signal_list[0] == 'design_end':
            self.design_end_time = rospy.Time.now()
            self.design_duration = self.design_end_time - self.design_start_time
            self.data_logging_file.write(str(self.participant_id)+','+str(self.controller_type)+',' + 'design_duration'+ ',' + str(round(self.design_duration.to_sec(),2))+'\n')
            self.data_logging_file.write(str(self.participant_id)+','+str(self.controller_type)+',' + 'design_no_handovers'+ ',' + str(self.number_of_handovers_design)+'\n')
            self.number_of_handovers_design = 0
            self.data_logging_file.write(str(self.participant_id)+','+str(self.controller_type)+',' + 'design_robot_idle'+ ',' + str(self.robot_idle_time)+'\n')
            self.robot_idle_time = 0.0
            self.data_logging_file.write(str(self.participant_id)+','+str(self.controller_type)+',' + 'design_human_idle'+ ',' + str(self.human_idle_time)+'\n')
            self.human_idle_time = 0.0

            phase_number = rospy.get_param('phase_number')
            [row,column] = self.get_duration_table_index(phase_number)
            self.duration_table_publisher.publish(str(row)+','+str(column)+','+str(round(self.design_duration.to_sec(),2)))
            rospy.set_param('phase_number', phase_number+1)

            self.design_duration = rospy.Duration.from_sec(0.0)

        elif signal_list[0] == 'test_start':
            # self.design_end_time = rospy.Time.now()
            # self.design_duration = self.design_end_time - self.design_start_time
            # self.data_logging_file.write(str(self.participant_id)+','+str(self.controller_type)+',' + 'design_duration'+ ',' + str(self.design_duration.to_sec())+'\n')
            # self.data_logging_file.write(str(self.participant_id)+','+str(self.controller_type)+',' + 'design_no_handovers'+ ',' + str(self.number_of_handovers_design)+'\n')
            # self.number_of_handovers_design = 0
            # self.data_logging_file.write(str(self.participant_id)+','+str(self.controller_type)+',' + 'design_robot_idle'+ ',' + str(self.robot_idle_time)+'\n')
            # self.robot_idle_time = 0.0
            # self.data_logging_file.write(str(self.participant_id)+','+str(self.controller_type)+',' + 'design_human_idle'+ ',' + str(self.human_idle_time)+'\n')
            # self.human_idle_time = 0.0

            self.test_start_time = rospy.Time.now()
            self.test_duration = rospy.Duration.from_sec(0.0)
            self.robot_idle_time = 0.0
            self.human_idle_time = 0.0

        elif signal_list[0] == 'test_end':
            self.test_end_time = rospy.Time.now()
            self.test_duration = self.test_end_time - self.test_start_time
            self.data_logging_file.write(str(self.participant_id)+','+str(self.controller_type)+',' + 'test_duration'+ ',' + str(round(self.test_duration.to_sec(),2))+'\n')
            self.data_logging_file.write(str(self.participant_id)+','+str(self.controller_type)+',' + 'test_robot_idle'+ ',' + str(self.robot_idle_time)+'\n')
            self.robot_idle_time = 0.0
            self.data_logging_file.write(str(self.participant_id)+','+str(self.controller_type)+',' + 'test_human_idle'+ ',' + str(self.human_idle_time)+'\n')
            self.human_idle_time = 0.0

            phase_number = rospy.get_param('phase_number')
            [row,column] = self.get_duration_table_index(phase_number)
            self.duration_table_publisher.publish(str(row)+','+str(column)+','+str(round(self.test_duration.to_sec(),2)))
            rospy.set_param('phase_number', phase_number+1)

            self.test_duration = rospy.Duration.from_sec(0.0)


        if signal_list[0] == 'handover_start':
            self.number_of_handovers_design+=1

    def get_duration_table_index(self, phase_number):
        column = (phase_number-1)//4
        row = (phase_number-1)%4
        return row, column

    def optitrack_callback(self):
        rate = rospy.Rate(10)
        i = 0
        while not rospy.is_shutdown():
            try:
                (trans_robot, rot_robot) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_gripper, rospy.Time(0))
                (trans_human, rot_human) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_human_hand, rospy.Time(0))
                # print("Human position =", trans_human)
                # print("Robot position =", trans_robot)
                if i is 0:
                    prev_trans_robot = trans_robot
                    prev_trans_human = trans_human
                    i+=1

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print 'Could not find transform'
                continue

            robot_difference = math.sqrt((prev_trans_robot[0]-trans_robot[0])**2+(prev_trans_robot[1]-trans_robot[1])**2+(prev_trans_robot[2]-trans_robot[2])**2)
            human_difference = math.sqrt((prev_trans_human[0]-trans_human[0])**2+(prev_trans_human[1]-trans_human[1])**2+(prev_trans_human[2]-trans_human[2])**2)
            if robot_difference < 0.01:
                self.robot_idle_time+=0.1
            if human_difference < 0.01:
                self.human_idle_time+=0.1
            prev_trans_robot = trans_robot
            prev_trans_human = trans_human
            rate.sleep()



if __name__ == '__main__':
    # Initialize node
    rospy.init_node('data_logger')
    try:
        data_logger = DataLogger(sys.argv[1])
    except rospy.ROSInterruptException:
        data_logger.data_logging_file.close()
        pass


