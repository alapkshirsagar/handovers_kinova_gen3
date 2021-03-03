#!/usr/bin/env python

import rospy
from config import HandoverConfig
import sys
import numpy as np
from kinova3_handover_controller import KinovaHandoverController
from std_msgs.msg import String

class TaskPlanner():
    def __init__(self, mode, controller):
        # ######################### Fields #####################################
        self.config = HandoverConfig()
        self.task_mode = mode
        self.task_controller = controller
        if self.task_mode not in ["trial", "test"]:
            print("Incorrect mode type. Change the mode to either trial or test")
            raise rospy.ROSInterruptException("Incorrect task mode")
        if self.task_controller not in ["mpc", "pid"]:
            print("Incorrect controller type. Change the controller to either pid or mpc or dmp")
            raise rospy.ROSInterruptException("Incorrect controller type")


        self.n_vaccines = 3 ## Number of vaccine types
        self.vaccines_stack = 6 ## Number of vaccines in the stack per type
        self.stack_status = np.zeros((self.n_vaccines,self.vaccines_stack)) ## Keep track of vaccines in the stack. 0 for no vaccine.
        for vaccine in range(self.n_vaccines):
            self.update_stack(vaccine + 1)
        
        self.kinova_handover_controller = KinovaHandoverController(controller)
        self.vaccine_stack_status_publisher = rospy.Publisher('/vaccine_stack_status', String, queue_size=10)
        if self.task_mode == "trial":
            self.trial_handovers()
        else:
            self.test_handovers()


    def trial_handovers(self):
        ## Function to perform handovers in the trial phase. Requires command line argument of the vaccine type
        vaccine_type = 0
        while True:
            vaccine_type = int(raw_input("Please write the vaccine type: "))
            
            if vaccine_type == -1:
                break

            if vaccine_type == 10:
                self.test_handovers()

            pickup_location = self.get_pickup_location(vaccine_type)
            timing_constraints = self.get_timing_constraints(vaccine_type)
            rospy.set_param('timing_constraints', timing_constraints)

            ## Pick-up object
            self.kinova_handover_controller.pickup_object(pickup_location, self.config.object_orientation)

            ## Perform handover
            if self.task_controller == 'pid' and rospy.has_param('pid_kp_'+str(vaccine_type)):
                controller_parameter = rospy.get_param('pid_kp_'+str(vaccine_type))
            elif self.task_controller == 'mpc' and rospy.has_param('mpc_t_'+str(vaccine_type)):
                controller_parameter = rospy.get_param('mpc_t_'+str(vaccine_type))
            self.kinova_handover_controller.handover_controller.perform_handover(controller_parameter)

            vaccine_type = 0

    def test_handovers(self):
        vaccine_stack_status = ""
        for i in range(0,3):
            vaccine_stack_status+=str(np.sum(self.stack_status[i,:]))
        self.vaccine_stack_status_publisher.publish(vaccine_stack_status)

        ## Function to perform handovers in the test phase. Requires no input
        vaccine_test_seq=np.array([1, 2, 3]*self.vaccines_stack )
        # rospy.set_param('vaccine_test_seq')

        for vaccine_type in vaccine_test_seq:
            pickup_location = self.get_pickup_location(vaccine_type)
            
            ## Pick-up object
            self.kinova_handover_controller.pickup_object(pickup_location, self.config.object_orientation)

            ## Perform handover
            if self.task_controller == 'pid' and rospy.has_param('pid_kp_'+str(vaccine_type)):
                controller_parameter = rospy.get_param('pid_kp_'+str(vaccine_type))
            elif self.task_controller == 'mpc' and rospy.has_param('mpc_t_'+str(vaccine_type)):
                controller_parameter = rospy.get_param('mpc_t_'+str(vaccine_type))
            self.kinova_handover_controller.handover_controller.perform_handover(controller_parameter)





    def get_timing_constraints(self, vaccine_type):
        return '20, 40'

    def update_stack(self,vaccine_type):
        ## Function to fill up stack_status with one type of vaccine 
        self.stack_status[vaccine_type-1,:] = np.ones(self.vaccines_stack)

    def get_pickup_location(self, vaccine_type):
        if ~(np.count_nonzero(self.stack_status[vaccine_type-1,:])): #to ensure the row isn't empty already
            index_to_go_to = np.argmax(self.stack_status[vaccine_type-1,:])+1 #TODO:index_to_go_to needs to be converted to a distance by another function
        else:
            self.update_stack(vaccine_type)
        
        return(self.config.hover_location)


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('handovers_task_planner')
    try:
        brain = TaskPlanner(sys.argv[1], sys.argv[2])
    except rospy.ROSInterruptException:
        pass


