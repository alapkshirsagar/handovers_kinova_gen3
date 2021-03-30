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
        if self.task_mode not in ["design", "test"]:
            print("Incorrect mode type. Change the mode to either design or test")
            raise rospy.ROSInterruptException("Incorrect task mode")
        if self.task_controller not in ["mpc", "pid"]:
            print("Incorrect controller type. Change the controller to either pid or mpc or dmp")
            raise rospy.ROSInterruptException("Incorrect controller type")


        self.n_vaccines = 3 ## Number of vaccine types
        self.vaccines_stack = 6 ## Number of vaccines in the stack per type
        self.stack_status = np.zeros((self.n_vaccines,self.vaccines_stack)) ## Keep track of vaccines in the stack. 0 for no vaccine.
        for vaccine in range(self.n_vaccines):
            self.refill_stack_row(vaccine + 1)
        
        # self.stack_status[0,:] = np.array([0,0,0,0,0,1])

        self.kinova_handover_controller = KinovaHandoverController(controller)
        self.vaccine_stack_status_publisher = rospy.Publisher('/vaccine_stack_status', String, queue_size=10)
        self.data_logger_publisher = rospy.Publisher('/data_logger_signal', String, queue_size = 10, latch = True)

        if self.task_mode == "design":
            self.design_handovers()
        else:
            self.test_handovers()



    def design_handovers(self):
        self.data_logger_publisher.publish('design_start'+','+self.task_controller)
        ## Function to perform handovers in the design phase. Requires command line argument of the vaccine type
        vaccine_type = 0
        while True:
            vaccine_type = int(raw_input("Please write the vaccine type: "))
            
            if vaccine_type == -1:
                break
            if vaccine_type == 10:
                self.data_logger_publisher.publish('design_end'+','+self.task_controller)
                continue
            if vaccine_type == 11:
                self.test_handovers()
                break

            if vaccine_type == 100:
                break

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

            self.stack_status= self.update_stack(vaccine_type)

            vaccine_type = 0

    def test_handovers(self):
        self.data_logger_publisher.publish('test_start'+','+self.task_controller)
        for i in range(1,4):
            self.refill_stack_row(i)
        ## Function to perform handovers in the test phase. Requires no input
        vaccine_test_seq=np.array((np.repeat(1,self.vaccines_stack),np.repeat(2,self.vaccines_stack),np.repeat(3,self.vaccines_stack))).flatten()
        print(vaccine_test_seq)

        for vaccine_type in vaccine_test_seq:
            vaccine_stack_status = ""
            for i in range(0,3):
                vaccine_stack_status+=str(np.sum(self.stack_status[i,:]))+','
            self.vaccine_stack_status_publisher.publish(vaccine_stack_status)

            pickup_location = self.get_pickup_location(vaccine_type)
            timing_constraints = self.get_timing_constraints(vaccine_type)
            rospy.set_param('timing_constraints', timing_constraints)

            ## Pick-up object
            self.kinova_handover_controller.pickup_object(pickup_location, self.config.object_orientation)
            print(self.task_controller)
            ## Perform handover
            if self.task_controller == 'pid' and rospy.has_param('pid_kp_'+str(vaccine_type)):
                controller_parameter = rospy.get_param('pid_kp_'+str(vaccine_type))
            elif self.task_controller == 'mpc' and rospy.has_param('mpc_t_'+str(vaccine_type)):
                controller_parameter = rospy.get_param('mpc_t_'+str(vaccine_type))
            self.kinova_handover_controller.handover_controller.perform_handover(controller_parameter)

            self.stack_status=self.update_stack(vaccine_type)
        self.data_logger_publisher.publish('test_end'+','+self.task_controller)



    def update_stack(self, vaccine_type):
        index_to_change= self.get_vaccine_index_in_stack(vaccine_type)
        self.stack_status[vaccine_type-1,index_to_change-1] = 0
        return self.stack_status

    def get_timing_constraints(self, vaccine_type):
        if vaccine_type == 1:
            return '10, 40'
        elif vaccine_type == 2:
            return '20, 50'
        elif vaccine_type == 3:
            return '30,60'


    def refill_stack_row(self,vaccine_type):
        ## Function to fill up stack_status with one type of vaccine
        # print("refilling..............")          
        self.stack_status[vaccine_type-1,:] = np.ones(self.vaccines_stack)

    def get_pickup_location(self, vaccine_type):
        index_to_go_to=self.get_vaccine_index_in_stack(vaccine_type) #TODO:index_to_go_to needs to be converted to a distance by another function
        first_vac_location=np.array(self.config.object_location)
        pickup_location_vac=first_vac_location+[-(index_to_go_to-1)*0.04, -(vaccine_type-1)*0.04, 0]
        # print(pickup_location_vac)
        # print(self.stack_status)
        return(pickup_location_vac)

    def get_vaccine_index_in_stack(self, vaccine_type):
        if (np.count_nonzero(self.stack_status[vaccine_type-1,:])): #to ensure the row isn't empty already
            vaccine_index = np.argmax(self.stack_status[vaccine_type-1,:])+1 
            # import pdb; pdb.set_trace()
            # print('^^^^getting vaccine index ')
            # print (vaccine_index)
            # print((np.count_nonzero(self.stack_status[vaccine_type-1,:])))
            return vaccine_index
        else:
            # print('!!!!!!!!!!!!!!!!!!!!!!!!!')
            # print('refilling stack row %d' %(vaccine_type))
            self.refill_stack_row(vaccine_type)
            vaccine_index = np.argmax(self.stack_status[vaccine_type-1,:])+1 
            return vaccine_index


        
if __name__ == '__main__':
    # Initialize node
    rospy.init_node('handovers_task_planner')
    try:
        brain = TaskPlanner(sys.argv[1], sys.argv[2])
    except rospy.ROSInterruptException:
        pass


