import rospkg
import numpy as np


class HandoverConfig(object):

    # gets paths of ros packages
    ros_pack = None

    # root path of the handover package
    package_root = ''

    # Control Type: 'pid' or 'dmp' or 'mpc'
    control_type = 'pid'

## Gripper controller parameters

    gripper_open = 0.55  # fraction between [0,1]

    gripper_close = 0.8  #  fraction between [0,1]

    finger_max_turn = 6800     # max thread rotation for one finger

    # object_transfer_threshold = 0.25 #in m STL controller reach phase ends

## Arm controller parameters
    offset_gripper_object = np.array([0.0,0.0,0.2])

    handover_zone_threshold = 0.5 #in m

    handover_zone_boundaries = [-0.5, 1.5, -1.5 , 1.5] # [x_min, x_max, y_min, y_max]

    # safety_zone_boundaries = [0.1, 0.5, 0.12, 1.1] # [x_min, x_max, y_min, y_max] #rectuangular non allowed region - v1

    safety_zone_boundaries = [0.156, 0.679, -0.12, 1.30] # [x_min, x_max, y_min, y_max] #rectangular non allowed region 

    safety_zone_allowed1_boundaries= [0.105, 0.51 , -0.12, 0.155] # [x_min, x_max, y_min, y_max] #rectangular allowed region - package 1

    safety_zone_allowed2_boundaries= [0.51, 0.69, 0.42, 0.82] # [x_min, x_max, y_min, y_max] #rectangular allowed region - package 2

    safety_zone_allowed3_boundaries= [0.12, 0.49, 0.99, 1.34] # [x_min, x_max, y_min, y_max] #rectangular allowed region - package 3

    retreat_kp = 1.0

    retreat_threshold = 0.05

    max_velocity = 0.5

    height_threshold = 0.2

    arm_velocity_pub_topic = '/my_gen3/in/cartesian_velocity'

    target_velocity_pub_topic = '/velocity_commands'

## Matlab interface parameters
    MATLAB_velocity_topic = '/stl_velocity_commands'

    MATLAB_finger_topic = '/stl_finger_commands'

    MATLAB_controller_request_topic = '/stl_controller_request'

    MATLAB_controller_notify_topic = '/stl_controller_notify'

## Proportional Velocity Controller parameters
    # Proportional gain
    pid_kp = 2.0

    # loop rate at which the velocity commands are published
    pid_loop_rate = 20 #Hz


    # pid_velocity_threshold = 0.05 #0.015
    pid_distance_threshold = 0.10 #0.015


    # Handover location for offline handover (in robot frame)
    fixed_handover_location = [0.6, 0, 0.4]

    # Robot home location (in robot frame)
    object_location = [0.409,-0.05,0.153]
    hover_location = [0.409,-0.05,0.153+0.1]
    object_orientation = [180,0,90]

    # Robot home location (in OptiTrack frame)
    home_location_optitrack = [-0.04, 0.0, 0.22]

## DMP Velocity Controller parameters
    # Proportional gain
    dmp_kp = 10.0

    # Differential gain
    dmp_kd = 5.0

    # Time to shift from humanlike to target driven
    dmp_mu = 3.0

    dmp_tau = 2.0

    dmp_alpha = 4.0

    dmp_sigma = 0.5

    dmp_loop_rate = 10 #Hz

    dmp_distance_threshold = 0.05 #0.015

    dmp_record_trajectory = False

## MPC controller parameters
    stl_reach_time = 5.0
## Optitrack parameters
    optitrack_tf_origin = 'optitrack_origin'

    optitrack_tf_object = 'object'

    optitrack_tf_human_hand = 'human_hand'

    optitrack_tf_gripper = 'kinova_gripper'

    velocity_control_file = 'SampleVelocityProfile.txt'

    initial_arm_pose = [0.9, 0.7, 0.3]

    initial_arm_orientation = [np.pi / 2., 0, np.pi / 2.]

    robot_base_pos = [-0.08, 0.45, 0.15]

    robot_base_orientation = [0, 0, np.pi / 2]

    end_effector_topic = '/j2s7s300_driver/out/tool_wrench'

    arm_pose_action_topic = '/j2s7s300_driver/pose_action/tool_pose'

    tool_pose_topic = '/j2s7s300_driver/out/tool_pose'

    finger_action_topic = '/j2s7s300_driver/fingers_action/finger_positions'

    arm_start_service_topic = '/j2s7s300_driver/in/start'

    emergency_service_topic = '/j2s7s300_driver/in/stop'

    arm_pose_tf_frame_name = 'j2s7s300_link_base'

    finger_position_topic = '/j2s7s300_driver/out/finger_position'

    robot_home_pos = [0.1, -0.5, 0.3]

    robot_target_pos = [0.0, -0.6, 0.5]

    robot_home_pos_mocap = [-0.134, 0.322, 1.059]

    robot_target_pos_mocap = [-0.239, 0.416, 1.256]


    handover_object_gripper_distance_max = 0.15

    online_pid_pos_offset = .2

    handover_done_hand_gripper_distance = 0.25

    object_release_when_hand_gripper_distance = 0.30

    gripper_pos_done_tolerance = 0.01

    max_finger_change_when_stable = 10

    max_finger_val_open = 100

    min_finger_val_closed = 3000

    object_gripper_side_max_dist = 0.03

    object_gripper_forward_max_dist = 0.14

    def __init__(self, **kwargs):
        super(HandoverConfig, self).__init__(**kwargs)
        pack = self.ros_pack = rospkg.RosPack()
        self.package_root = pack.get_path('handover_test')
