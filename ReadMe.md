## Steps to run the system
Connect all PCs to HRC2 WiFi

OptiTrack PC:
1. Start OptiTrack cameras
2. Create rigid bodies from markers in Motive

Kinova PC:
path is in ``/CatkinWorkspaces/HandoversKinovaGen3/``.

Terminal-1 (Kinova Driver)
```
export ROS_MASTER_URI=http://192.168.0.103:11311
export ROS_IP=192.168.0.103
source devel/setup.bash
roslaunch kortex_driver kortex_driver.launch gripper:=robotiq_2f_85
```
Terminal-2 (ROS-Motive bridge)
```
export ROS_IP=192.168.0.103
source devel/setup.bash
roslaunch mocap_optitrack mocap.launch

```

Terminal-3 (Data logger)
```
export ROS_IP=192.168.0.103
source devel/setup.bash
rosrun handover_test data_logger.py participant_number
```

Terminal-4 (User Interface)
```
export ROS_IP=192.168.0.103
source devel/setup.bash
rqt --standalone stl_experiment_ui --args design mpc
```

Terminal-5 (SoundPlay)
```
export ROS_IP=192.168.0.103
source devel/setup.bash
roslaunch handover_test human_safety_check.launch
```

Terminal-6 (Main Controller)
```
export ROS_IP=192.168.0.103
source devel/setup.bash
rosrun handover_test task_planner.py design mpc
```

MATLAB PC:

## Connect OptiTrack and ROS
Follow instructions from : https://github.com/tuw-cpsg/tuw-cpsg.github.io/tree/master/tutorials/optitrack-and-ros
