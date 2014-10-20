nao_dcm_robot
=============

DCM stack integrating ros controllers for NAO robots

to command your NAO remotely with ros control : 
In nao_dcm_bringup/config/nao_dcm.yaml, set the RobotIP parameter with your NAO's IP Adress

        roslaunch nao_dcm_bringup nao_dcm_H25_bringup_remote.launch
Then you can move your robot using moveit

        roslaunch nao_moveit_config moveit_planner.launch
Or sending trajectory to the desired controller (actionlib)

        rosrun actionlib axclient.py <name of the goal topic of the action server>
example

        rosrun actionlib axclient.py /nao_dcm/LeftArm_controller/follow_joint_trajectory/goal

To choose the controllers you want to load at launchtime you have to modify nao_control/launch/nao_control_trajectory.launch
To know the list of controllers implemented please refer to : nao_control/config/nao_trajectory_control.yaml 
You can start and stop the ros-controllers using the rqt plugin ControllerManager
