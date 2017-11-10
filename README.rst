pepper_dcm_robot
=============

DCM stack integrating tools to control Pepper robot

Installation
------------
- install dependencies

.. code-block:: bash

        sudo apt-get install ros-indigo-pepper-robot ros-indigo-pepper-meshes ros-indigo-pepper-control ros-indigo-naoqi-dcm-driver

- then, install pepper_dcm_bringup or compile it from source

.. code-block:: bash

        sudo apt-get install ros-indigo-pepper-dcm-bringup

- optionally, install pepper_moveit_config

.. code-block:: bash

        sudo apt-get install ros-indigo-pepper-moveit-config

How to use it
-------------

To command your robot remotely with Ros control:

- be aware that the package will stop Autonomous Life on your robot.
    
- start the DCM bringup proving your robot's IP

.. code-block:: bash

        roslaunch pepper_dcm_bringup pepper_bringup.launch robot_ip:=<ROBOT_IP>

- start Naoqi Driver (to get the odom frame) proving your robot's IP

.. code-block:: bash

        roslaunch roslaunch naoqi_driver naoqi_driver.launch nao_ip:=<ROBOT_IP>

- then, you could either start Moveit! to control the robots' arms via GUI

.. code-block:: bash

        roslaunch pepper_moveit_config moveit_planner.launch

- or you can send a trajectory to the desired controller (actionlib)

.. code-block:: bash

        rosrun actionlib axclient.py <name of the goal topic of the action server>

        example:

.. code-block:: bash

        rosrun actionlib axclient.py /pepper_dcm/LeftArm_controller/follow_joint_trajectory/goal

To choose the controllers you want to load, modify pepper_control/launch/pepper_control_trajectory.launch.
The list of implemented controllers, you can find in pepper_control/config/pepper_trajectory_control.yaml. 
You can start and stop the ros-controllers using the rqt plugin ControllerManager.
