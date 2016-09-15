pepper_dcm_robot
=============

DCM stack integrating tools to control Pepper robot

Installation
------------
- install dependencies

.. code-block:: bash

        sudo apt-get install ros-indigo-pepper-robot ros-indigo-pepper-meshes ros-indigo-pepper-control

- clone and compile `naoqi_dcm_driver <https://github.com/ros-aldebaran/naoqi_dcm_driver>`_

- then, clone the code from `pepper_dcm_bringup <http://wiki.ros.org/pepper_dcm_bringup>`_ and compile

- optionally, install `pepper_moveit_config <http://wiki.ros.org/pepper_moveit_config>`_

How to use it
-------------

To command your robot remotely with ros control : 

- first, turn off Autonomous Life on your robot. You can do it either in Choregraphe or by restarting Naoqi, like this:

.. code-block:: bash

        ssh nao@<YOUR_ROBOT_IP>

        nao stop

        naoqi-bin --disable-life
    
- wake up your robot

- export your robot IP address

.. code-block:: bash

        export NAO_IP=<your_robot_ip>

- then, start the DCM bringup

.. code-block:: bash

        roslaunch pepper_dcm_bringup pepper_bringup.launch

- you can control the robot using Moveit! (install it previously)

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
