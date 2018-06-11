Run these to execute python script:

In a new terminal, launch moveit instance
roslaunch manipulator_moveit_config demo.launch

In another terminal, change your directory
cd ~/catkin_ws/src/ROS_quadrotor_simulator/manipulator_planning/scripts

Under that second terminal, make sure pyton file is executable
chmod +x manipulator_move_group_python_interface.py

After that, run the python script
rosrun manipulator_planning balsa_arm_move_group_python_interface.py





OR

Just use a launch file with the following command:

roslaunch manipulator_planning manipulator_move_group_python_interface.launch
