Sketch of Test in Gazebo
========================

1. launch gazebo

    roscd pr2_gazebo && roslaunch pr2_empty_world.launch

2. launch dynamics identification controller

    roscd dynamics_identification && roslaunch di.launch

3. echo dynamics identification data on console

    rostopic echo /di_controller/data

4. send a command to dynamics identification controller

    rostopic pub -1 /di_controller/start dynamics_identification/Start l_elbow_flex_joint 0.01 0.1 0.1 1
