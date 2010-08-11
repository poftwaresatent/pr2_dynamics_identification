Sketch of Test in Gazebo
========================

1. launch gazebo
   - note that the one from the pr2_gazebo package does not work for this setup,
     use the setup from pr2_stanford_wbc instead

    roscd pr2_stanford_wbc && roslaunch launch/pr2_gazebo.launch

2. launch dynamics identification controller

    roscd dynamics_identification && roslaunch di.launch

3. echo dynamics identification data on console

    rostopic echo /di_controller/data

4. send a command to dynamics identification controller
   - note that the elbow flex joint behaves "weirdly" in Gazebo, hopefully
     it works better on the robot

    rostopic pub -1 -- /di_controller/start dynamics_identification/Start l_shoulder_pan_joint -0.2 0.5 5 1

5. plot it

    rxplot /di_controller/data/position[0]:velocity[0]:command_torque[0]:applied_torque[0]
