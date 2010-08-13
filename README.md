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

    rostopic pub -1 -- /di_controller/start dynamics_identification/Start l_shoulder_pan_joint -0.2 0.5 5 61

5. start analysing the data published by the di_controller

    ./bin/di_analysis 30

6. plot it

    rxplot /di_analysis/analysis/measured_position[0]:estimated_position[0] /di_analysis/analysis/measured_velocity[0]:estimated_velocity[0] /di_analysis/analysis/estimated_acceleration[0] /di_analysis/analysis/inertia[0]

