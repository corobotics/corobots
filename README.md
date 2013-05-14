# Corobots

This is the main repository for the [Corobotics](http://www.cs.rit.edu/~robotlab/corobots/) research project at the Rochester Institute of Technology CS Department.
Code that actually runs on the robot is stored here.

## Running

- Make sure the robot is powered on.
- Make sure the Upstart process is running:

        sudo start corobot

- Wait for a beep from the robot to indicate a connection has been made.
- Start the kinect-related processes:

        roslaunch corobot_bringup kinect.launch

- In a new terminal, start the primary applications:

        roslaunch corobot_bringup apps.launch
