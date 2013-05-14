# Corobots

This is the main repository for the Corobotics project, where code that actually runs on the robot is stored.

## Running

- Make sure the robot is powered on.
- Make sure the Upstart process is running:

        sudo start corobot

- Wait for a beep from the robot to indicate a connection has been made.
- Start the kinect-related processes:

        roslaunch corobot_bringup kinect.launch

- In a new terminal, start the primary applications:

        roslaunch corobot_bringup apps.launch
