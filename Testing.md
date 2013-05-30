# Current testing protocol:

Note: if logging in remotely for testing, you will need to ssh -X otherwise the qrcode nodes will not be able to spawn threads (?!)

First, turn on the robot.  Note that the power light will turn off once the code has connected to it, but it's still on.  You should turn it off again when you are done, which involves pushing the power button and watching nothing change. :)

You will probably want to run several terminals and/or tmux to run:

* `roslaunch corobot_bringup minimal.launch`
* wait for beep, Kinect power light on
* [new window] `roslaunch corobot_bringup 3d.launch`
* wait for this to finish
* [new window] `roslaunch corobot_bringup apps.launch`

The whole robot system should now be running.  Local user code can be found in ~/corobot-clients/python/ for testing.  Probably most useful would be to run `python3 nav_to.py <landmark-name>`

Note that if you bring some nodes down (not sure exactly which ones) you may need to restart the robot and thus the whole launch sequence.

Don't forget to turn the robot off before plugging it back in to charge.
