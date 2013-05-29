# Current testing protocol:

Note: if logging in remotely for testing, you will need to ssh -X otherwise the qrcode nodes will not be able to spawn threads (?!)

You will probably want to run several terminals and/or tmux to run:

* `roslaunch corobot_bringup minimal.launch`
* wait for beep, Kinect power light on
* [new window] `roslaunch corobot_bringup 3d.launch`
* wait for this to finish
* [new window] `roslaunch corobot_bringup apps.launch`

The whole robot system should now be running.  Local user code can be found in ~/corobot-clients/python/ for testing.

Note that if you bring some nodes down (not sure exactly which ones) you may need to restart the robot and thus the whole launch sequence.