Uses webcams to look for QR codes on the walls and determine the robot's position.

This node depends on the `zbar` library to detect QR codes.
See [INSTALL.md](https://github.com/corobotics/corobots/blob/master/INSTALL.md) for installation instructions.

Sample command to run this node:

rosrun corobot_qrcode detect  _device:="/dev/video0" _csvfile="../barcodePoints.csv" __name:="left"
