# Installation

See [corobot-extras](https://github.com/corobotics/corobot-extras) for detailed build instructions of the complete corobot platform.

- Install ROS Fuerte.

        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
        wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
        sudo apt-get update
        sudo apt-get install -y ros-fuerte-desktop-full ros-fuerte-turtlebot

- Put source in a workspace folder for future conversion to Groovy.

        mkdir ~/corobot_ws
        cd ~/corobot_ws
        git clone https://github.com/corobotics/corobots.git src

- Install Upstart scripts.

        cd src/corobot_bringup/system
        sudo ./install.bash
        echo "export ROS_PACKAGE_PATH=/home/corobot/corobot_ws/src:$ROS_PACKAGE_PATH" >> /etc/ros/setup.bash

- Install `zbar` for webcam QR code reading.

        sudo apt-get install -y python-gtk2-dev v4l-utils gettext git xmlto
        cd /usr/local/src
        sudo git clone https://github.com/corobotics/ZBar.git zbar
        cd zbar
        sudo autoreconf --install
        sudo ./configure --without-imagemagick
        sudo make
        sudo make install

- Compile things.

        cd ~/corobot_ws
        rosmake corobot_obstacle_avoidance
        rosmake corobot_qrcode
