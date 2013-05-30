# Installation

See [corobot-extras](https://github.com/corobotics/corobot-extras) for detailed build instructions of the complete corobot platform.

- Install ROS Groovy.

		sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu quantal main" > /etc/apt/sources.list.d/ros-latest.list'
        wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
        sudo apt-get update
        sudo apt-get install -y ros-groovy-desktop-full ros-groovy-turtlebot

- Put source in a workspace folder for compatibility with Groovy.

        mkdir ~/corobot_ws
        cd ~/corobot_ws
        rosws init .
        git clone https://github.com/corobotics/corobots.git src
        rosws add src/corobot_bringup
          [and similar for each package]

- Install Upstart scripts. [currently not working, TBD]

        cd src/corobot_bringup/system
        sudo ./install.bash
        echo "export ROS_PACKAGE_PATH=/home/corobot/corobot_ws/src:$ROS_PACKAGE_PATH" >> /etc/ros/setup.bash

- Install `zbar` for webcam QR code reading.

        sudo apt-get install -y python-gtk2-dev v4l-utils gettext git xmlto
        cd /usr/local/src
        sudo git clone https://github.com/corobotics/ZBar.git zbar
        cd zbar
        sudo autoreconf --install
        sudo ./configure --without-imagemagick --without-gtk
        sudo make
        sudo make install
        echo "export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH" >> ~/.zshrc
        echo "export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libv4l/v4l1compat.so" >> ~/.zshrc
		
- Compile things.

        cd ~/corobot_ws
        rosmake corobot_obstacle_avoidance
        rosmake corobot_qrcode
