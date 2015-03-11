# Installation

See [corobot-extras](https://github.com/corobotics/corobot-extras) for detailed build instructions of the complete corobot platform.

- Install ROS Groovy.

		sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu quantal main" > /etc/apt/sources.list.d/ros-latest.list'
        wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
        sudo apt-get update
        sudo apt-get install openni-dev libusb-1.0-0-dev
        sudo apt-get install -y ros-groovy-desktop-full ros-groovy-turtlebot
        sudo apt-get install ros-groovy-openni-launch
		sudo rosdep init
		rosdep update
		sudo apt-get install python-rosinstall 
			[need this for rosws init .]
		
- Environment setup

		echo "source /opt/ros/groovy/setup.bash" >> ~/.bashrc
		source ~/.bashrc

- Put source in a workspace folder for compatibility with Groovy.

        mkdir ~/corobot_ws
        cd ~/corobot_ws
        rosws init .
        git clone https://github.com/corobotics/corobots.git src
        rosws set src/corobot_bringup
          [and similar for each package]
	echo "export ROS_PACKAGE_PATH=/home/corobot/corobot_ws/src:$ROS_PACKAGE_PATH" >> ~/.bashrc

- Install Upstart scripts. [currently not working, TBD]

        cd src/corobot_bringup/system
        sudo ./install.bash
        
- Install `zbar` for webcam QR code reading.

        sudo apt-get install -y python-gtk2-dev v4l-utils gettext git xmlto acpi
        cd /usr/local/src
        sudo git clone https://github.com/corobotics/ZBar.git zbar
        cd zbar
        sudo autoreconf --install
        sudo ./configure --without-imagemagick --without-gtk
        sudo make
        sudo make install
        
        echo "export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
        echo "export LD_PRELOAD=/usr/lib/i386-linux-gnu/libv4l/v4l1compat.so" >> ~/.bashrc
        [The path of v4l1compat.so may change depending on platofrom, find it using "locate v4l1compat.so" ]

		
- Compile things.

        cd ~/corobot_ws
        rosmake corobot_obstacle_avoidance
        rosmake corobot_qrcode
        
        [ TODO: figure out why ROS_ROOT is blank. temp fix below that fixes compilation error]
        export ROS_PACKAGE_PATH=/opt/ros/groovy/share:$ROS_PACKAGE_PATH
        export ROS_PACKAGE_PATH=/opt/ros/groovy/stacks:$ROS_PACKAGE_PATH
        export ROS_ROOT=/opt/ros/groovy/share/ros/
