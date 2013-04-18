Uses webcams to look for QR codes on the walls and determine the robot's position.

This node depends on the `zbar` library to detect QR codes.

## Installing `zbar`

    sudo apt-get install -y python-gtk2-dev v4l-utils
    cd ~/Downloads
    wget -O zbar-0.10.tar.bz2 http://sourceforge.net/projects/zbar/files/zbar/0.10/zbar-0.10.tar.bz2/download
    tar xvf zbar-0.10.tar.bz2
    cd zbar-0.10
    ./configure --without-imagemagick
    make
    sudo make install
    cd ..
    rm -rf zbar-0.10
