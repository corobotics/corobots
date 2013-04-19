Uses webcams to look for QR codes on the walls and determine the robot's position.

This node depends on the `zbar` library to detect QR codes.

## Installing `zbar`

    sudo apt-get install -y python-gtk2-dev v4l-utils gettext xmlto
    cd /usr/local/src
    sudo hg clone http://zbar.hg.sourceforge.net:8000/hgroot/zbar
    cd zbar
    sudo autoreconf --install
    sudo ./configure --without-imagemagick
    sudo make
    sudo make install
