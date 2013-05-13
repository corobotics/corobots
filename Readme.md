# Corobots

Main repository for the corobot software, a research project at the Rochester Institute of Technology (Dept. of Computer Science)

[Project Website](http://www.cs.rit.edu/~robotlab/corobots/)

See [corobot-extras](https://github.com/corobotics/corobot-extras) for detailed build instructions of the complete corobot platform

## Software Dependencies

### Installing `zbar`

Needed for webcam qrcode reading.

    # Unsure whether you need xmlto, and it requires a ton of packages...
    sudo apt-get install -y python-gtk2-dev v4l-utils gettext git xmlto
    cd /usr/local/src
    sudo git clone https://github.com/corobotics/ZBar.git zbar
    cd zbar
    sudo autoreconf --install
    sudo ./configure --without-imagemagick
    sudo make
    sudo make install
