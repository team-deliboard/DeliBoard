#!bin/bash

sudo apt update -y
sudo apt install -y build-essential qt5-default qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools qtdeclarative5-dev qtmultimedia5-dev libqt5multimedia5 libqt5multimediawidgets5 libqt5multimedia5-plugins libqt5multimedia5 gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav

qmake
make

