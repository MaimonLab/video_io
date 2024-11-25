#!/bin/bash 
# install_dependencies 
# video_io 

sudo apt-get update
sudo apt-get install -y python3-pip

# ubuntu-restricted extras required to play H264 codecs?
# sudo apt-get install ubuntu-restricted-extras
sudo apt install ffmpeg

cd ~/maimon_ws/src
git clone git@github.com:MaimonLab/maimon_classes.git
pip3 install opencv-python

echo ""
echo "---------------------------------------------"
echo "Finished installing dependencies for video_io"
echo "---------------------------------------------"
echo ""