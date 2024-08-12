#!/bin/bash 
# install_dependencies 
# video_io 

sudo apt-get update
sudo apt-get install -y python3-pip

# ubuntu-restricted extras required to play H264 codecs?
sudo apt-get install ubuntu-restricted-extras

echo ""
echo "---------------------------------------------"
echo "Finished installing dependencies for video_io"
echo "---------------------------------------------"
echo ""