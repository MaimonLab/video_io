#!/bin/bash

# ubuntu-restricted extras required to play H264 codecs?
sudo apt-get install ubuntu-restricted-extras

# Codec to play `.avi` with the default movie player.
sudo apt-get install libdvdnav4  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly libdvd-pkg
