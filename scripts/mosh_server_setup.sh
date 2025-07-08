#!/bin/bash

# Author: Raegan Scheet <cscheet2@unl.edu>
# Date:   2025/19/04
#
# Script to setup mosh server on rover
#
# This script must be ran with sudo premissions
# to install required packages and restart the 
# server.
#
# To ssh into this server run wither
#  - mosh lunabotics@lunabotics-desktop
#  - ssh lunabotics@lunabotics-desktop
#  - mosh lunabotics@10.65.230.173
#  - ssh lunabotics@10.65.230.173
# 

sudo apt update && sudo apt upgrade -y
sudo apt install openssh-server ufw mosh -y
sudo systemctl enable ssh
sudo ufw allow ssh
sudo ufw allow 60000:61000/udp
sudo systemctl start ssh