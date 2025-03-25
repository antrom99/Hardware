#!/bin/bash

export ROS_MASTER_URI=http://PCAUT-06:11311
ETH_INTERFACE=enp2s0
IP_SEARCH="(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)"
export ROS_IP=$(ip address show $ETH_INTERFACE | grep -E -o "inet $IP_SEARCH" | awk '{print $2;}')