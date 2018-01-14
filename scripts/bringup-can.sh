#!/bin/bash

#echo BB-DCAN1 > /sys/devices/platform/bone_capemgr/slots
echo BB-DCAN1 > /sys/devices/bone_capemgr.9/slots

sudo modprobe can
sudo modprobe can-dev
sudo modprobe can-raw

# For some reason adding the sleep made this work on
# one BBB but isn't necessary on another. Something to do with
# the modules taking too long to load on boot?
# This only had an effec twhen run @reboot in crontab
sleep 5

sudo ip link set can0 up type can bitrate 250000
sudo ifconfig can0 up
