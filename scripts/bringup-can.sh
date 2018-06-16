#!/bin/bash

# Only use the capemgr if it exists
# Ideally, everything in this file should be setup in
# the /boot/uEnv.txt, /etc/modules-load.d,
# and /etc/network/interfaces respectively,
# as described in the DOCS.md
if [ -a /sys/devices/platform/bone_capemgr/slots ]
then
echo BB-DCAN1 > /sys/devices/platform/bone_capemgr/slots
fi

sudo modprobe can
sudo modprobe can-dev
sudo modprobe can-raw

sudo ip link set can0 up type can bitrate 250000
sudo ifconfig can0 up
