#!/bin/bash

#echo BB-DCAN1 > /sys/devices/platform/bone_capemgr/slots
echo BB-DCAN1 > /sys/devices/bone_capemgr.9/slots

sudo modprobe can
sudo modprobe can-dev
sudo modprobe can-raw

sudo ip link set can0 up type can bitrate 250000
sudo ifconfig can0 up
