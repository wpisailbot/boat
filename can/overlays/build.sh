#!/bin/bash

dtc -O dtb -o BB-DCAN1-00A0.dtbo -b 0 -@ BB-DCAN1-00A0.dts
cp BB-DCAN1-00A0.dtbo /lib/firmware
echo BB-DCAN1 > /sys/devices/bone_capemgr.*/slots

sudo modprobe can
sudo modprobe can-dev
sudo modprobe can-raw

sudo ip link set can0 up type can bitrate 250000
sudo ifconfig can0 up
