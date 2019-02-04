#!/bin/bash

#ulimit -c unlimited
#echo "/home/debian/cores/core.%p" > /proc/sys/kernel/core_pattern
DIR="$( cd "$( dirname $0 )" && pwd )"
$DIR/bringup-can.sh
sleep 1 # Shouldn't be necessary, but give can interface time to come up
mkdir -p ~/logs
$DIR/logger_main -logfilename /home/debian/logs/logfile-$(($( ls /home/debian/logs/logfile-* | grep -o "[0-9]*$" | sort -rn | sed -n 1p ) + 1)) &
$DIR/can-dump &
$DIR/server_main &
sleep 0.5
#$DIR/adaptive_control_main &
$DIR/simple_control_main &
$DIR/ballast_control_main &
$DIR/rigid_wing_main &
$DIR/monitor_main &
#$DIR/ballastTest_main &
$DIR/rudderTest_main &
$DIR/state_estimator_main &
$DIR/scamp_main -consts_file $DIR/zero_consts.pba &
$DIR/sbus-test-run &
$DIR/waypoint_manager_main &
sleep 5
#$DIR/line_tacking_main -initial_waypoints $DIR/waypoints.pba &
$DIR/line_plan_main -initial_waypoints $DIR/waypoints.pba -initial_obstacles $DIR/obstacles.pba &
