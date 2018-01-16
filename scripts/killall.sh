#!/bin/bash
killall -s SIGINT -r \(can-dump\|logger_main\|server_main\|simple_control_main\|adaptive_control_main\|state_estimator_main\|scamp_main\|sbus-test-run\|line_tacking_main\|rigid_wing_main\|waypoint_manager_main\)
sleep 5
killall -s SIGKILL -r \(can-dump\|logger_main\|server_main\|simple_control_main\|adaptive_control_main\|state_estimator_main\|scamp_main\|sbus-test-run\|line_tacking_main\|rigid_wing_main\|waypoint_manager_main\)
rm /dev/shm/*
