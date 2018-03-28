#!/bin/bash
# Note that this assumes that everything has been started from the startup.sh script,
# and so the name of the process should contain its full path which should include bin/sailbot
ps aux | grep "bin\/sailbot" | awk '{ print $2 }' | xargs --no-run-if-empty kill -SIGINT
sleep 5
ps aux | grep "bin\/sailbot" | awk '{ print $2 }' | xargs --no-run-if-empty kill -SIGKILL
rm /dev/shm/*
