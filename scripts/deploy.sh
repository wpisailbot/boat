#!/bin/bash
TARGET=debian@beaglebone
if [ -n "$3" ]
  then TARGET="debian@$3"
fi
set -e
ssh $TARGET "rm -f /tmp/output.zip"
ssh $TARGET "rm -f /tmp/site.zip"
scp $1 $TARGET:/tmp/output.zip
scp $2 $TARGET:/tmp/site.zip
ssh $TARGET "unzip -o /tmp/output.zip -d ~/bin/sailbot"
ssh $TARGET "unzip -o /tmp/site.zip -d ~/bin/sailbot/html"

set +e

# Restart all the code if another argument is passed
if [ -n "$4" ]
  then
  ssh $TARGET "sudo /home/debian/bin/sailbot/killall.sh"
  sleep 2
  ssh $TARGET "sudo /home/debian/bin/sailbot/startup.sh 2>/dev/null >/dev/null"
fi

# The argument to sync may not actually do anything...
ssh $TARGET "sync ~/bin/sailbot/*"
