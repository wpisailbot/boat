#!/bin/bash
TARGET=beaglebone
if [ -n "$3" ]
  then TARGET="$3"
fi
set -e
ssh $TARGET "rm -f /tmp/output.zip"
ssh $TARGET "rm -f /tmp/site.zip"
scp $1 $TARGET:/tmp/output.zip
scp $2 $TARGET:/tmp/site.zip
ssh $TARGET "unzip -o /tmp/output.zip -d ~/bin/sailbot"
ssh $TARGET "unzip -o /tmp/site.zip -d ~/bin/sailbot/html"
ssh $TARGET "sync"
