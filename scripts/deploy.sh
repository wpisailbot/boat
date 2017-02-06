#!/bin/bash
set -e
ssh beaglebone "rm -f /tmp/output.zip"
ssh beaglebone "rm -f /tmp/site.zip"
scp $1 beaglebone:/tmp/output.zip
scp $2 beaglebone:/tmp/site.zip
ssh beaglebone "unzip -o /tmp/output.zip -d ~/bin/sailbot"
ssh beaglebone "unzip -o /tmp/site.zip -d ~/bin/sailbot/html"
