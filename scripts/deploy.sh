#!/bin/bash
set -e
ssh beaglebone "rm -f /tmp/output.zip"
scp $1 beaglebone:/tmp/output.zip
ssh beaglebone "unzip -o /tmp/output.zip -d ~/bin/sailbot"
