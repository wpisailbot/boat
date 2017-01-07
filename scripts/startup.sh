#!/bin/bash

DIR="$( cd "$( dirname $0 )" && pwd )"
$DIR/logger_main -logfilename /home/debian/logfile &
$DIR/can-dump
