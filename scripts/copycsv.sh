#!/bin/bash

#Written by Eric Bakan

OLD=../RobotConfig.csv
IP=10.2.52.2
#IP=10.2.54.4
ftp -n $IP<<END

quote USER anonymous
quote PASS
put $OLD
quit
END

