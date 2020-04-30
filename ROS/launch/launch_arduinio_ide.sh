#!/bin/sh
xterm -hold -e "arduino --board arduino:avr:uno --port /dev/ttyUSB0 --upload /home/pi/catkin_ws/src/JACS/src/ardino/drive/drive.ino" &
sleep 2
xterm -hold -e "arduino --board arduino:avr:uno --port /dev/ttyUSB0 --upload /home/pi/catkin_ws/src/JACS/src/Police_Academy/actuators/readEnc/readEnc.ino"  
