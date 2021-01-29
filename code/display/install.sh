#!/bin/bash

NAME=$(awk '/#define\ MODULNAME/ {print substr($3,2,length($3)-2)}' display_driver.h)
if [ -z "$NAME" ];
then
  echo "No MODULENAME found in display_driver.h";
  exit -1;
fi

LOADED=$(lsmod | awk '/'$NAME'/ {print $1}')
if [ -z "$LOADED" ];
then
  LOADED=$(dtoverlay -l | awk '/spidev0-0_disabler/ {print $2}')
  if [ -z "$LOADED" ];
  then
    echo "dts file to diasable spidev isnt loaded"
    exit -1
  else
    sudo insmod display_driver.ko
  fi
else
  echo "reload kernel module display_driver."
  sudo rmmod display_driver
  sudo insmod display_driver.ko
fi

NUMBER=$(awk '/'$NAME'/ {print $1}' /proc/devices)
if [ -z "$NAME" ];
then
  echo "Module not listed in /proc/devices!";
  exit -1;
fi
#echo "Module found with major number $NUMBER."

LOADED=$(ls /dev/ | grep "$NAME" )
if [ -z $LOADED ]; then  
  sudo mknod /dev/$NAME c $NUMBER 0
  sudo chmod o+wr /dev/$NAME
else  
  sudo chmod o+wr /dev/$NAME
fi

