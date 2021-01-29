#!/bin/bash
NAME=$(awk '/#define\ MODULNAME/ {print substr($3,2,length($3)-2)}' bme280_driver.h)
if [ -z "$NAME" ];
then
  echo "No MODULENAME found in bme280_driver.h";
  exit -1;
fi

LOADED=$(lsmod | awk '/'$NAME'/ {print $1}')
if [ -z "$LOADED" ];
then  
  sudo insmod bme280_driver.ko
else
  echo "reload kernel module bme280_driver."
  sudo rmmod bme280_driver
  sudo insmod bme280_driver.ko
fi

NUMBER=$(awk '/'$NAME'/ {print $1}' /proc/devices)
if [ -z "$NAME" ];
then
  echo "Module not listed in /proc/devices!";
  exit -1;
fi

LOADED=$(ls /dev/ | grep "$NAME" )
if [ -z $LOADED ]; then  
  sudo mknod /dev/$NAME c $NUMBER 0
  sudo chmod o+wr /dev/$NAME
else  
  sudo chmod o+wr /dev/$NAME
fi
