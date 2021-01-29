#!/bin/bash
# See: https://www.raspberrypi.org/forums/viewtopic.php?p=1124560

LOADED=$(dtoverlay -l | awk '/spidev0-0_disabler/ {print $2}')
if [ -z "$LOADED" ];
then
  if [ -f "spidev0-0_disabler.dtbo" ]; then
    sudo dtoverlay -d . spidev0-0_disabler
  else
    if [ -f "spidev0-0_disabler.dts" ]; then
      dtc spidev0-0_disabler.dts -O dtb >spidev0-0_disabler.dtbo
      if [ -f "spidev0-0_disabler.dtbo" ]; then
        sudo dtoverlay -d . spidev0-0_disabler
      fi
    else
      echo "no dts file found!"
    fi
  fi
else
  echo "dts file already loaded, done.";
  exit -1;
fi

