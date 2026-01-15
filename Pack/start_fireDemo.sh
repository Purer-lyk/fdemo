#!/bin/bash

CAM="/dev/v4l/by-id/usb-SYD_USB_Camera_200901010001-video-index0"

/usr/bin/v4l2-ctl -d "$CAM" \
  --set-ctrl=auto_exposure=1 \
  --set-ctrl=exposure_time_absolute=35 \
  --set-ctrl=backlight_compensation=0

exec sudo /home/l/Pack/fireDemo
