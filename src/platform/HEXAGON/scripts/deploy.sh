#!/bin/bash

adb push obj/test.so /usr/lib/rfsa/adsp/betaflight.so
adb push src/platform/HEXAGON/host/betaflight /usr/bin
adb shell chmod a+x /usr/bin/betaflight
adb push src/platform/HEXAGON/host/voxl-inspect-osd /usr/bin
adb shell chmod a+x /usr/bin/voxl-inspect-osd
