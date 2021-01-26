# BU21026-TS - Touch screen Linux driver

[![License: GPL v3](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](http://www.gnu.org/licenses/gpl-3.0)
[![License: LGPL v3](https://img.shields.io/badge/License-LGPL%20v3-blue.svg)](http://www.gnu.org/licenses/lgpl-3.0)

This repository contains Linux drivers for LCD touch scree driver
implemented on OPEN-EYES-II devices from OPEN-EYES S.r.l.

This driver is licensed under the Gnu Public License.

This driver is tested under the Linux 5.X kernels.

For more information about OPEN-EYES-II devices visit http://www.open-eyes.it

The bu21026-ts Linux driver dialog with the ROHM chipset:

[bu21026](https://fscdn.rohm.com/en/products/databook/datasheet/ic/sensor/touch_screen/bu21026muv-e.pdf)

on OPEN-EYES-RPI multifunctions access system based on Rasberry compute module CM3

## Manual installation

### Build instructions

Prepare system for build:
```
sudo apt update
sudo apt upgrade
sudo apt-get install raspberrypi-kernel-headers git
```
Download from git:
```
git clone https://github.com/openeyes-lab/bu21026-ts.git
```
Compile and install
```
cd bu21026-ts/build
make
make install
```

### Implement device Tree overlay

source file : dts/bu21026-ts.dts

compile dts file and copy into /boot/overlays directory
```
dtc -@ -b 0 -I dts -O dtb -o bu21026-ts.dtbo dts/bu21026-ts.dts
```
change compiled file owner and move it into /boot/overlays directory
```
sudo chown root:root bu21026-ts.dtbo
sudo mv bu21026-ts.dtbo /boot/overlays
```
add this line
```
dtoverlay=bu21026-ts
```
into the file /boot/config.txt

reboot

## Automatic install/uninstall

After cloning the file;
to install driver execute:
```
cd bu21026-ts
bash install.sh
```
to uninstall execute:
```
cd bu21026-ts
bash uninstall.sh
```

# Interface involved

The bu21026 chipset implements a SLAVE I2C interface and answers to the
address 0x48.

# Reference

Linux input driver [documentation](https://www.kernel.org/doc/Documentation/input/input-programming.txt)

Python evdev [evdev](https://python-evdev.readthedocs.io/en/latest/)
