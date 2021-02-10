#!/bin/bash

echo "Running script $0 with params $1"

if [ -z "$1" ]; then
	sudo apt update
	sudo apt upgrade -y
	echo "Checking headers installation"
	dpkg-query -s raspberrypi-kernel-headers
	if [ $? -eq 1 ]
	then
		sudo apt install -y raspberrypi-kernel-headers 
	fi
fi

# build overlay dtbo
if dtc -@ -b 0 -I dts -O dtb -o bu21026-ts.dtbo dts/bu21026-ts.dts ; then
	sudo chown root:root bu21026-ts.dtbo
	sudo mv bu21026-ts.dtbo /boot/overlays
else
	echo "fail to compile dts"
	exit -1
fi

if grep -q "dtoverlay=bu21026-ts" /boot/config.txt ; then
	echo "confi.txt already prepared"
else
	echo "dtoverlay=bu21026-ts" | sudo tee -a /boot/config.txt
fi

cd build

make
if [ $? -ne 0 ]; then
    echo "Failed to make"
    echo -1
fi

make install
if [ $? -ne 0 ]; then
    echo "Failed to install"
    echo -1
fi

rm *.o
rm *.mod
rm modules.order
rm Module.symvers
rm *.mod.c
rm *.ko
rm .*.cmd

echo "bu21026 correctly installed: reboot to make effective"
