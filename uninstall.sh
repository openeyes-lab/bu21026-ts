sudo rm /boot/overlays/bu21026-ts.dtbo

    sudo sed -i -e "/bu21026/d" /boot/config.txt

    sudo rm /lib/modules/$(uname -r)/kernel/drivers/input/touchscreen/bu21026_ts.ko

