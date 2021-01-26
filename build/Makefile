bu21026_ts-objs := bu21026.o

obj-m += bu21026_ts.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

install:
	sudo cp bu21026_ts.ko /lib/modules/$(shell uname -r)/kernel/drivers/input/touchscreen
	sudo depmod -a

load:
	sudo insmod bu21026_ts.ko

reload:
	sudo rmmod bu21026_ts.ko
	sudo insmod bu21026_ts.ko
