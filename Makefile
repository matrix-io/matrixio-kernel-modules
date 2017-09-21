DTC=dtc

obj-m += matrixio-core.o
obj-m += matrixio-tty.o


all:	matrixio-core-overlay.dtbo
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

matrixio-core-overlay.dtbo: matrixio-core-overlay.dts
	$(DTC) -@ -I dts -O dtb -o matrixio-core-overlay.dtbo matrixio-core-overlay.dts

install: matrixio-core-overlay.dtbo
	sudo cp matrixio-core-overlay.dtbo /boot/overlays
	sudo cp matrixio-core.ko /lib/modules/$(shell uname -r)/kernel/drivers/mfd/
	sudo depmod -a

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm *.dtbo
