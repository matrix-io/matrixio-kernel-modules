DTC=dtc

obj-m += matrixio-core.o
obj-m += matrixio-tty.o
obj-m += matrixio-everloop.o

all:	matrixio-core.dtbo
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

matrixio-core.dtbo: matrixio-core-overlay.dts
	$(DTC) -@ -I dts -O dtb -o matrixio-core.dtbo matrixio-core-overlay.dts

install: matrixio-core.dtbo
	sudo cp matrixio-core.dtbo /boot/overlays
	sudo cp matrixio-core.ko /lib/modules/$(shell uname -r)/kernel/drivers/mfd/
	sudo depmod -a

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm *.dtbo
