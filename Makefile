DTC=dtc

obj-m += matrixio-core.o
obj-m += matrixio-tty.o
obj-m += matrixio-everloop.o
obj-m += matrixio-codec.o

all:	matrixio.dtbo
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

matrixio.dtbo: matrixio-overlay.dts
	$(DTC) -@ -I dts -O dtb -o matrixio.dtbo matrixio-overlay.dts

install: matrixio.dtbo
	sudo cp matrixio.dtbo /boot/overlays
	sudo cp *.ko /lib/modules/$(shell uname -r)/kernel/drivers/mfd/
	sudo depmod -a

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm *.dtbo
