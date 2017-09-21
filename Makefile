DTC=dtc

obj-m += matrixio-core.o matrix-tty.o


all:	matrixio-core.dtbo
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

matrixio-core.dtbo: matrixio-core.dts
	$(DTC) -@ -I dts -O dtb -o matrixio-core.dtbo matrixio-core.dts

install: matrixio-core.dtbo
	cp matrixio-core.dtbo /boot/overlays

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm *.dtbo
