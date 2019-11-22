DTC?=dtc
KDIR?=/lib/modules/$(shell uname -r)/build

all: matrixio.dtbo
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

matrixio.dtbo: matrixio.dts
	$(DTC) -W no-unit_address_vs_reg -@ -I dts -O dtb -o matrixio.dtbo matrixio.dts

install: matrixio.dtbo
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules_install
	cp matrixio.dtbo /boot/overlays

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
	rm -f matrixio.dtbo
