mkdir /usr/src/matrixio-0.1
cp *.c *.h *.dts dkms.conf Makefile /usr/src/matrixio-0.1
dkms add -m matrixio -v 0.1
dkms build -m matrixio -v 0.1
dkms install -m matrixio -v 0.1

dtc  -W no-unit_address_vs_reg -@ -I dts -O dtb -o matrixio.dtbo matrixio-overlay.dts
cp matrixio.dtbo /boot/overlays/ 
# i# dkms remove --force -m matrixio -v 0.1 --all

