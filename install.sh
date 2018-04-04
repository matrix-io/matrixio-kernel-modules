cp *.c *.h *.dts dkms.conf Makefile /usr/src/matrixio-0.1
dkms add -m matrixio -v 0.1
dkms build -m matrixio -v 0.1
dkms remove --force -m matrixio -v 0.1 --all

