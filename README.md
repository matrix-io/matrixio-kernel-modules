# matrixio-kernel-modules

MATRIXIO Kernel Modules is the kernel drivers for MATRIX Creator and MATRIX Voice.

## Warning: Kernel Version
This drivers **only** works with current stock raspbian kernel. To reverting back to current stock Raspbian kernel use:  

```
sudo apt-get install --reinstall raspberrypi-bootloader raspberrypi-kernel
```

## Option 1: Package Installation
```
# Add repo and key
curl https://apt.matrix.one/doc/apt-key.gpg | sudo apt-key add -
echo "deb https://apt.matrix.one/raspbian $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/matrixlabs.list

# Update packages and install
sudo apt-get update
sudo apt-get upgrade

# Reboot in case of Kernel Updates

sudo reboot

# Installation MATRIX Packages
sudo apt install matrixio-kernel-modules

# Reboot
sudo reboot
```
## Option 2: Cloning & compiling from sources

### Install Dependencies

```
# Add repo and key
curl https://apt.matrix.one/doc/apt-key.gpg | sudo apt-key add -
echo "deb https://apt.matrix.one/raspbian $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/matrixlabs.list

# Update packages and install
sudo apt-get update
sudo apt-get upgrade

# Installation MATRIX Pacakages
sudo apt install matrixio-creator-init

# Installation Kernel Packages
sudo apt-get -y install raspberrypi-kernel-headers raspberrypi-kernel git 

# Reboot
sudo reboot
```

### Cloning & Compiling
```
git clone https://github.com/matrix-io/matrixio-kernel-modules
cd matrixio-kernel-modules/src
make && make install
```
### Overlay Setup

Add in `/boot/config.txt`

```
dtoverlay=matrixio
```

Finally, load the remaining required modules
```
sudo cp ~/matrixio-kernel-modules/misc/matrixio.conf /etc/modules-load.d/
sudo cp ~/matrixio-kernel-modules/misc/asound.conf /etc/
sudo reboot
```

The reboot in the last step allows and activates the MATRIXIO Kernel modules. 
