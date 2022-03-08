# HOW TO MAKE A Duckietown Device

## hostname

Type the following command to edit /etc/hostname using nano or vi text editor:
```
sudo nano /etc/hostname
```
Delete the old name and setup new name.
Next Edit the /etc/hosts file:
```
sudo nano /etc/hosts
```
Replace any occurrence of the existing computer name with your new one.
Reboot the system to changes take effect:
```
sudo reboot
```

please follow the naming rule in https://github.com/ARG-NCTU/dt-kv260#readme

## dockers reboot

check all running docker containers
```
docker ps -a
```
stop and kill all docker containers
```
docker stop DOCKER_CONTAINER_ID
docker rm DOCKER_CONTAINER_ID
```
open all docker using docker compose in https://github.com/ARG-NCTU/dt-kv260/blob/main/installation/bin/dt-autoboot
(docker composer is a command to run multiple docker containers)
```
source /usr/local/bin/dt-autoboot
```
## dockers migrate

1. To move docker images with USB(for some hardware we can't do docker push)
https://ithelp.ithome.com.tw/articles/10191387
2. Docker push and pull, need to do docker login first, and we can check if it success on docker nub

## wifi

make sure our bot and top are in the same network, for ARG lab we use EE622 network, you can plus it when initialing SD card using dts, for rpi3B+ using DB19, we need to install wifi driver manually, please follow below steps.

Install git dkms
```
 $ sudo apt-get update
 $ sudo apt-get install git dkms
```
Clone a repo for wifi
```
 $ sudo git clone "https://github.com/RinCat/RTL88x2BU-Linux-Driver.git" /usr/src/rtl88x2bu-git
 $ sudo sed -i 's/PACKAGE_VERSION="@PKGVER@"/PACKAGE_VERSION="git"/g' /usr/src/rtl88x2bu-git/dkms.conf
```
Install dongle drive
```
 $ sudo dkms add -m rtl88x2bu -v git
 $ sudo dkms autoinstall
```
Install network tool
```
 $ sudo apt get network-manager
 $ sudo service dhcpcd stop
 $ sudo service network-manager restart
```

## avahi

We should modify the following files:

"/etc/avahi/services/dt.presence.service",
            "/etc/avahi/services/dt.robot_type.service",
            "/etc/avahi/services/dt.booting.service.disabled",
            "/etc/avahi/services/dt.robot_configuration.service",
            "/etc/avahi/services/dt.booting.service",
