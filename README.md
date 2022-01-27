# dt-kv260

## Essential Installation

### Device Driver

```
sudo apt-get update
sudo apt-get install git dkms
sudo git clone "https://github.com/RinCat/RTL88x2BU-Linux-Driver.git" /usr/src/rtl88x2bu-git
sudo sed -i 's/PACKAGE_VERSION="@PKGVER@"/PACKAGE_VERSION="git"/g' /usr/src/rtl88x2bu-git/dkms.conf
cat /usr/src/rtl88x2bu-git/dkms.conf
sudo dkms add -m rtl88x2bu -v git
sudo dkms autoinstall
```

### Network Tools
```
sudo apt install net-tools
sudo apt install openssh-server
sudo apt install vino
```

### VNC

```
git clone http://github.com/novnc/noVNC.git
cd noVNC/ && ./utils/launch.sh --vnc localhost:5900
./launch.sh
./utils/novnc_proxy --vnc 192.168.50.171:5901
```
TODO 

## PYNQ

See the detail in the announcement: [Link](https://discuss.pynq.io/t/pynq-now-available-for-the-kria-kv260-vision-ai-starter-kit/3579)


