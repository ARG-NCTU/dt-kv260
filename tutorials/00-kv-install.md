# kv260 Installation

Follow [iXilinx Announcement](https://discuss.pynq.io/t/pynq-now-available-for-the-kria-kv260-vision-ai-starter-kit/3579)


## Essential Installation

### Edimax wifi dongle Driver

```
$ sudo apt-get update
$ sudo apt-get install git dkms
$ sudo git clone "https://github.com/RinCat/RTL88x2BU-Linux-Driver.git" /usr/src/rtl88x2bu-git
$ sudo sed -i 's/PACKAGE_VERSION="@PKGVER@"/PACKAGE_VERSION="git"/g' /usr/src/rtl88x2bu-git/dkms.conf
$ sudo dkms add -m rtl88x2bu -v git
$ sudo dkms autoinstall
```

### Network Tools

```
$ sudo apt-get update
$ sudo apt-get install net-tools
$ sudo apt-get install openssh-server
```

### Docker
```
$ sudo apt-get update
$ sudo apt-get install ca-certificates curl gnupg lsb-release
$ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
$ echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
$ sudo apt-get update
$ sudo apt-get install docker-ce docker-ce-cli containerd.io
``` 

## PYNQ

See the detail in the announcement: [Link](https://discuss.pynq.io/t/pynq-now-available-for-the-kria-kv260-vision-ai-starter-kit/3579)

Install pynq from the [Kria-PYNQ](https://github.com/Xilinx/Kria-PYNQ) repository.

Run the Jupyter (password: xilinx)

![image](https://user-images.githubusercontent.com/16217256/151502021-d09e9d54-8a67-4f9d-99e6-7344637af665.png)


