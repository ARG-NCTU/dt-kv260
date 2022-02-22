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

## wifi

make sure our bot and top are in the same network, for ARG lab we use EE622 network, you can plus it when initialing SD card using dts

## camera

https://www.rs-online.com/designspark/jetson-nano-opencv-cn

## avahi

We should modify the following files:

"/etc/avahi/services/dt.presence.service",
            "/etc/avahi/services/dt.robot_type.service",
            "/etc/avahi/services/dt.booting.service.disabled",
            "/etc/avahi/services/dt.robot_configuration.service",
            "/etc/avahi/services/dt.booting.service",
