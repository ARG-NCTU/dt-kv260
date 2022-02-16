# HOW TO MAKE A Duckietown Device

## hostname

```
sudo vim /etc/hostname
```
please follow the naming rule in https://github.com/ARG-NCTU/dt-kv260#readme

## wifi

make sure our bot and top are in the same network, for ARG lab we use EE622 network, you can plus it when initialing SD card using dts

## avahi

We should modify the following files:

"/etc/avahi/services/dt.presence.service",
            "/etc/avahi/services/dt.robot_type.service",
            "/etc/avahi/services/dt.booting.service.disabled",
            "/etc/avahi/services/dt.robot_configuration.service",
            "/etc/avahi/services/dt.booting.service",
