# Duckietown Shell

Duckietown Shell is a pure Python, easily distributable (few dependencies) utility for Duckietown.

The idea is that most of the functionality is implemented as Docker containers, and dt-shell provides a nice interface for that, so that user should not type a very long docker run command line.

Duckietown Shell Commands
* [ARG-NCTU Fork](https://github.com/ARG-NCTU/duckietown-shell-commands) <-- The one we should modify
* [Duckietown](https://github.com/duckietown/duckietown-shell-commands)

Duckietown Shell
* https://github.com/duckietown/duckietown-shell

## Installation on Laptop

https://docs.duckietown.org/daffy/opmanual_duckiebot/out/laptop_setup.html

## Fleet

Discovers Duckietown robots in the local network. [code](https://github.com/ARG-NCTU/duckietown-shell-commands/blob/daffy/fleet/discover/command.py)
```
dts fleet discover
```

### Avahi
* Avahi is a system which facilitates service discovery on a local network via the mDNS/DNS-SD protocol suite. This enables you to plug your laptop or computer into a network and instantly be able to view other people who you can chat with, find printers to print to or find files being shared. Compatible technology is found in Apple MacOS X (branded "Bonjour" and sometimes "Zeroconf").
* See [code](https://github.com/ARG-NCTU/duckietown-shell-commands/blob/daffy/fleet/discover/command.py)
* Check avahi service. You should see "active" by default in all Ubuntu machines.
```
systemctl is-active avahi-daemon.service
```
* List all avahi service in local network
```
avahi-browse -at
```
* List all avahi service in local network for Duckietown
```
avahi-browse -at | grep DT::
```


TODO: 
```
dts fleet xbee
```

## Commands via RESTful API

Intro to RESTful API [video](https://youtu.be/7YcW25PHnAA)

Intro to [Flask](https://devs.tw/post/448)

### Privileged Commands 

* Reboot [code](https://github.com/ARG-NCTU/duckietown-shell-commands/blob/daffy/duckiebot/reboot/command.py)
* Shutdown [code](https://github.com/ARG-NCTU/duckietown-shell-commands/blob/daffy/duckiebot/shutdown/command.py)
```
dts duckiebot reboot XXX
dts duckiebot shutdown XXX
```
This will request a reboot to the service running on the container [duckietown/dt-device-health](https://github.com/duckietown/dt-device-health).
* http://{hostname}/health/trigger/reboot?value=dts&token=
* http://{hostname}/health/trigger/shutdown?value=dts&token=

### A Deeper Look at dt-device-health

* [Repo](https://github.com/duckietown/dt-device-health)
* Healthy Status, see [API](https://github.com/duckietown/dt-device-health/blob/daffy/packages/health_api/api.py)


