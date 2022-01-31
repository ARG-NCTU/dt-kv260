# dt-docker

What docker images are in the Duckiebot?

```
docker images
```
|                                     |         | Base                  |
| duckietown/dt-device-dashboard      | 500MB   | compose               |
| duckietown/dt-system-monitor        | 2.23GB  | dt-commons            |
| duckietown/dt-drone-interface       | 2.34GB  | 
| duckietown/dt-files-api             | 2.23GB  | 
| duckietown/dt-device-health         | 2.23GB  | dt-commons            | Provides a RESTful API to monitor the health of a Duckietown device |
| duckietown/dt-device-online         | 2.24GB  | dt-commons            | Connects a Duckietown device to the Duckietown community network |
| duckietown/dt-code-api              | 2.23GB  |
| duckietown/dt-core                  | 2.92GB  | dt-ros-commons        |
| duckietown/dt-duckiebot-interface   | 2.45GB  |
| duckietown/dt-rosbridge-websocket   | 2.38GB  |
| duckietown/dt-car-interface         | 2.33GB  |
| duckietown/dt-ros-commons           | 2.33GB  | dt-commons            |
| duckietown/dt-device-proxy          | 2.24GB  |
| duckietown/dt-commons               | 2.23GB  | dt-base-environment   |
| duckietown/dt-base-environment      | 2.2GB   | arm32v7/ubuntu:focal  |
| duckietown/dt-gui-tools             | 3.58GB  |
| portainer/portainer                 | 62.5MB  |

