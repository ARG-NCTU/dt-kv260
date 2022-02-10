# dt-docker

What docker images are in the Duckiebot?

```
docker images
```
https://github.com/ezrafollower/dt-kv260/blob/main/tutorials/img/dts_docker_images_graph.drawio.png


| Docker Images on Duckiebot          | Size     | Base Image   | Description | Repo Link |
| ----------------------------------- | -------- | ------------ | ----------- | --------- |
| duckietown/dt-base-environment      | 2.2GB   | arm32v7/ubuntu:focal  | Base Docker image environment common to (almost) any Docker image in Duckietown. | https://github.com/duckietown/dt-base-environment |
| duckietown/dt-commons               | 2.23GB  | dt-base-environment   | Base image containing common libraries and environment setup for non-ROS  applications. | https://github.com/duckietown/dt-commons |
| duckietown/dt-device-proxy          | 2.24GB  | dt-commons            | Provides a human-friendly mapping to APIs and services running on a Duckietown device | https://github.com/duckietown/dt-device-proxy |
| duckietown/dt-device-health         | 2.23GB  | dt-commons            | Provides a RESTful API to monitor the health of a Duckietown device. | https://github.com/duckietown/dt-device-health |
| duckietown/dt-device-online         | 2.24GB  | dt-commons            | Connects a Duckietown device to the Duckietown community network. | https://github.com/duckietown/dt-device-online |
| duckietown/dt-device-dashboard      | 500MB   | compose               | Provides the on-board Dashboard for Duckietown robots | https://github.com/duckietown/dt-device-dashboard |
| duckietown/dt-system-monitor        | 2.23GB  | dt-commons            | Provides tools for generating a diagnostics report on the status of a system | https://github.com/duckietown/dt-system-monitor|
| duckietown/dt-files-api             | 2.23GB  | dt-commons            | Provides an HTTP API to the `/data` directory of a Duckietown device | https://github.com/duckietown/dt-files-api |
| duckietown/dt-code-api              | 2.23GB  | dt-commons            | Provides a RESTful API through which it is possible to receive OTA updates from Duckietown. | https://github.com/duckietown/dt-code-api |
| duckietown/dt-ros-commons           | 2.33GB  | dt-commons            | Base image containing common libraries and environment setup for ROS applications. | https://github.com/duckietown/dt-ros-commons |
| duckietown/dt-drone-interface       | 2.34GB  | dt-ros-commons        | Provides a simple interface to a Duckiedrone robot | https://github.com/duckietown/dt-drone-interface |
| duckietown/dt-duckiebot-interface   | 2.45GB  | dt-ros-commons        | Contains all the drivers needed to communicate with sensors and actuators on a Duckietown device | https://github.com/duckietown/dt-duckiebot-interface |
| duckietown/dt-rosbridge-websocket   | 2.38GB  | dt-ros-commons        | WebSocket bridge from the RosBridge suite (http://wiki.ros.org/rosbridge_suite) | https://github.com/duckietown/dt-rosbridge-websocket |
| duckietown/dt-car-interface         | 2.33GB  | dt-ros-commons        | Provides a high-level interface to the robot motion capabilities | https://github.com/duckietown/dt-car-interface |
| duckietown/dt-core                  | 2.92GB  | dt-ros-commons        | Provides high-level autonomy and fleet-coordination capabilities. | https://github.com/duckietown/dt-core |
| duckietown/dt-gui-tools             | 3.58GB  | dt-core               | Provides access to GUI-based tools (e.g., rviz, rqt_image_view) | https://github.com/duckietown/dt-gui-tools |
| portainer/portainer                 | 62.5MB  | | Portainer Community Edition is a lightweight service delivery platform for containerized applications that can be used to manage Docker, Swarm, Kubernetes and ACI environments. It is designed to be as simple to deploy as it is to use. The application allows you to manage all your orchestrator resources (containers, images, volumes, networks and more) through a ‘smart’ GUI and/or an extensive API. | https://github.com/portainer/portainer |

