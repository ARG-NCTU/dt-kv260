#! /bin/bash

###################################### Docker images #####################################

# portainer
echo -e "\e[93m Download portainer docker image \e[0m"
docker pull portainer/portainer:linux-arm-1.24.1

# dt-gui-tools
echo -e "\e[93m Download dt-gui-tools docker image \e[0m"
docker pull duckietown/dt-gui-tools:daffy-arm64v8

# dt-base-environment
echo -e "\e[93m Download dt-base-environment docker image \e[0m"
docker pull duckietown/dt-base-environment:daffy-arm64v8

# dt-commons
echo -e "\e[93m Download dt-commons docker image \e[0m"
docker pull duckietown/dt-commons:daffy-arm64v8

# dt-device-proxy
echo -e "\e[93m Download dt-device-proxy docker image \e[0m"
docker pull duckietown/dt-device-proxy:daffy-arm64v8

# dt-ros-commons
echo -e "\e[93m Download dt-ros-commons docker image \e[0m"
docker pull duckietown/dt-ros-commons:daffy-arm64v8

# dt-car-interface
echo -e "\e[93m Download dt-car-interface docker image \e[0m"
docker pull duckietown/dt-car-interface:daffy-arm64v8

# dt-duckiebot-interface
echo -e "\e[93m Download dt-duckiebot-interface docker image \e[0m"
docker pull duckietown/dt-duckiebot-interface:daffy-arm64v8

# dt-rosbridge-websocket
echo -e "\e[93m Download dt-rosbridge-websocket docker image \e[0m"
docker pull duckietown/dt-rosbridge-websocket:daffy-arm64v8

# dt-core
echo -e "\e[93m Download dt-core docker image \e[0m"
docker pull duckietown/dt-core:daffy-arm64v8

# dt-files-api
echo -e "\e[93m Download dt-files-api docker image \e[0m"
docker pull duckietown/dt-files-api:daffy-arm64v8

# dt-code-api
echo -e "\e[93m Download dt-code-api docker image \e[0m"
docker pull duckietown/dt-code-api:daffy-arm64v8

# dt-device-online
echo -e "\e[93m Download dt-device-online docker image \e[0m"
docker pull duckietown/dt-device-online:daffy-arm64v8

# dt-device-health
echo -e "\e[93m Download dt-device-health docker image \e[0m"
docker pull duckietown/dt-device-health:daffy-arm64v8

# dt-system-monitor
echo -e "\e[93m Download dt-system-monitor docker image \e[0m"
docker pull duckietown/dt-system-monitor:daffy-arm64v8

# dt-device-dashboard 
echo -e "\e[93m Download dt-device-dashboard  docker image \e[0m"
docker pull duckietown/dt-device-dashboard:daffy-arm64v8

###################################### Setup crucial files #####################################

# copy services files to /etc/avahi/services
sudo cp installation/services/* /etc/avahi/services
echo -e "\e[93m Copy services files to /etc/avahi/services done \e[0m"

# copy bin files to /
sudo cp installation/bin/* /usr/local/bin
echo -e "\e[93m Copy bin files to /usr/local/bin done \e[0m"

# copy installation/data to /
sudo cp -r installation/data /
echo -e "\e[93m Copy installation/data to / done \e[0m"

# copy installation/triggers to /
sudo cp -r installation/triggers /
echo -e "\e[93m Copy installation/triggers to / done \e[0m"

# copy installation/secrets to /
sudo cp -r installation/secrets /
echo -e "\e[93m Copy installation/secrets to / done \e[0m"
