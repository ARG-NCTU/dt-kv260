#!/usr/bin/env bash

set -x

SD_DEVICE=/dev/mmcblk0
ROOT_PART_ID=1
CONFIG_DIR=/data/config
EVENTS_DIR=/data/stats/events
USAGE_DIR=/data/stats/usage
ROBOT_TYPE_FILE=${CONFIG_DIR}/robot_type
ROBOT_HARDWARE_FILE=${CONFIG_DIR}/robot_hardware
# Nvidia Jetson-based duckiebots need the GPIO pin #29 (aka sysfs gpio149) to be set to HIGH to keep the DTHut alive
HUT_RESET_SYSFS_PIN_NO=149


setup_etc_hosts() {
    cat >/etc/hosts <<EOL
127.0.0.1	localhost
127.0.0.1	$(cat /etc/hostname) $(cat /etc/hostname).local

# The following lines are desirable for IPv6 capable hosts
::1     ip6-localhost ip6-loopback
fe00::0 ip6-localnet
ff00::0 ip6-mcastprefix
ff02::1 ip6-allnodes
ff02::2 ip6-allrouters
EOL
}

store_event() {
    type=$1
    now=$(date +%s%N)
    # collect event
    echo "{\"type\": \"${type}\", \"stamp\": ${now}}" > "${EVENTS_DIR}/${now}.json"
}

run_on_first_boot() {
    echo "Setting up the robot for its first ever boot..."

    # setup /etc/hosts
    echo "Setting up the environment..."
    setup_etc_hosts

    # activate `booting` service
    cp /etc/avahi/services/dt.booting.service.disabled /etc/avahi/services/dt.booting.service

    # # resize root partition
    # sgdisk --move-second-header ${SD_DEVICE}
    # growpart ${SD_DEVICE} ${ROOT_PART_ID}
    # resize2fs ${SD_DEVICE}p${ROOT_PART_ID}
    # partprobe

    # sanitize files
    dt-sanitize-files

    # # sudo without a password
    # echo "duckie ALL=(ALL) NOPASSWD:ALL" | tee -a /etc/sudoers

    # # add "Duckietown Front Bumper" to the device tree
    # dtc -I dts -O dtb -@ -o /boot/duckietown_front_bumper_2_v1.dtbo /boot/duckietown_front_bumper_2_v1.dts
    # /opt/nvidia/jetson-io/config-by-hardware.py -n "Duckietown Front Bumper 2 V1"

    # setup ssh access
    echo "Setting up ssh..."
    mkdir -p /home/${USER}/.ssh
    touch /home/${USER}/.ssh/authorized_keys
    echo "ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABAQDFiaiFeU0Pc3ZjiGbjJOIx28ghcWPoem8jU1OSeJnbTkKT79vrzjIbYjkBg7uBlXb6kLnbHLWHUnPlLk2IUQTxHHvakubEJkJxePdN6XO+q1sKpEvz+1GL60iBKhRljCZ9h/CcJ78kkyNQkexHT15ZDMhOnUd8c9zxwUHxSjzPSOH5ns8bxjU3oSjmzDEojPnQJmY6Evhf5DVcKXenxkzs4XgDEo+ldKo37i30iUoFCL30OsXCP2tPcn1j39qjL7vnaUBO9WqY8eOssOHAX7/K1dNN1lDvNCKspq/2f05Ss8LopSpe6hOiMnPB0RlotJbZn+784kV1B4nJpqLj+EQr DT2018key" | tee -a /home/${USER}/.ssh/authorized_keys
    chmod 755 /home/${USER}
    chmod 700 /home/${USER}/.ssh
    chmod 600 /home/${USER}/.ssh/authorized_keys
    chown -R ${USER}:${USER} /home/${USER}

    # add user `${USER}` to the `root` group
    adduser ${USER} root

    # setup docker
    echo "Adding the user to the docker group..."
    adduser ${USER} docker

    # store the MAC addresses for future reference
    echo "Storing debug information..."
    cp /proc/*info /data/proc
    cat /sys/class/net/eth0/address > /data/stats/MAC/eth0
    cat /sys/class/net/wlan0/address > /data/stats/MAC/wlan0
    cp /etc/nv_tegra_release /data/stats/os-release

    # make sure the user owns its folders
    echo "Changing the ownership of the user directories..."
    chown -R 1000:1000 /data /code
    chmod -R 775 /data /code

    # # create swap
    # echo "Setting up swap..."
    # dd if=/dev/zero of=/swap0 bs=1M count=2048
    # chmod 0600 /swap0
    # mkswap /swap0
    # echo "/swap0 swap swap" >>/etc/fstab
    # swapon -a

    # # create tmpfs of size = 75% of the total memory
    # PHYMEM_SIZE=$(free --mega | awk '/^Mem:/{print $2}')
    # TMPFS_SIZE=$(python3 -c "print(int(0.75 * ${PHYMEM_SIZE}))")
    # echo "tmpfs /data/ramdisk tmpfs  defaults,noatime,nosuid,nodev,noexec,mode=0777,size=${TMPFS_SIZE}M 0 0" >> /etc/fstab

    # reload systemctl and restart docker
    systemctl daemon-reload
    systemctl enable docker
    systemctl restart docker

    while ! timeout --signal=INT 5s docker ps; do
        echo 'Waiting for Docker to start...'
        sleep 2
    done

    # # disable ZRAM
    # systemctl disable nvzramconfig

    # run customizable first-boot script
    dt-init-first-boot

    echo "Setting up the containers..."
    dt-autoboot

    # mark event
    store_event "first_boot"

    # collect usage stats
    mkdir -p "${USAGE_DIR}/disk_image"
    mkdir -p "${USAGE_DIR}/init_sd_card"
    cp /data/stats/disk_image/build.json "${USAGE_DIR}/disk_image/$(date +%s%N).json"
    cp /data/stats/init_sd_card/build.json "${USAGE_DIR}/init_sd_card/$(date +%s%N).json"

    echo "Setting up completed!"
}

run_on_every_boot() {
    echo "Setting up the robot for this boot..."

    # activate `booting` service
    cp /etc/avahi/services/dt.booting.service.disabled /etc/avahi/services/dt.booting.service

    # setup /etc/hosts
    setup_etc_hosts

    # run customizable any-boot script
    dt-init-any-boot

    # remove `booting` service
    rm -f /etc/avahi/services/dt.booting.service

    # get robot_type
    if [ ${#ROBOT_TYPE} -le 0 ]; then
        if [ -f "${ROBOT_TYPE_FILE}" ]; then
            ROBOT_TYPE=$(cat "${ROBOT_TYPE_FILE}")
        else
            ROBOT_TYPE="duckiebot"
        fi
    fi

    # get robot_hardware
    if [ ${#ROBOT_HARDWARE} -le 0 ]; then
        if [ -f "${ROBOT_HARDWARE_FILE}" ]; then
            ROBOT_HARDWARE=$(cat "${ROBOT_HARDWARE_FILE}")
        else
            ROBOT_HARDWARE="__NOTSET__"
        fi
    fi

    # # set HUT pin to HIGH (NVidia-Jetson-based robots only)
    # if [ $ROBOT_TYPE == "duckiebot" ] && [ $ROBOT_HARDWARE == "jetson_nano" ]; then
    #     pin="gpio${HUT_RESET_SYSFS_PIN_NO}"
    #     echo ${HUT_RESET_SYSFS_PIN_NO} >/sys/class/gpio/export
    #     echo out >/sys/class/gpio/${pin}/direction
    #     echo 1 >/sys/class/gpio/${pin}/value
    #     echo ${HUT_RESET_SYSFS_PIN_NO} >/sys/class/gpio/unexport
    # fi

    # launch triggers handlers
    # dt-serve-triggers

    # mark event
    store_event "boot"

    echo "Setting up completed!"
}

FIRST_BOOT_EVIDENCE_FILE="/boot/dt.first.boot.evidence"
if test -f "$FIRST_BOOT_EVIDENCE_FILE"; then
    echo "$FIRST_BOOT_EVIDENCE_FILE exists so we assume this is not the first boot!"
    run_on_every_boot 2>&1 | tee /data/logs/this_boot_init.log
else
    echo "$FIRST_BOOT_EVIDENCE_FILE does not exist so we assume this is the first boot!"
    run_on_first_boot 2>&1 | tee /data/logs/first_boot_init.log
    touch "$FIRST_BOOT_EVIDENCE_FILE"
    reboot
fi
