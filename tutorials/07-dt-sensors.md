# How to make Nano-duckiebot see
https://www.jetsonhacks.com/2019/04/02/jetson-nano-raspberry-pi-camera/

### camera with cv
https://www.rs-online.com/designspark/jetson-nano-opencv-cn

# How to make KV260-duckiebot see
https://github.com/Xilinx/Kria-PYNQ/blob/main/kv260/base/notebooks/video/mipi_to_displayport.ipynb

# How to make Rpi-duckiebot see

### Go to settings and enable camera function.
```
$ sudo raspi-config
```

### Select **Interface Options**
![raspi_config](img/raspi_config.png)

### Select **Camera**
![raspi_config_camera](img/raspi_config_camera.png)

### Press **Yes** to enable camera function
![enable_camera](img/enable_camera.png)

### Reboot Rpi device
```
$ sudo reboot now
```
