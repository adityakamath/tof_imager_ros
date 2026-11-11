# tof_imager_ros
![](https://img.shields.io/badge/ROS%202%20Humble-Ubuntu%2022.04-blue) ![](https://img.shields.io/badge/ROS%202%20Jazzy-Ubuntu%2024.04-blue) ![](https://img.shields.io/badge/ROS%202%20Kilted-Ubuntu%2024.04-blue) ![CI](https://github.com/adityakamath/tof_imager_ros/actions/workflows/ci.yml/badge.svg) ![GitHub License](https://img.shields.io/github/license/adityakamath/tof_imager_ros)

ROS 2 package for the [VL53L5CX](https://www.sparkfun.com/products/18642) and [VL53L7CX](https://www.pololu.com/product/3418/resources) ToF (8x8 Array) Imager. The same implementation works for both sensors without any additional configuration.

Note: This implementation is a bit over-engineered, as I have been experimenting with ROS 2 [managed/lifecycle](https://design.ros2.org/articles/node_lifecycle.html) nodes using Python.

## Implementation details

* ```tof_imager_publisher```: This executable uses the [vl53l5cx_python](https://github.com/Abstract-Horizon/vl53l5cx_python/tree/main) library to access sensor data (for both sensors) over I2C. Distance measurements from the sensor are converted to [Pointcloud2](https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointCloud.html) messages which are published periodically using a timer to the ```/pointcloud``` topic. The sensor works in 8x8 and 4x4 modes, with the resolution of 8x8 as default. This implementation is designed as a lifecycle component and can be run individually as well.

* ```tof_imager.launch.py```: This is the launch file that launches ```tof_imager_publisher``` as a lifecycle node, loads its parameters, configures the sensor and then activates it once configured, so the point cloud is published without any further steps. The node can also be started alone and driven with ```ros2 lifecycle```.

## Parameters

* ```frame_id```: Parent frame for the Pointcloud2 message (Default: ```tof_frame```)
* ```resolution```: 4 for 4x4 OR 8 for 8x8 (Default: ```8```)
* ```mode```: 1 for Continuous mode OR 3 for Autonomous mode (Default: ```1```)
* ```ranging_freq```: Ranging frequency, limited to 15Hz for 8x8 and 60Hz for 4x4 (Default: ```15```)
* ```timer_period```: Timer period in seconds (Default: ```0.1```)

More info about the mode and ranging frequency parameters can be found in the Datasheets for [VL53L5CX](https://www.st.com/resource/en/datasheet/vl53l5cx.pdf) and [VL53L7CX](https://www.pololu.com/file/0J1992/vl53l7cx.pdf), and Guides for [VL53L5CX](https://www.st.com/resource/en/user_manual/um2884-a-guide-to-using-the-vl53l5cx-multizone-timeofflight-ranging-sensor-with-wide-field-of-view-ultra-lite-driver-uld-stmicroelectronics.pdf) and [VL53L7CX](https://www.pololu.com/file/0J1993/um3038-a-guide-to-using-the-vl53l7cx-timeofflight-multizone-ranging-sensor-with-90-fov-stmicroelectronics.pdf).

## How to use

* Connect the VL53L5CX or VL53L7CX to the I2C GPIO pins of a Raspberry Pi device:
  * 3.3V to any 3.3V pin
  * SDA to BCM 2
  * SCL to BCM 3
  * GND to any ground pin
* Enable and update I2C on the Raspberry Pi
  Note: These steps are only for Ubuntu. For Raspbian, follow the steps [here](https://github.com/pimoroni/vl53l5cx-python)
  * Enable I2C:
    ```
    sudo apt install -y i2c-tools python3-pip
    sudo pip3 install smbus2
    ```
  * Update baud rate by editing the I2C setting in ```/boot/firmware/config.txt``` with the baud rate (1MHz in this case):
    ```
    dtparam=i2c_arm=on,i2c_arm_baudrate=1000000
    ```
* Clone this repository, with its submodule, in a ROS 2 workspace (the [vl53l5cx_python](https://github.com/Abstract-Horizon/vl53l5cx_python) library is in ```vl53l5cx_python/```):
  ```
  $ git clone --recursive https://github.com/adityakamath/tof_imager_ros
  ```
  If already cloned without ```--recursive```: ```git submodule update --init```.
* Install the required libraries into the Python that ROS uses. ```vl53l5cx_python``` is not a ROS package, so colcon does not build it:
  ```
  $ pip3 install ./vl53l5cx_python
  ```
  On Ubuntu 24.04 pip refuses system-wide installs; add ```--break-system-packages```, as a virtual environment is not visible to ROS nodes.
* Check the ```sensor_params.yaml``` file in the config directory, and make any necessary changes.
* Build the package and run the launch file: ```ros2 launch tof_imager_ros tof_imager.launch.py```. The launch file configures and activates the node, so ```ros2 lifecycle get /tof_imager``` should report ```active```.
* If the node exits immediately, the log shows the cause: ```No module named 'vl53l5cx'``` means the library above is missing from the Python ROS uses, and ```Sensor not alive``` points to wiring or I2C.

## Results

This package was tested using both [VL53L5CX ToF Imager](https://www.sparkfun.com/products/18642) and [VL53L7CX](https://www.pololu.com/product/3418/resources) and a Raspberry Pi 4 (4GB) running ROS 2 Humble.

The following results were achieved for the two devices:

VL53L5CX:
* With default I2C baud rates:
  * resolution=8, ranging_freq=15Hz, Both modes: ~6.1Hz
* With I2C baud rate=1MHz:
  * resolution=8, ranging_frequency=15Hz:
    * Continuous mode, timer_period=0.1: ~15Hz
    * Autonomous mode, timer_period=0.1: ~10Hz
    * Autonomous mode, timer_period=0.01: ~15Hz
  * resolution=4, ranging_frequency=60Hz:
    * Both modes, timer_period=0.01: ~56.5Hz
    * Both modes, timer_period=0.025: ~40Hz
   
VL53L7CX:
* With I2C baud rate=1MHz:
  * resolution=8, ranging_frequency=15Hz:
    * Continuous mode: ~7Hz
    * Autonomous mode: ~6Hz
  * resolution=4, ranging_frequency=60Hz, Both modes: ~18Hz:
