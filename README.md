# tof_imager_ros
ROS 2 package for the [VL53L5CX](https://www.sparkfun.com/products/18642) and [VL53L7CX](https://www.pololu.com/product/3418/resources) ToF (8x8 Array) Imager. The same implementation works for both sensors without any additional configuration.

Note: This implementation is a bit over-engineered, as I have been experimenting with ROS 2 [managed/lifecycle](https://design.ros2.org/articles/node_lifecycle.html) nodes using Python.

## Implementation details

* ```tof_imager_publisher```: This executable uses the [vl53l5cx_python](https://github.com/Abstract-Horizon/vl53l5cx_python/tree/main) library to access sensor data (for both sensors) over I2C. Distance measurements from the sensor are converted to [Pointcloud2](https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointCloud.html) messages which are published periodically using a timer to the ```/pointcloud``` topic. The sensor works in 8x8 and 4x4 modes, with the resolution of 8x8 as default. This implementation is designed as a lifecycle component and can be run individually as well.

* ```tof_imager_launch.py```: This is the launch file that launches ```tof_imager_publisher``` as a  lifecycle node, loads its parameters, and then configures and activates it. The lifecycle node is first initialized, and then set to 'configure' from the launch file. When the 'inactive' state is reached, the registered event handler activates the node.

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
* Install the vl53l5cx library (this library works for both sensors):
  ```
  $ git clone https://github.com/Abstract-Horizon/vl53l5cx_python
  $ cd vl53l5cx_python
  $ sudo python3 setup.py install
  ```
* Clone this repository in a ROS 2 workspace. Check the ```sensor_params.yaml``` file in the config directory, and make any necessary changes.
* Build the package and run the launch file: ```ros2 launch vl53l5cx_ros tof_imager_launch.py```

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
