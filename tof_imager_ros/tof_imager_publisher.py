# Copyright (c) 2023 Aditya Kamath
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import time
import rclpy
import numpy as np
from typing import Optional
from rclpy.lifecycle import Node, Publisher, State, TransitionCallbackReturn
from rclpy.timer import Timer
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from vl53l5cx.vl53l5cx import VL53L5CX, VL53L5CXResultsData as ToFImagerResults


# Bytes per I2C write: the library's 4 KiB writes hold a 400 kHz bus for about 90 ms during the firmware upload
WRITE_CHUNK_BYTES = 256
WRITE_PAUSE_S = 0.001


class ToFImager(VL53L5CX):
    """VL53L5CX that sends large writes in small chunks so other devices can use the shared I2C bus."""

    def wr_multi(self, addr, buffer, size):
        """Write size bytes starting at register addr, one short transaction at a time."""
        for position in range(0, size, WRITE_CHUNK_BYTES):
            data = buffer[position:position + min(WRITE_CHUNK_BYTES, size - position)]
            register = addr + position
            self._i2c_bus.i2c_rdwr(self.i2c_msg.write(
                self.i2c_address, [register >> 8 & 0xff, register & 0xff, *data]))
            time.sleep(WRITE_PAUSE_S)

class ToFImagerPublisher(Node):
    def __init__(self, node_name='tof_imager'):
        super().__init__(node_name)
        self.pcl_pub: Optional[Publisher] = None
        self.timer: Optional[Timer] = None   
        
        # declare parameters and default values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('frame_id', 'tof_frame'),
                ('resolution', 8),
                ('mode', 1), # 1 is continuous, 3 is autonomous
                ('ranging_freq', 15),
                ('timer_period', 0.1),
            ]
        )

        self.res = self.get_parameter('resolution').value 
        self.mode = self.get_parameter('mode').value
        self.freq = self.get_parameter('ranging_freq').value
        self.frame_id = self.get_parameter('frame_id').value
        self.sensor = None
        self._ranging = False
        self._rays = None
        self._fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)]

        self.get_logger().info('Initialized')

    def _build_rays(self):
        """Per-zone x/y factors, computed once. Zone centres across the 45 degree FOV; pixel rows run
        toward the sensor's left, so x is negated (right-handed frame, z along the boresight)."""
        per_px = np.deg2rad(45) / self.res
        angles = (np.arange(self.res) + 0.5)*per_px - np.deg2rad(45)/2
        sin = np.sin(angles).astype(np.float32)/1000
        self._rays = (-sin[:, None], sin[None, :])

    def read_sensor(self):
        """Read data from the ToF sensor and convert to point cloud data"""
        try:
            if not self.sensor.check_data_ready():
                return None
        except Exception as e:
            self.get_logger().error(f"Sensor error (code: {e}). Attempting to restart...")
            try:
                # Try to recover the sensor
                self.sensor.stop_ranging()
                self.sensor.start_ranging()
            except Exception as restart_error:
                self.get_logger().error(f"Failed to restart sensor: {restart_error}")
            return None

        try:
            data = self.sensor.get_ranging_data()
        except IndexError:
            data = ToFImagerResults(nb_target_per_zone=1)
        except Exception as e:
            self.get_logger().error(f"Error getting ranging data: {e}")
            return None

        distance_mm = np.asarray(data.distance_mm[:self.res*self.res], dtype=np.float32).reshape(self.res, self.res)
        np.maximum(distance_mm, 0, out=distance_mm)
        sx, sy = self._rays
        buf = np.empty((self.res, self.res, 3), dtype=np.float32)
        buf[..., 0] = distance_mm*sx
        buf[..., 1] = distance_mm*sy
        buf[..., 2] = distance_mm/1000
        return buf

    def publish_pcl(self):
        """Populate and publish the ToF imager pointcloud"""
        # Nobody listening: leave the shared I2C bus alone (the sensor keeps ranging on its own)
        if self.pcl_pub is None or not self.pcl_pub.is_activated or self.pcl_pub.get_subscription_count() == 0:
            return
        buf = self.read_sensor()
        if buf is None:
            return
        self.pcl_pub.publish(PointCloud2(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.frame_id),
            height=self.res,
            width=self.res,
            fields=self._fields,
            is_bigendian=False,
            is_dense=False,
            point_step=12,
            row_step=12*self.res,
            data=buf.tobytes()))

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure the ToF imager sensor"""
        if self.mode not in (1, 3):
            self.get_logger().info('Configuration Failure: Invalid sensor mode')
            return TransitionCallbackReturn.FAILURE

        if self.res not in (4, 8):
            self.get_logger().info('Configuration Failure: Invalid resolution')
            return TransitionCallbackReturn.FAILURE

        # Only distance is published: dropping the other outputs shrinks every frame's I2C read and parse
        self.sensor = ToFImager(
            disable_ambient_per_spad=True, disable_nb_spads_enabled=True, disable_nb_target_detected=True,
            disable_signal_per_spad=True, disable_range_sigma_mm=True, disable_reflectance_percent=True,
            disable_target_status=True, disable_motion_indicator=True)
        self.sensor.init()
        self.sensor.set_resolution(self.res*self.res)

        # frequency needs to be set after setting the resolution
        self.frequency = min(self.freq, 15) if self.res == 8 else min(self.freq, 60)
        self.sensor.set_ranging_frequency_hz(self.frequency)
        self.sensor.set_ranging_mode(self.mode)
        self._build_rays()

        if not self.sensor.is_alive():
            self.get_logger().info('Configuration Failure: Sensor not alive')
            return TransitionCallbackReturn.FAILURE
        self.get_logger().info('Configured: Inactive')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Start ranging and the polling timer"""
        try:
            self.sensor.start_ranging()
            self._ranging = True
            self.pcl_pub = self.create_lifecycle_publisher(PointCloud2, 'pointcloud', qos_profile=qos_profile_sensor_data)
            # Poll at least twice per frame: a timer equal to the frame period aliases and adds up to a frame of latency
            period = min(self.get_parameter('timer_period').value, 0.5/self.frequency)
            self.timer = self.create_timer(period, self.publish_pcl)
            self.get_logger().info("Activated")
        except Exception as e:
            self.get_logger().error(f"Activation failed: {e}")
            return TransitionCallbackReturn.FAILURE

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Stop the timer and ranging (the sensor idles instead of drawing power)"""
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None
        if self.pcl_pub is not None:
            self.destroy_publisher(self.pcl_pub)
            self.pcl_pub = None
        self._stop_ranging()

        self.get_logger().info("Deactivated")
        return super().on_deactivate(state)

    def _stop_ranging(self):
        if self._ranging:
            self.sensor.stop_ranging()
            self._ranging = False

    def _close_sensor(self):
        if self.sensor is not None:
            try:
                self._stop_ranging()
                self.sensor._i2c_bus.close()
            except Exception as e:
                self.get_logger().warning(f"Sensor close failed: {e}")
            self.sensor = None

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Release the I2C bus"""
        self.get_logger().info("Cleaning up...")
        self._close_sensor()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shutdown the node"""
        self.get_logger().info("Shutting down...")
        self._close_sensor()
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    rclpy.init(args=args)
    node = ToFImagerPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
