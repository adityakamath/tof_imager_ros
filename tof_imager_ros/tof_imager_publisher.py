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
import rclpy
import numpy as np
from typing import Optional
from rclpy.lifecycle import Node, Publisher, State, TransitionCallbackReturn
from rclpy.timer import Timer
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from vl53l5cx.vl53l5cx import VL53L5CX as ToFImager, VL53L5CXResultsData as ToFImagerResults

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
        self.sensor = None

        self.get_logger().info('Initialized')

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

        point_dim = 3  # x, y, z
        point_size = point_dim*4  # bytes
        
        try:
            data = self.sensor.get_ranging_data()
        except IndexError:
            data = ToFImagerResults(nb_target_per_zone=1)
        except Exception as e:
            self.get_logger().error(f"Error getting ranging data: {e}")
            return None

        distance_mm = np.array(data.distance_mm[:(self.res*self.res)]).reshape(self.res,self.res)
        buf = np.empty((self.res, self.res, point_dim), dtype=np.float32)
        it = np.nditer(distance_mm, flags=["multi_index"])
        per_px = np.deg2rad(45) / self.res
        for e in it:
            w, h = it.multi_index
            e = 0 if e < 0 else e
            x = e*np.cos(w*per_px - np.deg2rad(45)/2 - np.deg2rad(90))/1000
            y = e*np.sin(h*per_px - np.deg2rad(45)/2)/1000
            z = e/1000
            buf[w][h] = [x, y, z]
        
        return buf, point_size

    def publish_pcl(self):
        """Populate and publish the ToF imager pointcloud"""
        if self.pcl_pub is not None and self.pcl_pub.is_activated:
            sensor_data = self.read_sensor()
            if sensor_data is None:
                return
                
            buf, point_size = sensor_data
            pc_msg = PointCloud2(
                header = Header(
                    stamp = self.get_clock().now().to_msg(),
                    frame_id = self.get_parameter('frame_id').value),
                height = self.res,
                width = self.res,
                fields = [
                    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)],
                is_bigendian = False,
                is_dense = False,
                point_step = point_size,
                row_step = point_size*self.res,
                data = buf.tobytes()
            )  
            self.pcl_pub.publish(pc_msg)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure the ToF imager sensor"""
        self.sensor = ToFImager()
        self.sensor.init()

        if self.mode not in (1, 3):
            self.get_logger().info('Configuration Failure: Invalid sensor mode')
            return TransitionCallbackReturn.FAILURE

        if self.res not in (4, 8):
            self.get_logger().info('Configuration Failure: Invalid resolution')
            return TransitionCallbackReturn.FAILURE
        
        self.sensor.set_resolution(self.res*self.res)

        # frequency needs to be set after setting the resolution
        frequency = min(self.freq, 15) if self.res == 8 else min(self.freq, 60)
        self.sensor.set_ranging_frequency_hz(frequency)
        self.sensor.set_ranging_mode(self.mode)
        self.sensor.start_ranging()
        
        if self.sensor.is_alive():
            self.get_logger().info('Configured: Inactive')
            return TransitionCallbackReturn.SUCCESS
        else:
            self.get_logger().info('Configuration Failure: Sensor not alive')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate the node (start timers and data acquisition)"""
        try:
            self.pcl_pub = self.create_lifecycle_publisher(PointCloud2, 'pointcloud', qos_profile=qos_profile_sensor_data)
            self.timer = self.create_timer(self.get_parameter('timer_period').value, self.publish_pcl)
            self.get_logger().info("Activated")
        except Exception as e:
            self.get_logger().error(f"Activation failed: {e}")

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate the node (stop timers and publishing)"""
        
        self.sensor.stop_ranging()
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
        if self.pcl_pub is not None:
            self.destroy_publisher(self.pcl_pub)
            
        self.get_logger().info("Deactivated")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Clean up node resources"""
        self.get_logger().info("Cleaning up...")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shutdown the node"""
        self.get_logger().info("Shutting down...")
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
