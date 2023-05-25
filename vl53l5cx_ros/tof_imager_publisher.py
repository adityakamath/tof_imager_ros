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
from vl53l5cx.vl53l5cx import VL53L5CX, VL53L5CXResultsData

class ToFImagerPublisher(Node):
    def __init__(self, node_name='tof_imager'):
        super().__init__(node_name)
        self._pc_pub: Optional[Publisher] = None
        self._timer: Optional[Timer] = None

        self.declare_parameter('pointcloud_topic', 'pointcloud')
        self.declare_parameter('frame_id', 'tof_frame')
        self.declare_parameter('resolution', 8)
        self.declare_parameter('mode', 1) #1 is continuous, 3 is autonomous
        self.declare_parameter('ranging_freq', 15)
        self.declare_parameter('timer_period', 0.1)

        self._res = self.get_parameter('resolution').value 
        self._mode = self.get_parameter('mode').value
        self._freq = self.get_parameter('ranging_freq').value
        self._sensor = None

        self.get_logger().info('Initialized')

    def publish_pc(self):
        if self._pc_pub is not None and self._pc_pub.is_activated:
            if not self._sensor.check_data_ready():
                return

            point_dim = 3 # x, y, z
            point_size = point_dim*4 # bytes
            
            try:
                data = self._sensor.get_ranging_data()
            except IndexError:
                data = VL53L5CXResultsData(nb_target_per_zone=1)

            distance_mm = np.array(data.distance_mm[:(self._res*self._res)]).reshape(self._res,self._res)
            buf = np.empty((self._res, self._res, point_dim), dtype=np.float32)
            it = np.nditer(distance_mm, flags=["multi_index"])
            per_px = np.deg2rad(45) / self._res
            for e in it:
                w, h = it.multi_index
                e = 0 if e < 0 else e
                x = e*np.cos(w*per_px - np.deg2rad(45)/2 - np.deg2rad(90))/1000
                y = e*np.sin(h*per_px - np.deg2rad(45)/2)/1000
                z = e/1000
                buf[w][h] = [x, y, z]

            pc_msg = PointCloud2(
                header = Header(
                    stamp = self.get_clock().now().to_msg(),
                    frame_id = self.get_parameter('frame_id').value),
                height = self._res,
                width = self._res,
                fields = [
                    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)],
                is_bigendian = False,
                is_dense = False,
                point_step = point_size,
                row_step = point_size*self._res,
                data = buf.tobytes()
            )  
            self._pc_pub.publish(pc_msg)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._sensor = VL53L5CX()
        self._sensor.init()

        if self._mode not in (1, 3):
            self.get_logger().info('Configuration Failure: Invalid sensor mode')
            return TransitionCallbackReturn.FAILURE

        if self._res not in (4, 8):
            self.get_logger().info('Configuration Failure: Invalid resolution')
            return TransitionCallbackReturn.FAILURE
        
        self._sensor.set_resolution(self._res*self._res)

        # frequency needs to be set after setting the resolution
        frequency = min(self._freq, 15) if self._res == 8 else min(self._freq, 60)
        self._sensor.set_ranging_frequency_hz(frequency)
        self._sensor.set_ranging_mode(self._mode)
        self._sensor.start_ranging()
        
        if self._sensor.is_alive():
            qos_profile = qos_profile_sensor_data
            self._pc_pub = self.create_lifecycle_publisher(PointCloud2, 
                                                           self.get_parameter('pointcloud_topic').value, 
                                                           qos_profile=qos_profile)
            self._timer = self.create_timer(self.get_parameter('timer_period').value, self.publish_pc)
            
            self.get_logger().info('Configured')
            return TransitionCallbackReturn.SUCCESS
        else:
            self.get_logger().info('Configuration Failure: Sensor not alive')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activated')
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivated')
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.terminate()
        self.get_logger().info('Clean Up Successful')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.terminate()
        self.get_logger().info('Shut Down Successful')
        return TransitionCallbackReturn.SUCCESS
        
    def terminate(self):
        self._sensor.stop_ranging()
        if self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
        if self._pc_pub is not None:
            self.destroy_publisher(self._pc_pub)

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
        node.terminate()
        node.destroy_node()

if __name__ == '__main__':
    main()