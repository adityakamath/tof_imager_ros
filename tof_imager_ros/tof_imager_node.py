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
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from tof_imager_ros.tof_imager_publisher import ToFImagerPublisher

def main(args=None):
    rclpy.init(args=args)
    try:
        tof_imager_publisher = ToFImagerPublisher(node_name='tof_imager')

        executor = SingleThreadedExecutor()
        executor.add_node(tof_imager_publisher)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            tof_imager_publisher.destroy_node()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()