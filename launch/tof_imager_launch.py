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

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, DeclareLaunchArgument
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    ld = LaunchDescription()
    
    tof_imager_params_path = PathJoinSubstitution(
        [FindPackageShare("tof_imager_ros"), "config", "sensor_params.yaml"])

    ns_arg = DeclareLaunchArgument(
        name='ns',
        default_value='', # Do not change, else config params and remappings need to be updated
        description='Namespace of the system')

    frame_id_arg = DeclareLaunchArgument(
        name='frame_id',
        default_value='tof_imager_frame',
        description='Frame ID of the ToF Imager')

    config_path_arg = DeclareLaunchArgument(
        name='config_path',
        default_value=sensehat_config_path,
        description='Sense HAT configuration path')

    ld.add_action(ns_arg)
    ld.add_action(frame_id_arg)
    ld.add_action(config_path_arg)
           
    tof_imager_node = LifecycleNode(
        package='tof_imager_ros',
        executable='tof_imager_publisher',
        name='tof_imager',
        namespace=LaunchConfiguration('ns'),
        output='screen',
        parameters=[
            LaunchConfiguration('config_path'),
            {'frame_id': LaunchConfiguration('frame_id')}]) # this overwrites the config file param

    emit_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher = matches_action(tof_imager_node),
            transition_id = Transition.TRANSITION_CONFIGURE))

    register_activate_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=tof_imager_node, goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher = matches_action(tof_imager_node),
                        transition_id = Transition.TRANSITION_ACTIVATE))]))

    ld.add_action(tof_imager_node)
    ld.add_action(emit_configure_event)
    ld.add_action(register_activate_handler)

    return ld
