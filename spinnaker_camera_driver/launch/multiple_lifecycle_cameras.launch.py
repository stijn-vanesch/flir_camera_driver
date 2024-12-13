# -----------------------------------------------------------------------------
# Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
# from launch.ros.descriptions import ComposableLifecycleNode

camera_params = {
    'debug': False,
    'compute_brightness': False,
    'adjust_timestamp': True,
    'dump_node_map': False,
    'enable_external_control': True,
    # set parameters defined in blackfly_s.yaml
    'gain_auto': 'Off',
    'gain': 18,
    # 'pixel_format': 'BayerRG8',
    'exposure_auto': 'Off',
    'exposure_time': 14000,
    # to use a user set, do this:   
    'user_set_selector': 'UserSet0',
    'user_set_load': 'Yes',
    # These are useful for GigE cameras
    'device_link_throughput_limit': 60000000,
    # 'gev_scps_packet_size': 9000,
    # ---- to reduce the sensor width and shift the crop
    
    'image_width': 2448,
    'image_height': 2048,
    'offset_x': 0,
    'offset_y': 0,
    # 'binning_x': 1,
    # 'binning_y': 1,
    # 'connect_while_subscribed': True,
    'frame_rate_auto': 'Off',
    'frame_rate': 10.0,
    'frame_rate_enable': True,
    'buffer_queue_size': 10,
    'trigger_mode': 'Off',
    'chunk_mode_active': True,
    'chunk_selector_frame_id': 'FrameID',
    'chunk_enable_frame_id': True,
    'chunk_selector_exposure_time': 'ExposureTime',
    'chunk_enable_exposure_time': True,
    'chunk_selector_gain': 'Gain',
    'chunk_enable_gain': True,
    'chunk_selector_timestamp': 'Timestamp',
    'chunk_enable_timestamp': True,
}


def make_camera_node(name, camera_type, serial):
    parameter_file = PathJoinSubstitution(
        [FindPackageShare('spinnaker_camera_driver'), 'config', camera_type + '.yaml']
    )

    node = ComposableNode(
        package='spinnaker_camera_driver',
        plugin='spinnaker_camera_driver::CameraLifecycle',
        name=name,
        parameters=[camera_params, {'parameter_file': parameter_file, 'serial_number': serial}],
        remappings=[
            ('~/control', '/exposure_control/control'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    return node


def launch_setup(context, *args, **kwargs):
    """Create multiple camera."""
    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            #
            # These two camera nodes run independently from each other,
            # but in the same address space
            #
            make_camera_node(
                LaunchConfig('cam_0_name'),
                LaunchConfig('cam_0_type').perform(context),
                LaunchConfig('cam_0_serial'),
            ),
            make_camera_node(
                LaunchConfig('cam_1_name'),
                LaunchConfig('cam_1_type').perform(context),
                LaunchConfig('cam_1_serial'),
            ),
        ],
        output='screen',
    )  # end of container
    return [container]


def generate_launch_description():
    """Create composable node by calling opaque function."""
    return LaunchDescription(
        [
            LaunchArg(
                'cam_0_name',
                default_value=['cam_0'],
                description='camera name (ros node name) of camera 0',
            ),
            LaunchArg(
                'cam_1_name',
                default_value=['cam_1'],
                description='camera name (ros node name) of camera 1',
            ),
            LaunchArg('cam_0_type', default_value='blackfly_s', description='type of camera 0'),
            LaunchArg('cam_1_type', default_value='blackfly_s', description='type of camera 1'),
            LaunchArg(
                'cam_0_serial',
                default_value="'23306238'",
                description='FLIR serial number of camera 0 (in quotes!!)',
            ),
            LaunchArg(
                'cam_1_serial',
                default_value="'23054572'",
                description='FLIR serial number of camera 1 (in quotes!!)',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
