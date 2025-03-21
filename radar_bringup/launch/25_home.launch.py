from launch import LaunchDescription, actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from tf2_geometry_msgs.tf2_geometry_msgs import _get_quat_from_mat, _build_affine, _decompose_affine

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import TimerAction, Shutdown
from launch import LaunchDescription

import numpy as np
import os

debug = True

node_params = os.path.join(
    get_package_share_directory('radar_bringup'),
    'config',
    'config.25_home.yaml'
)


def get_xyzw_tf_broadcaster(cali: list, fr: str, child_fr: str):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace='radar',
        name=fr+'_to_'+child_fr,
        parameters=[{'use_sim_time': True}],
        arguments=['--x', str(cali[0]),
                   '--y', str(cali[1]),
                   '--z', str(cali[2]),
                   '--qx', str(cali[3]),
                   '--qy', str(cali[4]),
                   '--qz', str(cali[5]),
                   '--qw', str(cali[6]),
                   '--frame-id', fr,
                   '--child-frame-id', child_fr],)


def get_vision_container(cam_name: str):
    if not debug:
        return (ComposableNodeContainer(
            name=cam_name + '_vision_container',
            namespace='radar',
            package='rclcpp_components',
            executable='component_container_isolated',
            arguments=['--use_multi_threaded_executor'],
            composable_node_descriptions=[
                ComposableNode(
                    package='mind_camera',
                    plugin='mind_camera::MindCameraNode',
                    name='mind_camera',
                    namespace='radar/' + cam_name,
                    parameters=[node_params],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='img_recognizer',
                    plugin='img_recognizer::RecognizerNode',
                    name='img_recognizer',
                    namespace='radar/' + cam_name,
                    parameters=[
                        node_params,
                        {'use_sim_time': True,
                         'img_compressed': True},
                        ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ],
            output='both',
            emulate_tty=True,
            on_exit=Shutdown(),
        ),)
    else:
        return (
            Node(
                package='mind_camera',
                executable='mind_camera_node',
                namespace='radar/' + cam_name,
                parameters=[
                    {'use_sim_time': True},
                     {'frame_id': 'mind_camera_frame'},
                    node_params,
                ],
            ),
            Node(
                package='img_recognizer',
                executable='img_recognizer_node',
                namespace='radar/' + cam_name,
                parameters=[
                    node_params,
                    {'use_sim_time': True,
                     'img_compressed': True},
                ],
            ),
        )


def get_pc_container():
    if not debug:
        return (ComposableNodeContainer(
            name='pc_container',
            namespace='radar',
            package='rclcpp_components',
            executable='component_container_isolated',
            arguments=['--use_multi_threaded_executor'],
            composable_node_descriptions=[

                ComposableNode(
                    package='livox_v1_lidar',
                    plugin='livox_v1_lidar::LidarPublisher',
                    name='livox_v1_lidar',
                    namespace='radar/' + 'lidar_mid70',
                    parameters=[node_params],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='pc_detector',
                    plugin='pc_detector::DetectorNode',
                    name='pc_detector',
                    namespace='radar',
                    parameters=[node_params,
                                {'use_sim_time': True}],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ],
            output='both',
            emulate_tty=True,
            on_exit=Shutdown(),
        ),)
    else:
        return (
            Node(
                package='livox_v1_lidar',
                executable='livox_v1_lidar_node',
                name='livox_v1_lidar',
                namespace='radar/' + 'lidar_mid70',
                parameters=[node_params],
            ),
            Node(
                package='pc_detector',
                executable='pc_detector_node',
                name='pc_detector',
                namespace='radar',
                parameters=[node_params,
                            {'use_sim_time': True}],
            ),
        )


def generate_launch_description():
    return LaunchDescription([
            # 添加临时world变换
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace='radar',
            name='world_to_lidar_fallback',
            arguments=['0', '0', '0', '1', '0', '0', '0', 'world', 'lidar_mid70_frame'],
        ),
        
        *get_vision_container('mind_camera'),

        #待标定数据，需要更新)
        get_xyzw_tf_broadcaster(
            [
                0.31, 0.05, -0.12,  # 位置，需要根据实际标定更新
                0.0, -0.08, 0.0, 0.9967  # 方向四元数，需要根据实际标定更新
            ], 'lidar_mid70_frame', 'mind_camera_frame'
        ),
        *get_pc_container(),
        
        Node(
            package='radar_utils',
            executable='marker_pub',
            namespace='radar',
            parameters=[{"mesh": "24_bg2align_fix1.stl"}],
            output='both',
        ),
        Node(
            package='pc_aligner',
            executable='pc_aligner',
            namespace='radar',
            parameters=[
                node_params,
                {'use_sim_time': True},
                {'startup_manual_align': True},  # 禁用手动对齐
                {'init_trans': [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]},# 提供初始变换,还有疑问，需要修改，手动怎么对齐？
            ],
            output='both',
        ),
        Node(
            package='target_matcher',
            executable='target_matcher',
            namespace='radar',
            parameters=[node_params,
                        {'use_sim_time': True}],
            output='both',
        ),
        Node(
            package='dv_trigger',
            executable='dv_trigger',
            namespace='radar',
            parameters=[node_params,
                        {'use_sim_time': True}],
            output='both',
        ),
        Node(
            package='radar_supervisor',
            executable='radar_supervisor',
            namespace='radar',
            parameters=[node_params,
                        {'use_sim_time': True}],
            output='both',
        ),
        # Node(
        #     package='judge_bridge',
        #     executable='judge_bridge',
        #     name='judge_bridge',
        #     namespace='radar',
        #     parameters=[node_params,
        #                 {'use_sim_time': True}],
        #     output='both',
        # ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
        ),
        Node(
            package='target_visualizer',
            executable='target_visualizer',
            namespace='radar',
            output='both',
        ),
        Node(
            package='result_visualizer',
            executable='result_visualizer',
            namespace='radar',
            output='both',
        ),
    ])