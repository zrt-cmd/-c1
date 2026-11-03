#!/usr/bin/env python3
"""Launch HP60C depth camera (ascamera driver).

CRITICAL: Do NOT remap the camera topics and do NOT set a custom node name.
The ascamera driver decides which stream to start by searching the PUBLISHED
topic name for the substring "rgb0"/"depth0"/"ir0" (see CameraPublisher.cpp
getSubTopicStreamType). If a remap renames the topic to e.g.
"/depth_cam/rgb/image_raw" the "rgb0" substring is lost, streamType becomes 0
and the camera NEVER streams. So we keep the driver's native topic names:
  /ascamera_hp60c/camera_publisher/rgb0/image
  /ascamera_hp60c/camera_publisher/depth0/image_raw
  /ascamera_hp60c/camera_publisher/depth0/camera_info
  /ascamera_hp60c/camera_publisher/depth0/points
Downstream nodes (visual_grasp) subscribe to these native names directly.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ascamera_node = Node(
        namespace='ascamera_hp60c',
        package='ascamera',
        executable='ascamera_node',
        respawn=False,
        output='both',
        parameters=[
            {"confiPath": "/home/iflytek/ros2_ws/src/xflz_eai_car1/examples/camera/HP60C/src/ascamera/configurationfiles"},
            {"usb_bus_no": -1},
            {"usb_path": "null"},
            {"color_pcl": False},
            {"pub_tfTree": False},
            {"depth_width": 640},
            {"depth_height": 480},
            {"rgb_width": 640},
            {"rgb_height": 480},
            {"fps": 15},
        ],
        remappings=[]
    )

    return LaunchDescription([ascamera_node])
