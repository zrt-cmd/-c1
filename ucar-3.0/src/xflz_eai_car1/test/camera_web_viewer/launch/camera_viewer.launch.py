from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    confi_path = '/home/iflytek/ros2_ws/src/xflz_eai_car1/examples/camera/HP60C/src/ascamera/configurationfiles'

    # HP60C camera node (color_pcl enabled so the point cloud carries RGB)
    ascamera_node = Node(
        namespace='ascamera_hp60c',
        package='ascamera',
        executable='ascamera_node',
        respawn=True,
        output='both',
        parameters=[
            {"usb_bus_no": -1},
            {"usb_path": "null"},
            {"confiPath": confi_path},
            {"color_pcl": True},
            {"pub_tfTree": True},
            {"depth_width": 640},
            {"depth_height": 480},
            {"rgb_width": 640},
            {"rgb_height": 480},
            {"fps": 25},
        ],
        remappings=[]
    )

    # Web viewer server (Flask + SocketIO on 0.0.0.0:8000)
    web_server = Node(
        package='camera_web_viewer',
        executable='server',
        name='camera_web_server',
        output='both',
    )

    ld.add_action(ascamera_node)
    ld.add_action(web_server)
    return ld
