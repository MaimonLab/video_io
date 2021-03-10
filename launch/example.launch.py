from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory("video_io"), "config", "example.yaml"
    )
    video_player = Node(
        package="video_io",
        executable="video_player",
        name="video_player1",
        parameters=[config],
    )
    ld.add_action(video_player)

    video_saver = Node(
        package="video_io",
        executable="video_saver",
        name="video_saver1",
        parameters=[config],
    )
    ld.add_action(video_saver)

    multithread_video_saver = Node(
        package="video_io",
        executable="multithread_video_saver",
        name="video_saver2",
        parameters=[config],
    )
    ld.add_action(multithread_video_saver)

    rqt_image_view = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="image_viewer",
        arguments=["/video_player1/image"],
    )
    ld.add_action(rqt_image_view)

    return ld