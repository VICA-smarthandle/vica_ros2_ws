"""VICA D455 + nvblox bringup.

Isaac ROS Docker 안에서 실행한다(nvblox_ros / nvblox_examples_bringup가 Docker
workspace에만 있음). 기존 run_nvblox.sh의 긴 수동 명령을 그대로 이관하고,
esdf_slice_min_height를 바닥 제외 값으로 올린다.

    ros2 launch vica_nvblox_bringup vica_nvblox.launch.py

파라미터 우선순위: nvblox_base.yaml < nvblox_realsense.yaml <
vica_nvblox_overrides.yaml < launch 인자 override(dict). 뒤가 우선한다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    nvblox_examples_dir = get_package_share_directory("nvblox_examples_bringup")
    vica_nvblox_dir = get_package_share_directory("vica_nvblox_bringup")

    nvblox_base = os.path.join(
        nvblox_examples_dir, "config", "nvblox", "nvblox_base.yaml"
    )
    nvblox_realsense = os.path.join(
        nvblox_examples_dir,
        "config",
        "nvblox",
        "specializations",
        "nvblox_realsense.yaml",
    )
    vica_overrides = os.path.join(
        vica_nvblox_dir, "config", "vica_nvblox_overrides.yaml"
    )

    global_frame = LaunchConfiguration("global_frame")
    map_clearing_frame_id = LaunchConfiguration("map_clearing_frame_id")
    # 바닥을 slice 밴드에서 제외하는 1차 튜닝 노브. float으로 캐스팅해서 넘긴다.
    esdf_slice_min_height = ParameterValue(
        LaunchConfiguration("esdf_slice_min_height"), value_type=float
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "global_frame",
            default_value="odom",
            description="local costmap global_frame과 일치해야 한다(nav2_costmap_global_frame).",
        ),
        DeclareLaunchArgument(
            "map_clearing_frame_id",
            default_value="camera_link",
            description="map clearing / esdf slice bounds 시각화 기준 frame.",
        ),
        DeclareLaunchArgument(
            "esdf_slice_min_height",
            default_value="0.05",
            description="바닥(odom z≈0)을 slice에서 제외하는 튜닝 노브(범위 0.03~0.15).",
        ),
        Node(
            package="nvblox_ros",
            executable="nvblox_node",
            name="nvblox_node",
            output="screen",
            parameters=[
                nvblox_base,
                nvblox_realsense,
                vica_overrides,
                {
                    "global_frame": global_frame,
                    "num_cameras": 1,
                    "map_clearing_frame_id": map_clearing_frame_id,
                    "esdf_slice_bounds_visualization_attachment_frame_id":
                        map_clearing_frame_id,
                    # launch 인자로 빠르게 튜닝할 수 있도록 양쪽 mapper에 반영.
                    "static_mapper.esdf_slice_min_height": esdf_slice_min_height,
                    "dynamic_mapper.esdf_slice_min_height": esdf_slice_min_height,
                },
            ],
            remappings=[
                ("camera_0/depth/image", "/camera/camera/depth/image_rect_raw"),
                ("camera_0/depth/camera_info", "/camera/camera/depth/camera_info"),
                ("camera_0/color/image", "/camera/camera/color/image_raw"),
                ("camera_0/color/camera_info", "/camera/camera/color/camera_info"),
                # 2026-08-15 human 모드. use_segmentation: true 일 때만 구독한다.
                #
                # nvblox 는 depth · depth/camera_info · mask · mask/camera_info
                # **네 개를 시간 동기**시킨다(nvblox_node.cpp:269 의 4-way
                # ApproximateTimeSynchronizer). 그래서 마스크만으로는 부족하고
                # 짝이 되는 camera_info 가 반드시 있어야 한다.
                #
                # camera_info 는 리사이즈된 960x544 쪽이어야 한다. 마스크가 그
                # 크기이기 때문이다. 원본 컬러(640x480)의 camera_info 를 물리면
                # 투영이 틀어진다. 실측으로 짝을 확인했다:
                #   people_mask          960x544  camera_color_optical_frame
                #   camera_info_resized  960x544  camera_color_optical_frame
                #
                # 토픽 이름에 camera0 이 두 번 들어가는 것은 오타가 아니다.
                # segmentation.launch.py 가 네임스페이스 안에서 이미 네임스페이스가
                # 붙은 이름을 넘겨서 겹친 결과다. 실제로 그렇게 발행된다.
                ("camera_0/mask/image", "/camera0/segmentation/people_mask"),
                ("camera_0/mask/camera_info",
                 "/camera0/camera0/segmentation/camera_info_resized"),
            ],
        ),
    ])
