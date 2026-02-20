import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the path to the YAML config file
    ekf_config_path = os.path.join(
        get_package_share_directory("ahrs"), "config", "ekf_ahrs.yaml"
    )
    mag_cal_config_file = os.path.join(
        get_package_share_directory("mag_corrector"), "config", "calibration.yaml"
    )

    return LaunchDescription([
        # 1. ICM-20948 Driver
        Node(
            package="ros2_icm20948",
            executable="icm20948_node",
            name="icm20948",
            parameters=[
                {"i2c_address": 0x68},
                {"frame_id": "imu_icm20948"},
                {"pub_rate": 100},
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_pub_imu",
            # Arguments: x y z roll pitch yaw parent_frame child_frame
            # Replace 'imu_link' with the actual output you found in the step above
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_icm20948"],
        ),
        # 2. Mag Calibration
        Node(
            package="mag_corrector",
            executable="corrector",
            name="mag_corrector",
            parameters=[mag_cal_config_file],  # <--- Loads the YAML
        ),
        # 3. Madgwick Filter
        Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            name="imu_filter",
            output="screen",
            parameters=[
                {
                    "use_mag": True,
                    "publish_tf": True,
                    "world_frame": "enu",
                    "publish_debug_topics": True,
                }
            ],
        ),
        # 4. Robot Localization
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_config_path],  # Loads the YAML file found above
        ),
    ])
