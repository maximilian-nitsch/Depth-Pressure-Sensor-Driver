# @license BSD-3 https://opensource.org/licenses/BSD-3-Clause
# Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
# Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
# All rights reserved.

import os

import yaml
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    component = "external_pressure_sensor_coarse"

    # Create the launch description
    ld = LaunchDescription()

    # Declare the path to the config YAML file
    config_file_path = os.path.join(
        get_package_share_directory("external_pressure_sensor_package"),  # noqa
        "config",  # noqa
        "keller_series_10lhpx.yaml",  # noqa
    )

    # Open the YAML file and load the parameters
    with open(config_file_path, "r") as file:
        config = yaml.safe_load(file)

    # Create the node
    depth_pressure_sensor_driver_node = Node(
        package="external_pressure_sensor_package",
        namespace=("/auv/gnc/navigation_sensors/" + component),
        executable="external_pressure_sensor_package_node",
        name=component + "_node",
        output="screen",
        parameters=[config],
    )

    ld.add_action(depth_pressure_sensor_driver_node)

    return ld
