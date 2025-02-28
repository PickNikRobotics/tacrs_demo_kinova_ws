#! /usr/bin/env python3

# Copyright 2024 PickNik Robotics
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

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = FindPackageShare("picknik_006_gen3_hw")

    return LaunchDescription(
        [
            # create a fixed transform between odom and world
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
            ),

            # Set the rigid transforms between the AR tags and the target frame to be tracked.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0.115", "0.100", "0", "0", "0", "0", "satellite_tag_2901", "target_frame_satellite_tag_2901"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0.117", "-0.109", "0", "0", "0", "0", "satellite_tag_2902", "target_frame_satellite_tag_2902"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["-0.102", "0.102", "0", "0", "0", "0", "satellite_tag_2903", "target_frame_satellite_tag_2903"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["-0.103", "-0.113", "0", "0", "0", "0", "satellite_tag_2904", "target_frame_satellite_tag_2904"],
            ),

            # run our estimator
            Node(
                package="fuse_optimizers",
                executable="fixed_lag_smoother_node",
                name="state_estimator",
                parameters=[
                    PathJoinSubstitution([pkg_dir, "config", "fuse", "fuse.yaml"])
                ],
            ),
            # run visualization
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                arguments=[
                    "-d",
                    [PathJoinSubstitution([pkg_dir, "config", "fuse", "fuse.rviz"])],
                    "--ros-args",
                    "--log-level",
                    "warn",
                ],
            ),
        ]
    )
