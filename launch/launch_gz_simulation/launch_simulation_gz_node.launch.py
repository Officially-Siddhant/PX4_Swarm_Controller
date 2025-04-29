#!/usr/bin/env python3

__author__ = "Arthur Astier"
# Modified for Gazebo Ignition / Harmonic by Siddhant Baroth

import os
import json

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def parse_swarm_config(config_file):
    model_counts = {}
    swarm = config_file["swarm"]
    for item in swarm.values():
        model = item["model"]
        model_counts[model] = model_counts.get(model, 0) + 1

    script = ",".join(f"{model}:{count}" for model, count in model_counts.items())

    initial_poses = []
    initial_poses_dict = {}
    for idx, item in enumerate(swarm.values()):
        pose = item["initial_pose"]
        initial_poses_dict[f"px4_{idx+1}"] = pose
        initial_poses.append(f"{pose['x']},{pose['y']}")

    initial_poses_string = "\"" + "|".join(initial_poses) + "\""

    is_leaders = [item["is_leader"] for item in swarm.values()]

    return len(swarm), script, initial_poses_string, initial_poses_dict, is_leaders, config_file["trajectory"]


def generate_launch_description():
    ld = LaunchDescription()
    package_dir = get_package_share_directory('px4_swarm_controller')

    # Load configs
    with open(os.path.join(package_dir, 'config', 'swarm_config.json'), 'r') as f:
        nb_drones, script, initial_poses, initial_poses_dict, is_leaders, trajectory = parse_swarm_config(json.load(f))

    with open(os.path.join(package_dir, 'config', 'control_config.json'), 'r') as f:
        control_config = json.load(f)

    # Extract parameters
    neighborhood = control_config["neighborhood"]
    controller_info = control_config["controller"]

    neighbors_exe = neighborhood["neighbors_exe"]
    neighbors_distance = neighborhood["neighbor_distance"]
    neighbors_params = neighborhood["params"]

    controller_exe = controller_info["controller_exe"]
    controller_params = controller_info["params"]
    leader_follower = controller_info["leader_follower"]

    if leader_follower:
        neighbors_params = {"leaders": is_leaders, **neighbors_params}
    else:
        is_leaders = [False] * len(is_leaders)

    # Launch simulation manager node
    ld.add_action(Node(
        package='px4_swarm_controller',
        executable='simulation_gz_node',
        name='gz_simulation_node',
        parameters=[{'script': script, 'initial_pose': initial_poses}]
    ))

    xs_init, ys_init = [], []

    # Launch each drone node
    for (namespace, pose), is_leader in zip(initial_poses_dict.items(), is_leaders):
        xs_init.append(pose["y"])  # NED to ENU frame convention
        ys_init.append(pose["x"])

        if is_leader:
            ld.add_action(Node(
                package='px4_swarm_controller',
                executable='waypoint',
                name='waypoint',
                namespace=namespace,
                parameters=[{
                    "wp_path": os.path.join(package_dir, "config", "Trajectories", trajectory),
                    "x_init": pose["x"],
                    "y_init": pose["y"]
                }]
            ))
        else:
            ld.add_action(Node(
                package='px4_swarm_controller',
                executable=controller_exe,
                name=controller_exe,
                namespace=namespace,
                parameters=[controller_params]
            ))

    # Launch neighbor calculation node
    ld.add_action(Node(
        package='px4_swarm_controller',
        executable=neighbors_exe,
        name='nearest_neighbors',
        namespace='simulation',
        parameters=[{
            "nb_drones": nb_drones,
            "neighbor_distance": neighbors_distance,
            "x_init": xs_init,
            "y_init": ys_init,
            **neighbors_params
        }]
    ))

    # Launch arming node
    ld.add_action(Node(
        package='px4_swarm_controller',
        executable='arming',
        name='arming',
        namespace='simulation',
        parameters=[{"nb_drones": nb_drones}]
    ))

    return ld
