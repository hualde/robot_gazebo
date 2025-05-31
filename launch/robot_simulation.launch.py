#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'robot_gazebo'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Path to the URDF file
    urdf_path = os.path.join(pkg_share, 'urdf', 'differential_robot.urdf.xacro')
    
    # Get URDF via xacro
    robot_description_content = Command(['xacro ', urdf_path])
    
    # Define namespace for the robot
    namespace = 'robot1'
    
    # Group all robot-related nodes under a namespace
    robot_nodes = GroupAction([
        PushRosNamespace(namespace),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}],
            output='screen'
        ),
    ])
    
    # Launch Gazebo Fortress
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '4', 'empty.sdf'],
        output='screen'
    )
    
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        remappings=[
            ('/tf', f'/{namespace}/tf'),
            ('/tf_static', f'/{namespace}/tf_static'),
            ('/joint_states', f'/{namespace}/joint_states')
        ],
        output='screen'
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        arguments=[
            '-topic', f'/{namespace}/robot_description',
            '-name', namespace,
            '-z', '0.1',
            '-allow_renaming', 'false'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        robot_nodes,
        bridge,
        spawn_robot
    ]) 