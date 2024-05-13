from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


import os
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    world_path = PathJoinSubstitution([FindPackageShare('tortoisebot_gazebo'), 'worlds', 'room2.sdf'])
                                       
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={'world': world_path}.items()
    )

    robot_description_path =  os.path.join(get_package_share_directory('tortoisebot_description') , 'models', 'urdf', 'tortoisebot.xacro')
    robot_description_content = Command(['xacro ', robot_description_path])

 
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'robot_description': ParameterValue(robot_description_content, value_type=str)}]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'tortoisebot', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0'],
    )

    rviz_file_path = os.path.join(get_package_share_directory('move_in_circle'), 'rviz', 'display_rviz.rviz')
    print("rviz_file_path: ", rviz_file_path)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_file_path  ]
    )


    move_in_circle_node = Node(
        package='move_in_circle',
        executable='move_in_circle',
        output='screen'
    )



    return LaunchDescription([
        gazebo_node,
        robot_state_publisher_node,
        joint_state_publisher,
        spawn_entity_node,
        rviz_node,
        move_in_circle_node
    ])