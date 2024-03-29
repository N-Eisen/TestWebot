"""Launch Webots Universal Robot simulation."""

import os
import xacro
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


PACKAGE_NAME = 'final_package'


# Define all the ROS 2 nodes that need to be restart on simulation reset here
def get_ros2_nodes(*args):
    package_dir = get_package_share_directory(PACKAGE_NAME)
    ur5e_xacro_path = os.path.join(package_dir, 'resource', 'ur5e_with_gripper.urdf.xacro')
    ur5e_description = xacro.process_file(ur5e_xacro_path, mappings={'name': 'UR5eWithGripper'}).toxml()

    # Define your URDF robot here
    # The name of an URDF robot has to match the name of the robot of the driver node
    # You can specify the URDF content to use with robot_description
    # In case you have relative paths in your URDF, specify the relative_path_prefix as the directory of your xacro file
    spawn_URDF_ur5e = URDFSpawner(
        name='UR5e',
        robot_description=ur5e_description,
        relative_path_prefix=os.path.join(package_dir, 'resource'),
        translation='0 0 0.62',
        rotation='0 0 1 -1.5708',
    )

    # ROS control spawners
    controller_manager_timeout = ['--controller-manager-timeout', '500']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    ur5e_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_trajectory_controller', '-c', 'ur5e/controller_manager'] + controller_manager_timeout,
    )
    ur5e_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_state_broadcaster', '-c', 'ur5e/controller_manager'] + controller_manager_timeout,
    )
    ur5e_spawners = [ur5e_trajectory_controller_spawner, ur5e_joint_state_broadcaster_spawner]
    
    # Control nodes
    ur5e_controller = Node(
        package=PACKAGE_NAME,
        executable='ur5e_controller',
        namespace='ur5e',
        output='screen'
    )
    

    return [
        # Request to spawn the URDF robot
        spawn_URDF_ur5e,

        # Launch the driver node once the URDF robot is spawned.
        # You might include other nodes to start them with the driver node.
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessIO(
                on_stdout=lambda event: get_webots_driver_node(
                    event, [ur5e_controller] + ur5e_spawners 
                ),
            )
        ),
    ]


def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    world = LaunchConfiguration('world')

    # Starts Webots
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    # Driver nodes
    # When having multiple robot it is mandatory to specify the robot name.
    ur5e_xacro_path = os.path.join(package_dir, 'resource', 'ur5e_with_gripper.urdf.xacro')
    ur5e_control_params = os.path.join(package_dir, 'resource', 'ros2_control_config.yaml')
    ur5e_driver = WebotsController(
        robot_name='UR5e',
        namespace='ur5e',
        parameters=[
            {'robot_description': ur5e_xacro_path},
            {'xacro_mappings': ['name:=UR5eWithGripper']},
            {'use_sim_time': True},
            ur5e_control_params
        ],
        respawn=True
    )

    

    # The following line is important!
    # This event handler respawns the ROS 2 nodes on simulation reset (supervisor process ends).
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots._supervisor,
            on_exit=get_ros2_nodes,
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='robotic_arms.wbt',
            description='Choose one of the world files from `/final_package/worlds` directory'
        ),
        webots,
        webots._supervisor,

        ur5e_driver,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.UnregisterEventHandler(
                        event_handler=reset_handler.event_handler
                    ),
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        ),

        # Add the reset event handler
        reset_handler
    ] + get_ros2_nodes())