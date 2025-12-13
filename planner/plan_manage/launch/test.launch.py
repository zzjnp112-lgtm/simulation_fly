import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # LaunchConfiguration definitions
    init_x = LaunchConfiguration('init_x', default=0.0)
    init_y = LaunchConfiguration('init_y', default=0.0)
    init_z = LaunchConfiguration('init_z', default=0.0)
    target_x = LaunchConfiguration('target_x', default=100.0)
    target_y = LaunchConfiguration('target_y', default=100.0)
    target_z = LaunchConfiguration('target_z', default=100.0)
    drone_id = LaunchConfiguration('drone_id', default=0)
    odom_topic = LaunchConfiguration('odom_topic', default='/mavros/vision_pose/pose')
    obj_num = LaunchConfiguration('obj_num', default=10)

    # DeclareLaunchArgument definitions
    init_x_cmd = DeclareLaunchArgument('init_x', default_value=init_x, description='Initial x position of the drone')
    init_y_cmd = DeclareLaunchArgument('init_y', default_value=init_y, description='Initial y position of the drone')
    init_z_cmd = DeclareLaunchArgument('init_z', default_value=init_z, description='Initial z position of the drone')
    target_x_cmd = DeclareLaunchArgument('target_x', default_value=target_x, description='Target x position')
    target_y_cmd = DeclareLaunchArgument('target_y', default_value=target_y, description='Target y position')
    target_z_cmd = DeclareLaunchArgument('target_z', default_value=target_z, description='Target z position')
    drone_id_cmd = DeclareLaunchArgument('drone_id', default_value=drone_id, description='ID of the drone')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')
    obj_num_cmd = DeclareLaunchArgument('obj_num', default_value=obj_num, description='Number of moving objects')

    # Include advanced parameters
    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'drone_id': drone_id,
            'odometry_topic': odom_topic,
            'obj_num_set': obj_num,
            'cloud_topic': 'cloud_registered_body',
            'max_vel': str(2.0),
            'max_acc': str(3.0),
            'planning_horizon': str(7.5),    
            'flight_type': str(2),
            'point_num': str(1),
            'point0_x': target_x,
            'point0_y': target_y,
            'point0_z': target_z,


        }.items()
    )

    # Trajectory server node
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline'])
        ],
        parameters=[
            {'traj_server/time_forward': 1.0}
        ]
    )

    # Include simulator 
    simulator_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ego_planner'), 'launch', 'simulator.launch.py')),
        launch_arguments={
            'drone_id': drone_id,
            'init_x_': init_x,
            'init_y_': init_y,
            'init_z_': init_z,
            'odometry_topic': odom_topic
        }.items()
    )


    # Create LaunchDescription and add actions
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(init_x_cmd)
    ld.add_action(init_y_cmd)
    ld.add_action(init_z_cmd)
    ld.add_action(target_x_cmd)
    ld.add_action(target_y_cmd)
    ld.add_action(target_z_cmd)
    ld.add_action(drone_id_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(obj_num_cmd)

    # Add nodes and includes
    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)
    ld.add_action(simulator_include)
    # ld.add_action(obj_generator_node)    

    return ld
