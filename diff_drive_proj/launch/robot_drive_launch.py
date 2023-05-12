from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():

    # bridge_pkg = get_package_share_directory('ros_ign_bridge')
    # bridge_launch_file = os.path.join(bridge_pkg, 'launch', 'ros_ign_bridge_launch.py')
    current_dir = os.path.dirname(os.path.abspath(__file__))
    sdf_file = os.path.join(current_dir, '..', '..', 'urdf-sdf', 'building_robot.sdf')
    # launch_ign_gazebo = Command(['ign', 'gazebo', sdf_file])
    # print(sdf_file)
    # ign_gazebo_node = Node(
    #     package='launch',
    #     executable='executable',
    #     arguments=['ign', 'gazebo', sdf_file],
    #     output='screen'
    # )

    launch_description = LaunchDescription([
        Node(
            package='robot_controller',
            executable='controller_node', #check the install folder or what you named the executable (first arg in cmake targets)
            name='robot_controller',  #usually name of class?
            output='screen',
            emulate_tty=True,
            parameters=[
                {'wheel_radius': .4},
                {'wheel_distance': 1.2}
            ]
        ),
        Node(
            package='robot_controller',
            executable='trajectory_server',
            name='trajectory_server',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            # remappings=[ ('/motor_speeds', '/robot_msgs/MotorSpeeds'),    ('/diff_drive', '/twist'),],
            arguments=[
            # '/angle_speed_in@robot_msgs/msg/DifferentialIn@ignition.msgs.Vector2d',
            # motor_speeds:robot_msgs/msg/MotorSpeeds@ignition/msgs/Vector2d', 
            # 'robot_msgs/msg/MotorSpeeds@/motor_speeds@ignition.msgs.Vector2d'
            '/motor_speeds@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/world/car_world/dynamic_pose/info@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
            ]
        )
    ])

    launch_description.add_action(ExecuteProcess(cmd=['ign', 'gazebo', sdf_file]))
    # launch_description.add_action(ign_gazebo_node)

    return launch_description


