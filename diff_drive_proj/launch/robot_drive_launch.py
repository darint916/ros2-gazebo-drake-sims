from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # bridge_pkg = get_package_share_directory('ros_ign_bridge')


    return LaunchDescription([
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
            package='ros_gz_bridge',
            executable='parameter_bridge',
            # remappings=[ ('/motor_speeds', '/robot_msgs/MotorSpeeds'),    ('/diff_drive', '/twist'),],
            arguments=[
            # '/angle_speed_in@robot_msgs/msg/DifferentialIn@ignition.msgs.Vector2d',
            # motor_speeds:robot_msgs/msg/MotorSpeeds@ignition/msgs/Vector2d', 
            # 'robot_msgs/msg/MotorSpeeds@/motor_speeds@ignition.msgs.Vector2d'
            '/motor_speeds@robot_msgs/msgs/MotorSpeeds@gz.msgs.Double'
            
        ]

        )
    ])


