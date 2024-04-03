from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
import os


def process_user_input():
    #config here, could later use config file?
    #TODO: YAML CONFIG FILE FROM GUI
    ''''''''''''''''''''''''
    joint_names = ['pitch_joint', 'stroke_joint']
    model_name = 'URDF_LargeWings' 
    # model_name = 'URDF_Bodies2SLDASM'
    # world_name = 'world1'
    ''''''''''''''''''''''''
    return joint_names, model_name


def generate_launch_description():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    sdf_file = os.path.join(current_dir, '..', 'optimization', 'data', 'processed.sdf')
    # sdf_file = os.path.join(current_dir, '..', 'gazebo', 'flapping.sdf')

    data_file = os.path.join(current_dir, '..', 'optimization', 'data','data.csv')
    pid_data_file = os.path.join(current_dir, '..', 'optimization', 'data', 'pid_data.csv')
    joint_names, model_name = process_user_input()
    position_topic = '/odom'  #from sdf plugin
    joint_control_topics = []
    for joint in joint_names:
        joint_control_topics.append('/model/' + model_name + '/joint/' + joint + '/cmd_force')  


    #default odom topic, then joint forces
    bridge_args = ['/odom@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V', '/world/diff_drive/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist']
    for joint_topic in joint_control_topics:
        bridge_args.append(joint_topic + '@std_msgs/msg/Float64]gz.msgs.Double')
    
    launch_description = LaunchDescription([
        Node(
            package='flapping_controller',
            executable='control_node',
            name='control_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'joint_names': joint_names},
                {'joint_control_topics': joint_control_topics},
                {'position_topic': position_topic},
                {'control_publish_frequency': 20000}, 
                {'data_file_path': data_file}, 
                {'amplitude': 8.758}, #2.73 best lift, Must have decimal, or ros wont take as a double
                {'frequency': 3.0}, 
                {'altitude_pid_enabled': True},
                {'altitude_kp': 0.3},
                {'altitude_ki': 0.001},
                {'altitude_kd': 0.02},
                {'altitude_max_pid_output': 6.27},
                {'altitude_max_integral': 3.0},
                {'static_altitude': 10.0},
                {'pid_data_enabled': False},
                {'pid_data_file_path': pid_data_file},
            ]
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=bridge_args,
        )
    ])

    launch_description.add_action(ExecuteProcess(cmd=['ign', 'gazebo', sdf_file]))

    return launch_description

    