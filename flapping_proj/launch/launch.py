from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
import os


def process_user_input():
    #config here, could later use config file?

    ''''''''''''''''''''''''
    joint_names = ['joint_LW_J_Pitch', 'joint_RW_J_Pitch', 'joint_LW_J_Flap', 'joint_RW_J_Flap']
    model_name = 'URDF_Bodies2SLDASM'
    world_name = 'world1'
    ''''''''''''''''''''''''
    return joint_names, model_name, world_name


def generate_launch_description():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    sdf_file = os.path.join(current_dir, '..', 'gazebo', 'flapping.sdf')
    data_file = os.path.join(current_dir, '..', 'data', 'data.csv')

    joint_names, model_name, world_name = process_user_input()
    position_topic = '/world/' + world_name + '/dynamic_pose/info'  
    joint_control_topics = []
    for joint in joint_names:
        joint_control_topics.append('/' + model_name + '/joint/' + joint + '/cmd_force')  


    #default odom topic, then joint forces
    bridge_args = ['/odom@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V']
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
                {'control_publish_frequency': -1}, #not implemented yet?
                {'data_file_path': data_file}, 
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

    