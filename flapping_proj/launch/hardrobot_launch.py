import json
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
import os


def process_user_input():
    #config here, could later use config file?
    #TODO: YAML CONFIG FILE FROM GUI
    
    joint_names = ['stroke_joint_1', 'pitch_joint_1', 'stroke_joint_2', 'pitch_joint_2']
    model_name = 'hardrobot' 
    # model_name = 'URDF_Bodies2SLDASM'
    # world_name = 'world1'
    ''''''''''''''''''''''''
    return joint_names, model_name


def generate_launch_description():
    json_config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../optimization/data/input_config.json')
    with open(json_config_path, 'r') as json_file:
        config = json.load(json_file)
    ''''''''''''''''''''''''
    current_dir = os.path.dirname(os.path.abspath(__file__))
    sdf_file = os.path.join(current_dir, '..', 'gazebo', 'hardrobot.sdf')
    # sdf_file = os.path.join(current_dir, '..', 'gazebo', 'flapping.sdf')

    data_file = os.path.join(current_dir, '..', 'data', 'data.csv')
    input_joint_data_file = os.path.join(current_dir, '..', 'data', 'input_joint_data.csv')
    pid_data_file = os.path.join(current_dir, '..', 'data', 'pid_data.csv')
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
                {'control_publish_frequency': 50000}, 
                {'data_file_path': data_file}, 
                {'input_joint_data_file_path': input_joint_data_file},
                {'amplitude': 0.1}, #2.73 best lift, Must have decimal, or ros wont take as a double
                {'frequency': 15.0}, 
                {'altitude_pid_enabled': False},
                {'altitude_kp': 0.3},
                {'altitude_ki': 0.001},
                {'altitude_kd': 0.02},
                {'altitude_max_pid_output': 6.27},
                {'altitude_max_integral': 3.0},
                {'static_altitude': 10.0},
                {'pid_data_enabled': False},
                {'pid_data_file_path': pid_data_file},
                {'motor_torque_calc_enabled': True},
                {'max_voltage': 6.0}, #AC voltage sin wave typically 6 V
                {'motor_resistance': 8.8},
                {'motor_torque_constant': 0.00109},
                {'sim_length': 20.0}, #duration before kill poll
                {'gear_ratio': 20.0},
                {'motor_back_emf': 0.000114}, #V/rpm
                {'motor_dynamic_friction': 0.00000000102}, #Nm/rpm
                # {'kill_flag_path': kill_flag_path}
                #controls
                {'control_opt_en': True},
                {'control_a': config["voltage"]["waves"][0]["amplitude"]},
                {'control_b': config["voltage"]["waves"][1]["amplitude"]},
                {'control_frequency': config["voltage"]["frequency"]},
                {'control_a_phase': config["voltage"]["waves"][0]["phase"]},
                {'control_b_phase': config["voltage"]["waves"][1]["phase"]},
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

    