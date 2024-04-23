from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
import os
import json
current_dir = os.path.dirname(os.path.abspath(__file__))

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

def parse_opt_config():
    with open(os.path.join(current_dir, '..', 'optimization', 'data', 'config.json'), 'r') as json_file:
        config = json.load(json_file)
        motor_torque_calc_enabled = config['inputs']['motor_torque_calc_enabled']
        frequency = config['inputs']['frequency']
        max_voltage = config['inputs']['max_voltage']
        motor_resistance = 0.01
        motor_torque_constant = 0.01 #arbituary val
        if motor_torque_calc_enabled:
            motor_resistance = config['inputs']['motor_resistance']
            motor_torque_constant = config['inputs']['motor_torque_constant']
        sim_length = config['sim_length']
    return motor_torque_calc_enabled, frequency, max_voltage, motor_resistance, motor_torque_constant, sim_length

def generate_launch_description():
    sdf_file = os.path.join(current_dir, '..', 'optimization', 'data', 'processed.sdf')
    # sdf_file = os.path.join(current_dir, '..', 'gazebo', 'flapping.sdf')

    data_file = os.path.join(current_dir, '..', 'optimization', 'data','data.csv')
    pid_data_file = os.path.join(current_dir, '..', 'optimization', 'data', 'pid_data.csv')
    
    kill_flag_path = os.path.join(current_dir, '..', 'optimization', '_kill_me.txt') #synchronized with top_level poll

    joint_names, model_name = process_user_input()
    motor_torque_calc_enabled, frequency, max_voltage, motor_resistance, motor_torque_constant, sim_length = parse_opt_config()

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
                {'amplitude': 1.78}, 
                {'frequency': frequency}, 
                {'altitude_pid_enabled': True},
                {'altitude_kp': 0.3},
                {'altitude_ki': 0.001},
                {'altitude_kd': 0.02},
                {'altitude_max_pid_output': 6.27},
                {'altitude_max_integral': 3.0},
                {'static_altitude': 10.0},
                {'pid_data_enabled': False},
                {'pid_data_file_path': pid_data_file},
                {'motor_torque_calc_enabled': motor_torque_calc_enabled},
                {'max_voltage': max_voltage}, #AC voltage sin wave typically 6 V
                {'motor_resistance': motor_resistance},
                {'motor_torque_constant': motor_torque_constant},
                {'sim_length': sim_length}, #duration before kill poll
                {'kill_flag_path': kill_flag_path}
            ]
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=bridge_args,
        )
    ])

    launch_description.add_action(ExecuteProcess(cmd=['ign', 'gazebo', '-r', sdf_file]))

    return launch_description

    