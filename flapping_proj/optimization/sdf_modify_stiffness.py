import xml.etree.ElementTree as ET
import os
import numpy as np
from xml.dom.minidom import parseString
from utils.message import Message
# Load and parse the SDF file
curr_path = os.path.dirname(os.path.abspath(__file__))
sdf_file_path = os.path.join(curr_path,'../gazebo/hardrobot_stiff_op.sdf')
Message.info("SDF_Generate: sdf source path: " + sdf_file_path)
tree = ET.parse(sdf_file_path)
root = tree.getroot()
# Load the JSON configuration

def modify_sdf(joint_names: list[str], stroke_stiffness: float, pitch_stiffness: float, stroke_damping: float, output_path: str = None):
    print("modifying: " , sdf_file_path)
    existing_joints = {joint_name: False for joint_name in joint_names}
    for model in root.findall('.//model'):
        for joint in model.findall('.//joint'):
            joint_name = joint.get('name')
            if joint_name in joint_names:
                existing_joints[joint_name] = True
                dynamics = joint.find('axis').find('dynamics')
                if dynamics is not None:
                    if joint_name.startswith('stroke'):
                        if dynamics.find('damping') is not None:
                            dynamics.find('damping').text = str(stroke_damping)
                        if dynamics.find('spring_stiffness') is not None:
                            dynamics.find('spring_stiffness').text = str(stroke_stiffness)
                    else:
                        if dynamics.find('spring_stiffness') is not None:
                            dynamics.find('spring_stiffness').text = str(pitch_stiffness)
                else:
                    Message.error(f"Joint {joint_name} does not have dynamics")
    for joint_name, exists in existing_joints.items():
        if not exists:
            Message.error(f"Joint {joint_name} does not exist in the SDF file")
    if output_path is not None:
        tree.write(output_path)
    else:
        # tree.write(os.path.join(curr_path,'data','processed.sdf'))  #write this so launch can detect
        #make pretty
        formatted_xml = parseString(ET.tostring(root)).toprettyxml(indent="    ")
        line = formatted_xml.split('\n')
        filtered_lines = filter(lambda x: x.strip(), line)
        formatted_xml = '\n'.join(filtered_lines)
        with open(os.path.join(sdf_file_path), "w") as f:
            f.write(formatted_xml)
        Message.info("SDF_Generate: sdf modified in place: ", os.path.join(curr_path,'data','processed.sdf'))
    return root