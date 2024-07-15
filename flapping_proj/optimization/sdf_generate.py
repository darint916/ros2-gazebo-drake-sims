import xml.etree.ElementTree as ET
import json
import os
import numpy as np
from xml.dom.minidom import parseString
from utils.message import Message
# Load and parse the SDF file
curr_path = os.path.dirname(os.path.abspath(__file__))
sdf_file_path = os.path.join(curr_path,'data','iterative_wing.sdf')
Message.info("SDF_Generate: sdf source path: " + sdf_file_path)
tree = ET.parse(sdf_file_path)
root = tree.getroot()

# Load the JSON configuration
json_config_path = os.path.join(curr_path,'data','config.json')
inertia_key = ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']

def generate_sdf(output_path:str=None, chord_cps:np.array= None, spar_cps:np.array= None, blade_area:np.array=None):
    with open(json_config_path, 'r') as json_file:
        config = json.load(json_file)
    link_names = config['link_names']
    existing_links = {link_name: False for link_name in link_names}
    existing_joints = {joint_name: False for joint_name in config['joint_names']}
    phys = root.find('.//physics')
    phys.find('max_step_size').text = str(config['max_step_size']) #max step size
    for model in root.findall('.//model'):
        for link in model.findall('.//link'):
            link_name = link.get('name')
            if link_name in link_names:
                existing_links[link_name] = True
                inertial = link.find('inertial')
                if inertial is not None:
                    mass_element = inertial.find('mass')
                    if mass_element is not None and link_name in config:
                        mass_element.text = str(config[link_name]['mass'])
                    inertia_element = inertial.find('inertia')
                    if inertia_element is not None and link_name in config:
                        inertia = config[link_name]['inertia']
                        for i, element in enumerate(inertia_element):
                            element.text = str(inertia[inertia_key[i]]) #Inertia text
                    pose = inertial.find('pose') #COM position
                    if pose is not None and link_name == "wing":
                        pose.text = ', '.join(map(str, config[link_name]['com']))
        for joint in model.findall('.//joint'):
            joint_name = joint.get('name')
            if joint_name in config['joint_names']:
                existing_joints[joint_name] = True
                dynamics = joint.find('axis').find('dynamics')
                if dynamics is not None:
                    if config[joint_name]['damping'] is not None and dynamics.find('damping') is not None:
                        dynamics.find('damping').text = str(config[joint_name]['damping'])
                    if config[joint_name]['spring_stiffness'] is not None and dynamics.find('spring_stiffness') is not None:
                        dynamics.find('spring_stiffness').text = str(config[joint_name]['spring_stiffness'])
                else:
                    Message.error(f"Joint {joint_name} does not have dynamics")
            
        # print(chord_cps)
        # print("chord above spar below")
        # print( spar_cps)
        if chord_cps is not None and spar_cps is not None:
            for plugin in model.findall('plugin'):
                if plugin.get('name') == 'aerodynamics':
                    link = plugin.find('link')
                    if link is not None:
                        cp_list = link.find('center_pressure_list')
                        if cp_list is not None:
                            cp_in = []
                            for i, spar_cp in enumerate(spar_cps):
                                cp_in.append(spar_cp)
                                cp_in.append(chord_cps[i])
                                cp_in.append(0)
                            cp_list.text = ', '.join(map(str, cp_in))
                        blade_area_element = link.find('blade_area_list')
                        if blade_area_element is not None:
                            blade_area_element.text = ', '.join(map(str, blade_area))
                            # blade_area_element.text = blade_area_element.text[:-2]
                        upward_vector_list = link.find('upward_vector_list')
                        if upward_vector_list is not None:
                            upward_vector_list.text = '0,0,1, ' * len(spar_cps)
                            upward_vector_list.text = upward_vector_list.text[:-2]
                        chord_direction_list = link.find('chord_direction_list')
                        if chord_direction_list is not None:
                            chord_direction_list.text = '0,0,1, ' * len(spar_cps)
                            chord_direction_list.text = chord_direction_list.text[:-2]
                        blades = link.find('blades')
                        if blades is not None:
                            blades.text = str(len(spar_cps))
    for joint_name, exists in existing_joints.items():
        if not exists:
            Message.error(f"Joint {joint_name} does not exist in the SDF file")
    for link_name, exists in existing_links.items():
        if not exists:
            Message.error(f"Link {link_name} does not exist in the SDF file")

    # Add the center of pressure and blade area to the SDF file gazebo extension
    
    if output_path is not None:
        tree.write(output_path)
    else:
        # tree.write(os.path.join(curr_path,'data','processed.sdf'))  #write this so launch can detect
        #make pretty
        formatted_xml = parseString(ET.tostring(root)).toprettyxml(indent="    ")
        line = formatted_xml.split('\n')
        filtered_lines = filter(lambda x: x.strip(), line)
        formatted_xml = '\n'.join(filtered_lines)
        with open(os.path.join(curr_path, 'data', "processed.sdf"), "w") as f:
            f.write(formatted_xml)
        Message.info("SDF_Generate: sdf written to default location: " + os.path.join(curr_path,'data','processed.sdf'))
    return root


if __name__ == '__main__':
    sdf = generate_sdf('processed.sdf')
    