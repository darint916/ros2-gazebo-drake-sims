import xml.etree.ElementTree as ET
import json
import os
from xml.dom.minidom import parseString
# Load and parse the SDF file
curr_path = os.path.dirname(os.path.abspath(__file__))
sdf_file_path = os.path.join(curr_path,'data','iterative_wing.sdf')
print("SDF_Generate: sdf source path: ", sdf_file_path)
tree = ET.parse(sdf_file_path)
root = tree.getroot()

# Load the JSON configuration
json_config_path = os.path.join(curr_path,'data','config.json')
with open(json_config_path, 'r') as json_file:
    config = json.load(json_file)
inertia_key = ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']
def generate_sdf(output_path:str=None):
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
        for joint in model.findall('.//joint'):
            joint_name = joint.get('name')
            if joint_name in config['joint_names']:
                existing_joints[joint_name] = True
                axis = joint.find('axis')
                if config[joint_name]['damping'] is not None and joint.find('damping') is not None:
                    joint.find('damping').text = str(config[joint_name]['damping'])
                if config[joint_name]['spring_stiffness'] is not None and joint.find('spring_stiffness') is not None:
                    joint.find('spring_stiffness').text = str(config[joint_name]['spring_stiffness'])
    for joint_name, exists in existing_joints.items():
        if not exists:
            print(f"Joint {joint_name} does not exist in the SDF file")
    for link_name, exists in existing_links.items():
        if not exists:
            print(f"Link {link_name} does not exist in the SDF file")
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
        print("SDF_Generate: sdf written to default location: ", os.path.join(curr_path,'data','processed.sdf'))
    return root


if __name__ == '__main__':
    sdf = generate_sdf('processed.sdf')
    