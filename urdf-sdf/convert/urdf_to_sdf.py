import xml.etree.ElementTree as ET
import argparse
import os
import xml.dom.minidom as minidom

def attrib_to_text(element, attrib_name):
    if element is None: return
    if attrib_name in element.attrib:
        element.text = element.attrib[attrib_name]
        del element.attrib[attrib_name]

def attrib_to_element(element, attrib_name, tag_name=None):
    if element is None: return
    if tag_name is None:
        tag_name = attrib_name
    if attrib_name in element.attrib:
        attrib_value = element.attrib[attrib_name]
        attrib_element = ET.Element(tag_name)
        attrib_element.text = attrib_value
        del element.attrib[attrib_name]
        element.append(attrib_element)

def all_attrib_to_element(element):
    if element is None: return
    attrib_list = list(element.attrib)
    for attrib_name in attrib_list:
        attrib_to_element(element, attrib_name)

def convert_urdf_to_sdf(urdf_fp, sdf_fp, world_name):
    curr_dir = os.path.dirname(os.path.abspath(__file__))
    template_fp = os.path.join(curr_dir, 'template.sdf')
    world_template_tree = ET.parse(template_fp)

    urdf_tree = ET.parse(urdf_fp)
    # urdf_root = urdf_tree.getroot()
    # print("root:" + urdf_tree.tag)

    world_element = world_template_tree.find('world')
    if world_element is None:
        print("world not found in SDF")
        return None
    print("world:" + world_name)
    world_element.attrib['name'] = world_name
    for element in urdf_tree.iter('robot'):
        # print(element.tag )
            # print("robot found in URDF:" + element.attrib['name'])
        element.tag = 'model'
        world_element.append(element)
    # robot_element = urdf_tree.findall('robot')
    # if robot_element is None:
    #     print("robot not found in URDF")
    # if robot_element is not None:
    # # for robot_element in urdf_tree.findall('robot'):
    #     print("robot found in URDF:")
    #     print(robot_element)
    #     for robot in robot_element:
    #         print(robot.attrib['name'])
    #         world_element.append(robot)
    #     # robot_element[0].tag = 'model' 
    # for model_element in urdf_tree.findall('model'):
    #     print("model found in URDF:" + model_element.attrib['name'])
    #     world_element.append(model_element)
    
    sdf_tree = world_template_tree
    base_pose_coord_list = [0] * 6
    for i, model_element in enumerate(sdf_tree.iter('model')):
        print("model with findall found in SDF:" + model_element.attrib['name'])
        pose_element = ET.Element('pose')
        pose_element.attrib['relative_to'] = world_name
        pose_element.text = ' '.join([str(x) for x in base_pose_coord_list])
        base_pose_coord_list[0] += 10 * (i + 1)
        model_element.insert(0, pose_element)  

        for link_element in model_element.findall('link'):
            print("link found in SDF:" + link_element.attrib['name'])
            inertial_element = link_element.find('inertial')
            if inertial_element is not None: 
                attrib_to_text(inertial_element.find('mass'), 'value')
                all_attrib_to_element(inertial_element.find('inertia'))

            visual_element = link_element.find('visual')
            if visual_element is not None: #adds attr
                visual_element.attrib['name'] = 'visual'
            
            collision_element = link_element.find('collision')
            if collision_element is not None: #adds attr
                collision_element.attrib['name'] = 'collision'

        for joint_element in model_element.findall('joint'):
            print("joint found in SDF:" + joint_element.attrib['name'])
            attrib_to_text(joint_element.find('parent'), 'link')
            attrib_to_text(joint_element.find('child'), 'link')
            all_attrib_to_element(joint_element.find('axis'))
            all_attrib_to_element(joint_element.find('limit'))
            limit = joint_element.find('limit')
            axis = joint_element.find('axis')
            if limit is not None and axis is not None:
                joint_element.remove(limit)
                axis.append(limit)
                xyz = axis.find('xyz')
                if xyz is not None:
                    xyz.attrib['expressed_in'] = '__model__' #Not sure if needed? allows for xyz axis in respect to model frame
        
    for origin in sdf_tree.iter('origin'):
        print("origin found in SDF:" + origin.attrib['xyz'])
        origin.tag = 'pose'
        origin.text = origin.attrib['xyz'] + ' ' + origin.attrib['rpy']
        origin.attrib.clear()
    #change origin value of links?
    
    for mesh in sdf_tree.iter('mesh'):
        print("mesh found in SDF:" + mesh.attrib['filename'])
        attrib_to_element(mesh, 'filename', 'uri')

    sdf_root = sdf_tree.getroot()
    formatted_xml = minidom.parseString(ET.tostring(sdf_root)).toprettyxml(indent="   ")
    line = formatted_xml.split('\n')
    filtered_lines = filter(lambda x: x.strip(), line)
    formatted_xml = '\n'.join(filtered_lines)
    with open(sdf_fp, "w") as f:
        f.write(formatted_xml)
    # sdf_tree.write(sdf_fp)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert URDF to SDF')

    parser.add_argument('-u', '--urdf', help='URDF file path', dest='urdf_path',required=True, type=str)
    parser.add_argument('-s', '--sdf', help='SDF file path', dest='sdf_path', required=True, type=str)
    parser.add_argument('-w', '--world', help='World name', dest='world_name', required=False, type=str, default='world')

    args = parser.parse_args()
    convert_urdf_to_sdf(args.urdf_path, args.sdf_path, args.world_name)

