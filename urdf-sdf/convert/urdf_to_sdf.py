import xml.etree.ElementTree as ET
import argparse
import os

def convert_urdf_to_sdf(urdf_fp, sdf_fp, world_name):
    curr_dir = os.path.dirname(os.path.abspath(__file__))
    template_fp = os.path.join(curr_dir, 'template.sdf')
    world_template_tree = ET.parse(template_fp)

    urdf_tree = ET.parse(urdf_fp)
    # urdf_root = urdf_tree.getroot()
    # print("root:" + urdf_tree.tag)

    world_element = world_template_tree.find('world')
    print("world:"  +world_name)
    world_element.attrib['name'] = world_name
    for element in urdf_tree.iter():
        # print(element.tag )
        if element.tag == 'robot':
            print("robot found in URDF:" + element.attrib['name'])
    robot_element = urdf_tree.findall('robot')
    if robot_element is None:
        print("robot not found in URDF")
    if robot_element is not None:
    # for robot_element in urdf_tree.findall('robot'):
        print("robot found in URDF:")
        print(robot_element)
        for robot in robot_element:
            print(robot.attrib['name'])
            world_element.append(robot)
        # robot_element[0].tag = 'model' 
    for model_element in urdf_tree.findall('model'):
        print("model found in URDF:" + model_element.attrib['name'])
        world_element.append(model_element)
    
    sdf_tree = world_template_tree
    base_pose_coord_list = [0] * 6
    for i, model_element in enumerate(world_template_tree.findall('model')):
        pose_element = ET.Element('pose')
        pose_element.attrib['relative_to'] = world_name
        pose_element.text = ' '.join([str(x) for x in base_pose_coord_list])
        base_pose_coord_list[0] += 10 * (i + 1)
        model_element.insert(0, pose_element)  

        for link_element in model_element.findall('link'):
            mass_element = link_element.find('mass')
            if mass_element is not None: #fixes mass format
                mass_value = mass_element.attrib['value']
                mass_element.text = mass_value
                del mass_element.attrib['value']
            
            inertia_element = link_element.find('inertia')
            if inertia_element is not None: #fixes inertia format
                for inertia_axis, value in inertia_element.attrib.items():
                    inertia_axis_element = ET.Element(inertia_axis)
                    inertia_axis_element.text = int(value)
                    inertia_element.append(inertia_axis_element)
            
            visual_element = link_element.find('visual')
            if visual_element is not None: #adds attr
                visual_element.attrib['name'] = 'visual'
            
            collision_element = link_element.find('collision')
            if collision_element is not None: #adds attr
                collision_element.attrib['name'] = 'collision'
                
    #change origin value of links?
     


    sdf_tree.write(sdf_fp)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert URDF to SDF')

    parser.add_argument('-u', '--urdf', help='URDF file path', dest='urdf_path',required=True, type=str)
    parser.add_argument('-s', '--sdf', help='SDF file path', dest='sdf_path', required=True, type=str)
    parser.add_argument('-w', '--world', help='World name', dest='world_name', required=False, type=str, default='world')

    args = parser.parse_args()
    convert_urdf_to_sdf(args.urdf_path, args.sdf_path, args.world_name)

