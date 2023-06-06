import xml.etree.ElementTree as ET
import argparse
import os
import xml.dom.minidom as minidom

''' main changes 
Converting attributes to text
adding new material
changing routing
#TODO add dynamics to joints

'''


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

def delete_material(sdf_tree): #deletes all material except for ground_plane def 
    for model in sdf_tree.iter('model'):
        if model.attrib['name'] == 'ground_plane':
            ground_vis = model.find('link').find('visual')
            ground_material = ground_vis.find('material')
    for visual_element in sdf_tree.iter('visual'):
        material_element = visual_element.find('material')
        if material_element is not None:
            visual_element.remove(material_element)
    ground_vis.append(ground_material)

#insert custom material settings
def insert_material_snippet(sdf_tree, snippet = ''' 
        <ambient>0.8 0.8 0.8 1</ambient>
        <diffuse>0.8 0.8 0.8 1</diffuse>
        <specular>0.8 0.8 0.8 1</specular>
    '''): #basically dummy values that get replacd by the rgba values that were originally in urdf
    visual_parse = ET.fromstring('<root>' + snippet + '</root>')
    for material_element in sdf_tree.iter('material'):
        material_element.attrib.clear()
        color_element = material_element.find('color')
        if color_element is not None:
            color_val = color_element.attrib['rgba']
            material_element.remove(color_element)
        for visual_child in visual_parse.iter():
            visual_child.text = color_val
            material_element.append(visual_child)


def origin_to_pose(origin_element, relative_name=None):
    if origin_element is None: return
    origin_element.tag = 'pose'
    origin_element.text = origin_element.attrib['xyz'] + ' ' + origin_element.attrib['rpy']
    origin_element.attrib.clear()
    if relative_name is not None:
        origin_element.attrib['relative_to'] = relative_name

def add_revolute_joint_limit(sdf_tree, limit_snippet=
    '''<limit>
        <lower>-1.79769e+308</lower>    <!--negative infinity-->
        <upper>1.79769e+308</upper>     <!--positive infinity-->
    </limit>'''):
    limit_parse = ET.fromstring('<root>' + limit_snippet + '</root>')
    for joint_element in sdf_tree.iter('joint'):

            #remove outer limit
            # limit = joint_element.find('limit')
            # axis = joint_element.find('axis')
            # if limit is not None and axis is not None:
            #     joint_element.remove(limit)

        if joint_element.attrib['type'] == 'revolute':
            axis_element = joint_element.find('axis')
            outer_limit = joint_element.find('limit')
            if outer_limit is not None and axis_element is not None:
                joint_element.remove(outer_limit)
            if axis_element is not None:
                limit_element = axis_element.find('limit')
                if limit_element is not None:
                    axis_element.remove(limit_element)
                axis_element.append(limit_parse.find('limit')) #removes root wrapper
                    
    

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
    # print("world:" + world_name)
    world_element.attrib['name'] = world_name
    for element in urdf_tree.iter('robot'):
        element.tag = 'model'
        world_element.append(element)
    
    sdf_tree = world_template_tree
    base_pose_coord_list = [0] * 6
    for i, model_element in enumerate(sdf_tree.iter('model')):
        # print("model with findall found in SDF:" + model_element.attrib['name'])
        pose_element = ET.Element('pose')
        pose_element.attrib['relative_to'] = 'world' #might cause error?
        pose_element.text = ' '.join([str(x) for x in base_pose_coord_list])
        base_pose_coord_list[0] += 10 * i
        model_element.insert(0, pose_element)  

        for link_element in model_element.findall('link'):
            # print("link found in SDF:" + link_element.attrib['name'])
            inertial_element = link_element.find('inertial')
            if inertial_element is not None: 
                attrib_to_text(inertial_element.find('mass'), 'value')
                all_attrib_to_element(inertial_element.find('inertia'))

            visual_element = link_element.find('visual')
            if visual_element is not None: #adds attr
                visual_element.attrib['name'] = link_element.attrib['name']
            #     origin_element = visual_element.find('origin')
            #     if origin_element is not None:
            #         visual_element.remove(origin_element)

            collision_element = link_element.find('collision')
            if collision_element is not None: #adds attr
                collision_element.attrib['name'] = link_element.attrib['name']
            #     origin_element = collision_element.find('origin')
            #     if origin_element is not None:
            #         collision_element.remove(origin_element)
        
        joint_child_map = {} #child link: parent joint
        for joint_element in model_element.findall('joint'):
            joint_element.attrib['name'] = 'joint_' + joint_element.attrib['name']
            # print("joint found in SDF:" + joint_element.attrib['name'])
            attrib_to_text(joint_element.find('parent'), 'link')
            attrib_to_text(joint_element.find('child'), 'link')
            all_attrib_to_element(joint_element.find('axis'))
            all_attrib_to_element(joint_element.find('limit'))
            joint_child_map[joint_element.find('child').text] = joint_element.attrib['name'] #save child name
            origin_to_pose(joint_element.find('origin'), joint_element.find('parent').text) #fix joint pose to be relative to parent
            
            
                # axis.append(limit)
                # xyz = axis.find('xyz')
                # if xyz is not None:
                #     xyz.attrib['expressed_in'] = '__model__' #Not sure if needed? allows for xyz axis in respect to model frame
    #Pulls inertia pose out to be link pose
    # for tag in ['link', 'joint']:
    #     for entity in sdf_tree.iter(tag):
    #         inertial = entity.find('inertial')
    #         if inertial is not None:
    #             origin = inertial.find('origin')
    #             if origin is not None:
    #                 print("origin found in SDF:" + origin.attrib['xyz'])
    #                 origin.tag = 'pose'
    #                 # origin.text = origin.attrib['xyz'] + ' ' + origin.attrib['rpy']
    #                 origin.text = '0 0 0 0 0 0' #for custom position inputs to each comp
    #                 origin.attrib.clear()
    #                 entity.insert(0, origin)
    #             inertial.remove(origin)
    
    # for origin in sdf_tree.iter('origin'):
    #     origin.tag = 'pose'
    #     origin.text = origin.attrib['xyz'] + ' ' + origin.attrib['rpy']
    #     origin.attrib.clear()
    # print("joint child mapping:")
    # print(joint_child_map)
    for visual_element in sdf_tree.iter('visual'):
        # print("visual found in SDF:" + visual_element.attrib['name'])
        origin_element = visual_element.find('origin')
        # print(origin_element.tag)
        origin_to_pose(origin_element, joint_child_map.get(visual_element.attrib['name'], None))
    
    for collision_element in sdf_tree.iter('collision'):
        origin_element = collision_element.find('origin')
        origin_to_pose(origin_element, joint_child_map.get(collision_element.attrib['name'], None))

    for inertial_element in sdf_tree.iter('inertial'):
        origin_element = inertial_element.find('origin')
        origin_to_pose(origin_element)
        
    #fixes relational pose references (passes visual pose to overall link pose)
    for link_element in sdf_tree.iter('link'):
        visual_element = link_element.find('visual')
        if visual_element is not None:
            visual_pose_element = visual_element.find('pose')
            if visual_pose_element is not None:
                link_element.insert(0, visual_pose_element)
                
    #change origin value of links?
    
    for mesh in sdf_tree.iter('mesh'):
        # print("mesh found in SDF:" + mesh.attrib['filename'])
        attrib_to_element(mesh, 'filename', 'uri')


    #Optional modificiations (1 or the other)
    delete_material(sdf_tree)
    # insert_material_snippet(sdf_tree)

    #add revolute joint limit
    add_revolute_joint_limit(sdf_tree)

    #fixing path
    for mesh_element in sdf_tree.iter('mesh'):
        for uri_element in mesh_element.iter('uri'):
            uri_element.text = uri_element.text.replace('package://', 'model://') #not sure if every case is covered

    #printing/writing
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
    parser.add_argument('-w', '--world', help='World name', dest='world_name', required=False, type=str, default='world1')

    args = parser.parse_args()
    convert_urdf_to_sdf(args.urdf_path, args.sdf_path, args.world_name)
    print("winning?")

