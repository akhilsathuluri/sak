# %%
import meshio
from tqdm import tqdm
import xml.etree.ElementTree as ET
from pathlib import Path
import tempfile

# install the updated odio_urdf from pip install git+https://github.com/akhilsathuluri/odio_urdf.git@comodo
from odio_urdf import *

# %%
# inspired from how urdf generation is dealt with in comodo
# take in urdf package details and create a temp urdf every time 

class URDFutils:
    def __init__(self, package_path, package_name, urdf_name):
        self.urdf_name = urdf_name
        self.urdf_path = package_path + package_name + "urdf/" + urdf_name
        # self.mesh_path = package_path 
        self.package_path = package_path
        
    def urdf2drake(self, in_mesh_format="stl", out_mesh_format="obj"):
        # load the urdf
        tree = ET.parse(self.urdf_path)
        self.root = tree.getroot()
        print("Converting all the visual meshes from .stl to .obj")
        for child in tqdm(self.root.findall("./link/visual/geometry/mesh")):
            path = child.attrib["filename"]
            child_mesh_path = self.package_path + path.split("package://")[1]
            child_mesh_name = child_mesh_path.replace(in_mesh_format, out_mesh_format)
            if not Path(child_mesh_name).is_file():
                temp_mesh = meshio.read(child_mesh_path)
                temp_mesh.write(child_mesh_name)
            child.set("filename", path.replace(in_mesh_format, out_mesh_format))

        print("Converting all the visual meshes from .stl to .obj")
        for child in tqdm(self.root.findall("./link/collision/geometry/mesh")):
            path = child.attrib["filename"]
            child_mesh_path = self.package_path + path.split("package://")[1]
            child_mesh_name = child_mesh_path.replace(in_mesh_format, out_mesh_format)
            if not Path(child_mesh_name).is_file():
                temp_mesh = meshio.read(child_mesh_path)
                temp_mesh.write(child_mesh_name)
            child.set("filename", path.replace(in_mesh_format, out_mesh_format))

    def remove_collisions_except(self, link_list):
        # list = ['l_foot', 'r_foot']
        for link in self.root.findall("./link"):
            if link.attrib["name"] not in link_list:
                for col in link.findall("./collision"):
                    link.remove(col)

    def fix_joints_except(self, joint_list):
        joints = self.root.findall("./joint")
        for j in joints:
            if j.attrib["name"] not in joint_list:
                j.attrib["type"] = "fixed"
        
    def add_drake_tags(self):
        odio_robot = xml_to_odio(self.root)
        self.robot_odio = eval(odio_robot)
        # Extract the non-fixed DoF as a list
        joints = self.root.findall("./joint")
        joint_names = []
        for j in joints:
            if j.attrib["type"] != "fixed":
                joint_names.append([j.attrib["name"], j.attrib["type"]])

        for j in joint_names:
            actuator_name = "actuator_" + j[0]
            transmission_name = "transmission" + j[0]
            temp_trans = Transmission(
                Type("SimpleTransmission"),
                Actuator(Mechanicalreduction("1"), name=actuator_name),
                Transjoint(Hardwareinterface("EffortJointInterface"), j[0]),
                name=transmission_name,
            )
            self.robot_odio(temp_trans)
    
    def modify_and_write_urdf(self, out_path, out_urdf_name):
        self.urdf2drake(in_mesh_format="STL")
        self.remove_collisions_except([])
        self.add_drake_tags()
        
        file_name = out_path + out_urdf_name
        with open(file_name, "w") as f:
            print(self.robot_odio, file=f)
        return file_name

    def get_modified_urdf(self):
        self.urdf2drake(in_mesh_format="STL")
        self.remove_collisions_except([])
        self.add_drake_tags()
        urdf_string = self.robot_odio
        
        path_temp_urdf = tempfile.NamedTemporaryFile(mode="w+", suffix=".urdf")
        with open(path_temp_urdf.name, "w") as f:
            print(self.robot_odio, file=f)
        
        return urdf_string, path_temp_urdf

    def add_flat_feet(self):
        pass

    def add_foot_point_contacts(self):
        pass

# %%
# Example usage
# package_path = "../robot_descriptions/"
# package_name = "v1-biped-description/"
# urdf_name = "robot-full-ass-urdf.urdf"

# urdf_utils = URDFutils(package_path, package_name, urdf_name)
# urdf_utils.modify_and_write_urdf("../", "out.urdf")
# urdf_str, temp_urdf = urdf_utils.get_modified_urdf()