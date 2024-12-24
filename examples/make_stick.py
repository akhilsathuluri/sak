# %%

# CONVERT ANY ROBOT INTO A CORRESPONDING STICK FIGURE FOR CO-DESIGN
# 1. Removes all existing meshes and collisions and replaces with existing geometries such as box or cylinder

# %%

import os
import glob

# import meshio
# from mpl_toolkits.mplot3d import Axes3D
# import matplotlib.pyplot as plt
# import open3d as o3d
# from odio_urdf import *
import xml.etree.ElementTree as ET
from urchin import *
import trimesh
from tqdm import tqdm
import numpy as np
from copy import copy as pycopy

# %%

# Load meshes
mesh_directory = "../biped-descriptions/biped-full/meshes/"
mesh_files = glob.glob(os.path.join(mesh_directory, "*.stl"))
# load urdf
urdf_file = "../biped-descriptions/biped-full/urdf/flamingo_lazy.urdf"
tree = ET.parse(urdf_file)
root = tree.getroot()
# odio_dsl = xml_to_odio(root)
# odio_robot = eval(odio_dsl)

# %%

# mesh_file = mesh_files[0]
# mesh = o3d.io.read_triangle_mesh(mesh_file)
# mesh = mesh.compute_vertex_normals()
# o3d.visualization.draw_geometries(
#     [mesh], window_name="STL", left=1000, top=200, width=800, height=650
# )

# %%

# Read the URDF using ElementTree and extract the root
tree = ET.parse(urdf_file)
root = tree.getroot()
robot = URDF.load(urdf_file, lazy_load_meshes=True)


# convert all meshes to primitives
prev_transform = np.eye(4)
for ii in tqdm(range(len(robot.links))):
    # for ii in tqdm(range(6)):
    # modify the joints
    mesh = trimesh.load(robot.links[ii].visuals[0].geometry.mesh.filename)
    bbox = mesh.bounding_primitive
    # modify the origin
    temp_transform = np.eye(4)
    temp_transform[0:3, 0:3] = pycopy(bbox.primitive.transform[0:3, 0:3])
    # replace the mesh
    if isinstance(bbox, trimesh.primitives.Box):
        robot.links[ii].visuals[0].geometry = Geometry(
            box=Box(
                size=bbox.primitive.extents / 1000.0,
            )
        )
        robot.links[ii].collisions[0].geometry = Geometry(
            box=Box(
                size=bbox.primitive.extents / 1000.0,
            )
        )

    elif isinstance(bbox, trimesh.primitives.Cylinder):
        robot.links[ii].visuals[0].geometry = Geometry(
            cylinder=Cylinder(
                radius=bbox.primitive.radius / 1000.0,
                length=bbox.primitive.height / 1000.0,
            )
        )
        robot.links[ii].collisions[0].geometry = Geometry(
            cylinder=Cylinder(
                radius=bbox.primitive.radius / 1000.0,
                length=bbox.primitive.height / 1000.0,
            )
        )

    # try:
    if ii != len(robot.joints) and robot.joints[ii].joint_type == "fixed":
        temp_transform[:3, 3] = (
            bbox.primitive.transform[:3, 3] / 1000.0 - prev_transform[:3, 3]
        )
    if ii == len(robot.joints) - 2:
        temp_transform[:3, 3] = (
            bbox.primitive.transform[:3, 3] / 1000.0 - prev_transform[:3, 3]
        )
    # except:
    #     pass
    robot.links[ii].visuals[0].origin = pycopy(temp_transform)
    robot.links[ii].collisions[0].origin = pycopy(temp_transform)
    robot.links[ii].inertial.origin = pycopy(temp_transform)

    temp_transform[:3, 3] = pycopy(bbox.primitive.transform[:3, 3] / 1000.0)
    prev_transform = pycopy(temp_transform)

robot.save(mesh_directory + "flamingo_stick.urdf")
# %%

# get link and joint names
joint_names = []
for joint in robot.joints:
    if len(joint.name.split("fixed_")) == 1:
        joint_names.append(joint.name)

link_names = []
for link in robot.links:
    link_names.append(link.name)

# %%

# now we need to make the robot drake compatible
