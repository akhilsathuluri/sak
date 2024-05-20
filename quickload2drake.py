# %%

import numpy as np
import matplotlib.pyplot as plt
from urdf2drake import URDFutils
from pathlib import Path

from pydrake.all import *

# import casadi as cs

# %%

# meshcat = StartMeshcat()

# # %%
# meshcat.Delete()
# meshcat.DeleteAddedControls()

# # %%
# package_path = "../robot_descriptions/"
# package_name = "v1-biped-description/"
# urdf_name = "robot-full-ass-urdf.urdf"

# urdf_utils = URDFutils(package_path, package_name, urdf_name)
# urdf_str, temp_urdf = urdf_utils.get_modified_urdf()


# %%
# -----------------------
# DIRECT JOINT TELEOP
# -----------------------
# Slider teleop for the robot
def robot_joint_teleop(meshcat, package_path, package_name, temp_urdf, init_pose):
    builder = DiagramBuilder()
    time_step = 0
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    parser = Parser(plant, scene_graph)
    abs_path = Path(package_path).resolve().__str__()
    parser.package_map().Add(package_name.split("/")[0], abs_path + "/" + package_name)
    model = parser.AddModels(temp_urdf.name)[0]

    body_indices = plant.GetBodyIndices(model)
    for bi in body_indices:
        AddFrameTriadIllustration(scene_graph=scene_graph, body=plant.get_body(bi))

    plant.Finalize()
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    # reset meshcat for each run
    meshcat.Delete()
    meshcat.DeleteAddedControls()

    sliders = builder.AddSystem(JointSliders(meshcat, plant, initial_value=init_pose))
    diagram = builder.Build()

    sliders.Run(diagram)
    meshcat.DeleteAddedControls()


# %%


def compute_walking_pose(
    meshcat, package_path, package_name, temp_urdf, foot_names, base_link_name
):
    builder = DiagramBuilder()
    time_step = 0
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    parser = Parser(plant, scene_graph)
    abs_path = Path(package_path).resolve().__str__()
    parser.package_map().Add(package_name.split("/")[0], abs_path + "/" + package_name)
    model = parser.AddModels(temp_urdf.name)[0]

    plant.Finalize()
    if meshcat is not None:
        MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
        meshcat.Delete()
        meshcat.DeleteAddedControls()
    body_indices = plant.GetBodyIndices(model)
    for bi in body_indices:
        AddFrameTriadIllustration(scene_graph=scene_graph, body=plant.get_body(bi))
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(diagram_context)

    foot_l_frame = plant.GetBodyByName(foot_names[0]).body_frame()
    foot_r_frame = plant.GetBodyByName(foot_names[1]).body_frame()
    base_link_frame = plant.GetBodyByName(base_link_name).body_frame()
    foot_orientation = RotationMatrix.MakeYRotation(-np.pi / 2).multiply(
        RotationMatrix.MakeZRotation(np.pi)
    )

    ik = InverseKinematics(plant, plant_context)
    ik.AddPositionConstraint(
        foot_l_frame,
        [0, 0, 0],
        plant.world_frame(),
        np.array([0, 0.1, 0.07]),
        np.array([0, 0.1, 0.07]),
    )
    ik.AddOrientationConstraint(
        foot_l_frame,
        RotationMatrix(),
        plant.world_frame(),
        foot_orientation.inverse(),
        1e-2,
    )

    ik.AddPositionConstraint(
        foot_r_frame,
        [0, 0, 0],
        plant.world_frame(),
        np.array([0, -0.1, 0.07]),
        np.array([0, -0.1, 0.07]),
    )
    ik.AddOrientationConstraint(
        foot_r_frame,
        RotationMatrix(),
        plant.world_frame(),
        foot_orientation.inverse(),
        1e-2,
    )
    ik.AddPositionConstraint(
        base_link_frame,
        [0, 0, 0],
        plant.world_frame(),
        np.array([-0.15, -np.inf, 0.5]),
        np.array([-0.15, np.inf, 0.5]),
    )
    ik.AddOrientationConstraint(
        base_link_frame,
        RotationMatrix(),
        plant.world_frame(),
        RotationMatrix.MakeYRotation(np.pi / 2),
        0.0,
    )

    prog = ik.get_mutable_prog()
    q = ik.q()
    q0 = plant.GetPositions(plant_context)
    prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
    prog.SetInitialGuess(q, q0)
    result = Solve(ik.prog())

    # result = Solve(prog)
    if result.is_success():
        optimized_q = result.GetSolution(q)
        print("Optimized joint angles:", optimized_q)
        plant.SetPositions(plant_context, optimized_q)
        diagram.ForcedPublish(diagram_context)
    else:
        print("Optimization failed.")
        print(result.GetInfeasibleConstraintNames(prog))
    # return result
    return result.GetSolution()


# %%
