# %%

import numpy as np
import matplotlib.pyplot as plt
from urdf2drake import URDFutils
from pathlib import Path

from pydrake.all import *

# import casadi as cs

# %%

meshcat = StartMeshcat()

# %%
meshcat.Delete()
meshcat.DeleteAddedControls()

# %%
package_path = "../robot_descriptions/"
package_name = "v1-biped-description/"
urdf_name = "robot-full-ass-urdf.urdf"

urdf_utils = URDFutils(package_path, package_name, urdf_name)
urdf_str, temp_urdf = urdf_utils.get_modified_urdf()

# %%
# -----------------------
# DIRECT JOINT TELEOP
# -----------------------
# Slider teleop for the robot
def robot_joint_teleop(plant, model, meshcat):
    builder = DiagramBuilder()
    time_step = 1e-4
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    parser = Parser(plant, scene_graph)
    abs_path = Path(package_path).resolve().__str__()
    parser.package_map().Add(package_name.split('/')[0], abs_path+ '/' + package_name)
    model = parser.AddModels(temp_urdf.name)[0]

    body_indices = plant.GetBodyIndices(model)
    for bi in body_indices:
        AddFrameTriadIllustration(scene_graph=scene_graph, body=plant.get_body(bi))

    # X_R = RigidTransform(RotationMatrix.MakeYRotation(np.pi/2), np.array([0, 0, 0]))
    # plant.WeldFrames(plant.world_frame(), plant.get_body(plant.GetBodyIndices(model)[0]).body_frame(), X_R)
    plant.Finalize()
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    # reset meshcat for each run
    meshcat.Delete()
    meshcat.DeleteAddedControls()
    
    sliders = builder.AddSystem(JointSliders(meshcat, plant))
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)    
        
    sliders.Run(diagram)
    meshcat.DeleteAddedControls()

robot_joint_teleop(plant, model, meshcat)

# %%
plant.num_actuators()

# %%

# Identifying the correct walking pose
# constraint that both the feet are on the ground
# solve for the joint angles that satisfy this constraint
def compute_walking_pose(meshcat):
    builder = DiagramBuilder()
    time_step = 0
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    parser = Parser(plant, scene_graph)
    abs_path = Path(package_path).resolve().__str__()
    parser.package_map().Add(package_name.split('/')[0], abs_path+ '/' + package_name)
    model = parser.AddModels(temp_urdf.name)[0]

    # Add feet contact visual and collision model
    xMinMax = [-0.11, 0.04]
    yMinMax = [-0.04,0.04]

    plant.RegisterVisualGeometry(
            plant.GetBodyByName('foot_l'),
            RigidTransform(np.array([0.07,0,0])),
            Box(1e-3,yMinMax[1]-yMinMax[0], xMinMax[1]-xMinMax[0]),
            "l_sole_collision",
            np.array([1.0,1.0,1.0,1]))
    plant.RegisterVisualGeometry(
            plant.GetBodyByName('foot_r'),
            RigidTransform(np.array([0.07,0,0])),
            Box(1e-3,yMinMax[1]-yMinMax[0], xMinMax[1]-xMinMax[0]),
            "r_sole_collision",
            np.array([1.0,1.0,1.0,1]))

    # add the feet hydroelastic model boxes
    proximity_properties = ProximityProperties()
    AddContactMaterial(
        1e4, 1e7, CoulombFriction(
            static_friction = 1.0,
            dynamic_friction = 1.0), proximity_properties
    )
    AddCompliantHydroelasticProperties(0.01, 1e8, proximity_properties)

    plant.RegisterCollisionGeometry(
            plant.GetBodyByName('foot_l'),
            RigidTransform(np.array([0.07,0,0])),
            Box(1e-3,yMinMax[1]-yMinMax[0], xMinMax[1]-xMinMax[0]),
            "l_sole_collision",
            proximity_properties)
    plant.RegisterCollisionGeometry(
            plant.GetBodyByName('foot_r'),
            RigidTransform(np.array([0.07,0,0])),
            Box(1e-3,yMinMax[1]-yMinMax[0], xMinMax[1]-xMinMax[0]),
            "l_sole_collision",
            proximity_properties)


    plant.Finalize()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    meshcat.Delete()
    meshcat.DeleteAddedControls()
    body_indices = plant.GetBodyIndices(model)
    for bi in body_indices:
        AddFrameTriadIllustration(scene_graph=scene_graph, body=plant.get_body(bi))
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(diagram_context)

    foot_l_frame = plant.GetBodyByName("foot_l").body_frame()
    foot_r_frame = plant.GetBodyByName("foot_r").body_frame()
    base_link_frame = plant.GetBodyByName("base_link").body_frame()
    foot_orientation = RotationMatrix.MakeYRotation(-np.pi/2).multiply(RotationMatrix.MakeZRotation(np.pi))
    
    ik = InverseKinematics(plant, plant_context)
    ik.AddPositionConstraint(
                foot_l_frame,
                [0, 0, 0],
                plant.world_frame(),
                np.array([0,0.1,0.07]),
                np.array([0,0.1,0.07]),
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
                np.array([0,-0.1,0.07]),
                np.array([0,-0.1,0.07]),
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
                np.array([-0.15,-np.inf,0.5]),
                np.array([-0.15,np.inf,0.5]),
            )
    ik.AddOrientationConstraint(
                base_link_frame,
                RotationMatrix(),
                plant.world_frame(),
                RotationMatrix.MakeYRotation(np.pi/2),
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
        return result
    else:
        print("Optimization failed.")
        # return None
        print(result.GetInfeasibleConstraintNames(prog))
        return result

# Call the function with the 'plant' object
result = compute_walking_pose(meshcat)
# 
# computed walking pose:
# array([ 7.07132374e-01, -3.52998388e-06,  7.07081187e-01, -2.60464555e-05,
    #    -1.50000000e-01,  1.65057030e-01,  5.00000000e-01, -1.31351885e-04,
    #     5.23850595e-01, -1.56995529e+00,  1.04103461e+00, -7.23781492e-03,
    #     5.25645822e-01, -1.58187524e+00,  1.04416407e+00])

# %%
