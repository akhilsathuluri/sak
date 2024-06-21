# %%
import xml.etree.ElementTree as ET
import logging

import pydrake.multibody.plant as pmp
import pydrake.systems.primitives as psp
from pydrake.geometry import Box, MeshcatVisualizer, Sphere
from pydrake.math import RigidTransform
from pydrake.multibody.meshcat import ContactVisualizer, ContactVisualizerParams
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import ContactModel
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, EventStatus
from pydrake.systems.primitives import Demultiplexer, ZeroOrderHold
from pydrake.systems.drawing import plot_graphviz, plot_system_graphviz

# Additional imports
from include.addon_systems import *
from sak import *

# %%
# TODO: Figure out how to coordinate the logging accross modules
logger = logging.getLogger(__name__)


def run_jumping_sim(urdf_sim_path, mesh_path, meshcat=None, time_step=1e-4):
    # ------------------------------------------------------------------------------------
    # load urdf
    builder = DiagramBuilder()
    logging.info("Updating simulation at: {} Hz".format(1 / time_step))
    plant, scene_graph = pmp.AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    parser = Parser(plant, scene_graph)
    parser.package_map().Add("blackrobot", mesh_path)
    robot_model_sim = parser.AddModels(urdf_path_sim)[0]
    # ------------------------------------------------------------------------------------
    # set contact model
    plant.set_contact_model(ContactModel.kHydroelasticsOnly)
    # ------------------------------------------------------------------------------------
    # get essentials
    plant.Finalize()
    nq = plant.num_positions()  # 30
    nv = plant.num_velocities()  # 29
    na = plant.num_actuators()  # 23
    # ------------------------------------------------------------------------------------
    # configure visualisation
    if meshcat is not None:
        MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
        ContactVisualizer.AddToBuilder(
            builder,
            plant,
            meshcat,
            ContactVisualizerParams(newtons_per_meter=1e3, newton_meters_per_meter=1e1),
        )
    # ------------------------------------------------------------------------------------
    # add joint actuation model
    joint_actuation = JointActuation(
        plant,
        gear_ratio_matrix,  # gear ratios
        motor_inertia_matrix,  # motor inertia
        tau_sat,  # torque saturation
        f_K0,  # f_K0
        f_K1,  # f_K1
        f_K2,  # f_K2
        np.zeros((na, na)),  # K_bemf
        delay=1e-3,  # delay -- 1 ms
    )

    # ------------------------------------------------------------------------------------
    if meshcat is not None:
        taurlog = psp.LogVectorOutput(
            joint_actuation.GetOutputPort("realised_torque"), builder
        )
        qlog = psp.LogVectorOutput(
            plant.get_state_output_port(robot_model_sim), builder
        )
    tau_flog = psp.LogVectorOutput(
        joint_actuation.GetOutputPort("friction_torque"), builder
    )
    tau_mlog = psp.LogVectorOutput(
        joint_actuation.GetOutputPort("motor_torque"), builder
    )
    mv_log = psp.LogVectorOutput(
        joint_actuation.GetOutputPort("motor_velocity"), builder
    )
    # ------------------------------------------------------------------------------------
    # build the diagram
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
    plant.SetPositions(
        plant_context, np.concatenate((base_ori_init_wxyz, base_pos_init, s_init_sim))
    )
    if meshcat is not None:
        meshcat.Delete()
        meshcat.DeleteAddedControls()
    simulator = Simulator(diagram, diagram_context)

    def monitor(context):
        return EventStatus.ReachedTermination(diagram, "Controller failed")

    simulator.set_monitor(monitor)

    try:  # there is still a possibility that the simulator will fail due to the solver
        if meshcat is not None:
            meshcat.StartRecording()
        simulator.AdvanceTo(Tf)
        if meshcat is not None:
            meshcat.StopRecording()
    except RuntimeError as e:
        logging.error("Simulation failed with RuntimeError:\n {}".format(e))
    sim_context = simulator.get_context()
    Tf = sim_context.get_time()

    return Tf
