from pydrake.all import (
    DiagramBuilder,
    StartMeshcat,
    MeshcatVisualizer,
    Simulator,
    RigidTransform,
    MultibodyPlant,
    ContactModel,
    RobotDiagramBuilder,
    Parser,
    WeldJoint,
    RollPitchYaw,
    MeshcatVisualizerParams,
    Role,
    LoadModelDirectives,
    ProcessModelDirectives,
)
import numpy as np
import os
import yaml

yaml_file = os.path.dirname(os.path.abspath(__file__)) + "/yamls/3DOFFLIPPER.dmd.yaml"

meshcat = StartMeshcat()
builder = RobotDiagramBuilder()
plant = builder.plant()
scene_graph = builder.scene_graph()
parser = builder.parser()
parser.SetAutoRenaming(True)

parser.package_map().Add("iris_environments", os.path.dirname(os.path.abspath(__file__)) +"/iris_environments/assets")
# parser.AddModels(yaml_file)
directives = LoadModelDirectives(yaml_file)
ProcessModelDirectives(directives, plant, parser)


plant.Finalize()
inspector = scene_graph.model_inspector()
meshcat_params = MeshcatVisualizerParams()
meshcat_params.role = Role.kIllustration
visualizer = MeshcatVisualizer.AddToBuilder(
        builder.builder(), scene_graph, meshcat, meshcat_params)
diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()
plant_context = plant.GetMyMutableContextFromRoot(diagram_context)
diagram.ForcedPublish(diagram_context)

simulator = Simulator(diagram)
simulator.AdvanceTo(0.001)