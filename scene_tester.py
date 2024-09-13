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
import time

# Set manually
# TEST_SCENE = "3DOFFLIPPER"
# TEST_SCENE = "5DOFUR3"
# TEST_SCENE = "6DOFUR3"
# TEST_SCENE = "7DOFIIWA"
# TEST_SCENE = "7DOFBINS"
# TEST_SCENE = "7DOF4SHELVES"
# TEST_SCENE = "14DOFIIWAS"
TEST_SCENE = "15DOFALLEGRO"


yaml_file = os.path.dirname(os.path.abspath(__file__)) + "/yamls/" + TEST_SCENE + ".dmd.yaml"

print("Illustration:")
meshcat = StartMeshcat()
print("Proximity:")
meshcat2 = StartMeshcat()

builder = RobotDiagramBuilder()
plant = builder.plant()
scene_graph = builder.scene_graph()
parser = builder.parser()
parser.SetAutoRenaming(True)

parser.package_map().Add("iris_environments", os.path.dirname(os.path.abspath(__file__)) +"/iris_environments/assets")
directives = LoadModelDirectives(yaml_file)
ProcessModelDirectives(directives, plant, parser)


plant.Finalize()
inspector = scene_graph.model_inspector()

# Create the MeshcatVisualizerParams object for illustration
meshcat_params_illustration = MeshcatVisualizerParams()
meshcat_params_illustration.role = Role.kIllustration

# Create the MeshcatVisualizerParams object for proximity
meshcat_params_proximity = MeshcatVisualizerParams()
meshcat_params_proximity.role = Role.kProximity

# Add the MeshcatVisualizer to the builder for illustration role
MeshcatVisualizer.AddToBuilder(
    builder.builder(), scene_graph, meshcat, meshcat_params_illustration)

# Add the MeshcatVisualizer to the builder for proximity role
MeshcatVisualizer.AddToBuilder(
    builder.builder(), scene_graph, meshcat2, meshcat_params_proximity)

diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()
plant_context = plant.GetMyMutableContextFromRoot(diagram_context)
diagram.ForcedPublish(diagram_context)

simulator = Simulator(diagram)
simulator.set_publish_every_time_step(True)
meshcat.StartRecording()
simulator.AdvanceTo(5)
meshcat.PublishRecording()
time.sleep(5)
print(f"View the scene simulation at: {meshcat.web_url()}/download")