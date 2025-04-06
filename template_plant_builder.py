"""
Template file that shows how to build a generic MultibodyPlant containing one of
the 8 test scenes.
"""

from pydrake.all import (
    StartMeshcat,
    AddDefaultVisualization,
    Simulator,
    RobotDiagramBuilder,
    SceneGraphCollisionChecker,
    RandomGenerator,
)

import os
import numpy as np

# TEST_SCENE = "2DOFFLIPPER"
# TEST_SCENE = "3DOFFLIPPER"
# TEST_SCENE = "5DOFUR3"
# TEST_SCENE = "6DOFUR3"
TEST_SCENE = "7DOFIIWA"
# TEST_SCENE = "7DOFBINS"
# TEST_SCENE = "7DOF4SHELVES"
# TEST_SCENE = "14DOFIIWAS"
# TEST_SCENE = "15DOFALLEGRO"

rng = RandomGenerator(1234)
np.random.seed(1234)

# MODIFY PATH AS NEEDED
parent_directory = os.path.dirname(os.path.abspath(__file__))
data_directory = os.path.join(parent_directory)
scene_yaml_file = os.path.join(data_directory, "yamls", TEST_SCENE + ".dmd.yaml")

meshcat = StartMeshcat()

robot_diagram_builder = RobotDiagramBuilder()
parser = robot_diagram_builder.parser()
iris_environement_assets = os.path.join(data_directory, "iris_environments", "assets")
parser.package_map().Add("iris_environments", iris_environement_assets)
robot_model_instances = parser.AddModels(scene_yaml_file)
plant = robot_diagram_builder.plant()
plant.Finalize()
AddDefaultVisualization(robot_diagram_builder.builder(), meshcat=meshcat)
diagram = robot_diagram_builder.Build()

# Roll forward sim a bit to show the visualization
simulator = Simulator(diagram)
simulator.AdvanceTo(0.001)

plant_context = plant.CreateDefaultContext()

num_robot_positions = plant.num_positions()

collision_checker_params = {}
collision_checker_params["robot_model_instances"] = robot_model_instances
collision_checker_params["model"] = diagram
collision_checker_params["edge_step_size"] = 0.125
collision_checker = SceneGraphCollisionChecker(**collision_checker_params)
