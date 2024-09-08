from .ur3e_demo import UrDiagram
from functools import partial
from pydrake.all import (StartMeshcat,
                         RobotDiagramBuilder,
                         MeshcatVisualizer,
                         LoadModelDirectives,
                         ProcessModelDirectives,
                         RigidTransform,
                         RotationMatrix,
                         MeshcatVisualizerParams,
                         Role,
                         RollPitchYaw,
                         Meldis,
                         AddDefaultVisualization,
                         Box
                         )
import numpy as np
import os

env_names = ['2DOFFLIPPER', 
             '3DOFFLIPPER', 
             '5DOFUR3', 
             '6DOFUR3',
             #'MYCOBOT', 
             '7DOFIIWA', 
             '7DOF4SHELVES', 
             '7DOFBINS', 
             '14DOFIIWAS',
             '15DOFALLEGRO']


def plant_builder_2dof_blocks(usemeshcat = False, size = 1.4, pos =1.0, radius = 0.01):
    if usemeshcat:
        #meshcat = StartMeshcat()
        meldis = Meldis()
        meshcat = meldis.meshcat
    builder = RobotDiagramBuilder()
    plant = builder.plant()
    scene_graph = builder.scene_graph()
    parser = builder.parser()
    # plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    # parser = Parser(plant)
    path_repo = os.path.dirname(os.path.abspath(__file__))
    
    urdf = f'''
<robot name="ball_in_boxes">
  <link name="fixed">
    <visual name="top_left_visual">
      <origin rpy="0 0 0" xyz="{-pos:.2f} {pos:.2f} 0"/>
      <geometry><box size="{size:.2f} {size:.2f} {size:.2f}"/></geometry>
      <material name="blue">
        <color rgba="1 0 0 0.9"/>
      </material>
    </visual>
    <visual name="top_right_visual">
      <origin rpy="0 0 0" xyz="{pos:.2f} {pos:.2f} 0"/>
      <geometry><box size="{size:.2f} {size:.2f} {size:.2f}"/></geometry>
      <material name="green">
        <color rgba="1 0 0 0.9"/>
      </material>
    </visual>
    <visual name="bottom_left_visual">
      <origin rpy="0 0 0" xyz="{-pos:.2f} {-pos:.2f} 0"/>
      <geometry><box size="{size:.2f} {size:.2f} {size:.2f}"/></geometry>
      <material name="red">
        <color rgba="1 0 0 0.9"/>
      </material>
    </visual>
    <visual name="bottom_right_visual">
      <origin rpy="0 0 0" xyz="{pos:.2f} {-pos:.2f} 0"/>
      <geometry><box size="{size:.2f} {size:.2f} {size:.2f}"/></geometry>
      <material name="yellow">
        <color rgba="1 0 0 0.9"/>
      </material>
    </visual>
    <collision name="top_left">
      <origin rpy="0 0 0" xyz="{-pos:.2f} {pos:.2f} 0"/>
      <geometry><box size="{size:.2f} {size:.2f} {size:.2f}"/></geometry>
    </collision>
    <collision name="top_right">
      <origin rpy="0 0 0" xyz="{pos:.2f} {pos:.2f} 0"/>
      <geometry><box size="{size:.2f} {size:.2f} {size:.2f}"/></geometry>
    </collision>
    <collision name="bottom_left">
      <origin rpy="0 0 0" xyz="{-pos:.2f} {-pos:.2f} 0"/>
      <geometry><box size="{size:.2f} {size:.2f} {size:.2f}"/></geometry>
    </collision>
    <collision name="bottom_right">
      <origin rpy="0 0 0" xyz="{pos:.2f} {-pos:.2f} 0"/>
      <geometry><box size="{size:.2f} {size:.2f} {size:.2f}"/></geometry>
    </collision>
  </link>
  <joint name="fixed_link_weld" type="fixed">
    <parent link="world"/>
    <child link="fixed"/>
  </joint>
  <link name="movable">
    <collision name="sphere">
      <geometry><sphere radius="{radius:.2f}"/></geometry>
    </collision>
  </link>
  <link name="for_joint"/>
  <joint name="x" type="prismatic">
    <axis xyz="1 0 0"/>
    <limit lower="-2" upper="2"/>
    <parent link="world"/>
    <child link="for_joint"/>
  </joint>
  <joint name="y" type="prismatic">
    <axis xyz="0 1 0"/>
    <limit lower="-2" upper="2"/>
    <parent link="for_joint"/>
    <child link="movable"/>
  </joint>
</robot>
'''
    parser.AddModelsFromString(urdf, "urdf")
    plant.Finalize()
    inspector = scene_graph.model_inspector()
    if usemeshcat:
        meshcat_params = MeshcatVisualizerParams()
        meshcat_params.role = Role.kIllustration
        meshcat.Set2dRenderMode(RigidTransform(RotationMatrix.MakeXRotation(-np.pi/2)), xmin=-3, xmax=3, ymin=-3, ymax=3)
        visualizer = AddDefaultVisualization(builder.builder(), meshcat)
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(diagram_context)
    diagram.ForcedPublish(diagram_context)
    if usemeshcat:
        print(meshcat.web_url())
    #meshcat.SetProperty('/drake/proximity', "visible", True)
    return plant, scene_graph, diagram, diagram_context, plant_context, meshcat if usemeshcat else None


def plant_builder_5dof_ur3(usemeshcat = False, cfg = {'add_shelf': True, 'add_gripper': True}):
    ur = UrDiagram(num_ur = 1, weld_wrist = True, add_shelf = cfg['add_shelf'],
                    add_gripper = cfg['add_gripper'], use_meshcat=usemeshcat)

    if usemeshcat: meshcat = ur.meshcat
    plant = ur.plant
    diagram_context = ur.diagram.CreateDefaultContext()
    ur.diagram.ForcedPublish(diagram_context)
    diagram = ur.diagram
    plant_context = ur.plant.GetMyMutableContextFromRoot(
            diagram_context)
    # scene_graph_context = ur.scene_graph.GetMyMutableContextFromRoot(
    #     diagram_context)
    scene_graph = ur.scene_graph
    return plant, scene_graph, diagram, diagram_context, plant_context, meshcat if usemeshcat else None

def plant_builder_6dof_ur3(usemeshcat = False, cfg = {'add_shelf': True, 'add_gripper': True}):
    ur = UrDiagram(num_ur = 1, weld_wrist = False, add_shelf = cfg['add_shelf'],
                    add_gripper = cfg['add_gripper'], use_meshcat=usemeshcat)

    if usemeshcat: meshcat = ur.meshcat
    plant = ur.plant
    diagram_context = ur.diagram.CreateDefaultContext()
    ur.diagram.ForcedPublish(diagram_context)
    diagram = ur.diagram
    plant_context = ur.plant.GetMyMutableContextFromRoot(
            diagram_context)
    # scene_graph_context = ur.scene_graph.GetMyMutableContextFromRoot(
    #     diagram_context)
    scene_graph = ur.scene_graph
    return plant, scene_graph, diagram, diagram_context, plant_context, meshcat if usemeshcat else None

def plant_builder_2dof_flipper_obs(usemeshcat = False):
    if usemeshcat:
        #meshcat = StartMeshcat()
        meldis = Meldis()
        meshcat = meldis.meshcat
    builder = RobotDiagramBuilder()
    plant = builder.plant()
    scene_graph = builder.scene_graph()
    parser = builder.parser()
    # plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    # parser = Parser(plant)
    path_repo = os.path.dirname(os.path.abspath(__file__))
    #oneDOF_iiwa_asset = rel_path_cvisiris + "assets/oneDOF_iiwa7_with_box_collision.sdf"#FindResourceOrThrow("drake/C_Iris_Examples/assets/oneDOF_iiwa7_with_box_collision.sdf")
    twoDOF_iiwa_asset = path_repo + "/assets/twoDOF_iiwa7_with_skinny_box_coll.sdf"#FindResourceOrThrow("drake/C_Iris_Examples/assets/twoDOF_iiwa7_with_box_collision.sdf")
    #twoDOF_iiwa_asset = path_repo + "/assets/2dof_iiwa_translating_base.sdf"#FindResourceOrThrow("drake/C_Iris_Examples/assets/twoDOF_iiwa7_with_box_collision.sdf")

    box_asset = path_repo + "/assets/box_small.urdf" #FindResourceOrThrow("drake/C_Iris_Examples/assets/box_small.urdf")
    #box2_asset = path_repo + "/assets/box_long.urdf"
    obs_asset = path_repo + "/assets/2d_obs.urdf"
    path_repo = os.path.dirname(os.path.abspath(__file__)) 
    parser.package_map().Add("iris_environments", path_repo+"/assets")
    parser.SetAutoRenaming(True)
    models = []
    models.append(parser.AddModels(box_asset)[0])
    #models.append(parser.AddModels(box2_asset)[0])
    models.append(parser.AddModels(obs_asset)[0])
    models.append(parser.AddModels(twoDOF_iiwa_asset)[0])
    #models.append(parser.AddModels(roi_asset)[0])

    locs = [[0.,0.,0.],
            [0,0,0,],
            [0,0.6,0.19],
            [0.,0.3,0.2],
            ]
    plant.WeldFrames(plant.world_frame(), 
        plant.GetFrameByName("base", models[0]),
        RigidTransform(locs[0]))
    from pydrake.geometry import ProximityProperties, AddContactMaterial
    from pydrake.all import CoulombFriction
    proximity_properties = ProximityProperties()
    AddContactMaterial(dissipation=0.1,
                        point_stiffness=250.0,
                        friction=CoulombFriction(0.9, 0.5),
                        properties=proximity_properties)
    from pydrake.all import Sphere, Cylinder
    box_shape = Sphere(0.5)
    cyl_shape = Cylinder(0.5, 0.3)
    frame = plant.GetBodyByName("box1")
    #X1 = RigidTransform(np.array([0,1.1,0.1]))
    #X1 = RigidTransform(RollPitchYaw(0,0,0).ToRotationMatrix(),np.array([0,1.37,0.47]))
    X1 = RigidTransform(RollPitchYaw(0,np.pi/2,0).ToRotationMatrix(),np.array([0,1.37,0.47]))
    plant.RegisterVisualGeometry(frame,X1,
                                              cyl_shape, "obs1",
                                              np.array([0.2, 0.2, 0.2, 1]))
    obs1 = plant.RegisterCollisionGeometry(frame, X1,
                                              cyl_shape, "obs1",
                                               proximity_properties)
    # cyl_shape2 = Cylinder(0.3, 0.3)
    # X2 = RigidTransform(RollPitchYaw(0,np.pi/2,0).ToRotationMatrix(),np.array([0,1.35,0.15]))
    # plant.RegisterVisualGeometry(frame,X2,
    #                                           cyl_shape2, "obs2",
    #                                           np.array([0.2, 0.2, 0.2, 1]))
    # obs1 = plant.RegisterCollisionGeometry(frame, X2,
    #                                           cyl_shape2, "obs2",
    #                                            proximity_properties)
    # X2 = RigidTransform(np.array([0,0.75,-0.75]))
    
    # X3 = RigidTransform(RollPitchYaw(np.pi/2.5,0,0).ToRotationMatrix(), np.array([0,1.1,-0.78]))
    #obsbox
    plant.WeldFrames(plant.world_frame(), 
        plant.GetFrameByName("base", models[1]),
        RigidTransform(np.array([0,0,10])))
    
    plant.WeldFrames(plant.world_frame(), 
                    plant.GetFrameByName("iiwa_twoDOF_link_0", models[-1]), 
                    RigidTransform(RollPitchYaw([0,np.pi/4, -np.pi/2]).ToRotationMatrix(), locs[-1]))
  
    
    plant.Finalize()
    inspector = scene_graph.model_inspector()
    if usemeshcat:
        meshcat_params = MeshcatVisualizerParams()
        meshcat_params.role = Role.kIllustration
        visualizer = AddDefaultVisualization(builder.builder(), meshcat)
        
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(diagram_context)
    diagram.ForcedPublish(diagram_context)
    if usemeshcat:
        print(meshcat.web_url())
    return plant, scene_graph, diagram, diagram_context, plant_context, meshcat if usemeshcat else None


def plant_builder_2dof_flipper2_obs(usemeshcat = False):
    if usemeshcat:
        #meshcat = StartMeshcat()
        meldis = Meldis()
        meshcat = meldis.meshcat
    builder = RobotDiagramBuilder()
    plant = builder.plant()
    scene_graph = builder.scene_graph()
    parser = builder.parser()
    # plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    # parser = Parser(plant)
    path_repo = os.path.dirname(os.path.abspath(__file__))
    #oneDOF_iiwa_asset = rel_path_cvisiris + "assets/oneDOF_iiwa7_with_box_collision.sdf"#FindResourceOrThrow("drake/C_Iris_Examples/assets/oneDOF_iiwa7_with_box_collision.sdf")
    twoDOF_iiwa_asset = path_repo + "/assets/twoDOF_iiwa7_with_box_collision.sdf"#FindResourceOrThrow("drake/C_Iris_Examples/assets/twoDOF_iiwa7_with_box_collision.sdf")
    #twoDOF_iiwa_asset = path_repo + "/assets/2dof_iiwa_translating_base.sdf"#FindResourceOrThrow("drake/C_Iris_Examples/assets/twoDOF_iiwa7_with_box_collision.sdf")

    box_asset = path_repo + "/assets/box_small.urdf" #FindResourceOrThrow("drake/C_Iris_Examples/assets/box_small.urdf")
    #box2_asset = path_repo + "/assets/box_long.urdf"
    #obs_asset = path_repo + "/assets/2d_obs.urdf"
    path_repo = os.path.dirname(os.path.abspath(__file__)) 
    parser.package_map().Add("iris_environments", path_repo+"/assets")
    parser.SetAutoRenaming(True)
    models = []
    models.append(parser.AddModels(box_asset)[0])
    #models.append(parser.AddModels(box2_asset)[0])
    #models.append(parser.AddModels(obs_asset)[0])
    models.append(parser.AddModels(twoDOF_iiwa_asset)[0])
    #models.append(parser.AddModels(roi_asset)[0])
    
    locs = [[0.,20,0],
            [0.,0,0],
            ]
    plant.WeldFrames(plant.world_frame(), 
        plant.GetFrameByName("base", models[0]),
        RigidTransform(locs[0]))
    from pydrake.geometry import ProximityProperties, AddContactMaterial
    from pydrake.all import CoulombFriction
    proximity_properties = ProximityProperties()
    AddContactMaterial(dissipation=0.1,
                        point_stiffness=250.0,
                        friction=CoulombFriction(0.9, 0.5),
                        properties=proximity_properties)
    from pydrake.all import Sphere, Cylinder
    cyl_shape = Cylinder(0.7, 0.3)
    frame = plant.GetBodyByName("box1")
    #X1 = RigidTransform(np.array([0,1.1,0.1]))
    #X1 = RigidTransform(RollPitchYaw(0,0,0).ToRotationMatrix(),np.array([0,1.37,0.47]))
    X1 = RigidTransform(RollPitchYaw(0,np.pi/2,0).ToRotationMatrix(),np.array([0,1.4,0.0+20]))
    plant.RegisterVisualGeometry(frame,X1,
                                              cyl_shape, "obs1",
                                              np.array([0.2, 0.2, 0.2, 1]))
    obs1 = plant.RegisterCollisionGeometry(frame, X1,
                                              cyl_shape, "obs1",
                                               proximity_properties)
    # cyl_shape2 = Cylinder(0.3, 0.3)
    # X2 = RigidTransform(RollPitchYaw(0,np.pi/2,0).ToRotationMatrix(),np.array([0,1.35,0.15]))
    # plant.RegisterVisualGeometry(frame,X2,
    #                                           cyl_shape2, "obs2",
    #                                           np.array([0.2, 0.2, 0.2, 1]))
    # obs1 = plant.RegisterCollisionGeometry(frame, X2,
    #                                           cyl_shape2, "obs2",
    #                                            proximity_properties)
    # X2 = RigidTransform(np.array([0,0.75,-0.75]))
    
    # X3 = RigidTransform(RollPitchYaw(np.pi/2.5,0,0).ToRotationMatrix(), np.array([0,1.1,-0.78]))
    
    plant.WeldFrames(plant.world_frame(), 
                    plant.GetFrameByName("iiwa_twoDOF_link_0", models[-1]), 
                    RigidTransform(RollPitchYaw([0,0, -np.pi/2]).ToRotationMatrix(), locs[-1]))
  
    
    plant.Finalize()
    inspector = scene_graph.model_inspector()
    if usemeshcat:
        from pydrake.all import VisualizationConfig, ApplyVisualizationConfig
        config = VisualizationConfig()
        config.enable_alpha_sliders = True
        
        #visualizer = AddDefaultVisualization(builder.builder(), meshcat)
        ApplyVisualizationConfig(config, builder.builder(), meshcat=meshcat)
        
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(diagram_context)
    diagram.ForcedPublish(diagram_context)
    if usemeshcat:
        print(meshcat.web_url())
    return plant, scene_graph, diagram, diagram_context, plant_context, meshcat if usemeshcat else None

def plant_builder_3dof_flipper(usemeshcat = False):
    meshcat = StartMeshcat()
    builder = RobotDiagramBuilder()
    plant = builder.plant()
    scene_graph = builder.scene_graph()
    parser = builder.parser()
    parser.SetAutoRenaming(True)
    path_repo = os.path.dirname(os.path.abspath(__file__)) 
    parser.package_map().Add("iris_environments", path_repo+"/assets")
    # plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    # parser = Parser(plant)
    rel_path = path_repo
    oneDOF_iiwa_asset = rel_path + "/assets/oneDOF_iiwa7_with_box_collision.sdf"#FindResourceOrThrow("drake/C_Iris_Examples/assets/oneDOF_iiwa7_with_box_collision.sdf")
    twoDOF_iiwa_asset = rel_path + "/assets/twoDOF_iiwa7_with_box_collision.sdf"#FindResourceOrThrow("drake/C_Iris_Examples/assets/twoDOF_iiwa7_with_box_collision.sdf")

    box_asset = rel_path + "/assets/box_small.urdf" #FindResourceOrThrow("drake/C_Iris_Examples/assets/box_small.urdf")
    print(box_asset)
    models = []
    models.append(parser.AddModels(box_asset)[0])
    models.append(parser.AddModels(twoDOF_iiwa_asset)[0])
    models.append(parser.AddModels(oneDOF_iiwa_asset)[0])

    locs = [[0.,0.,0.],
            [0.,.55,0.],
            [0.,-.55,0.]]
    plant.WeldFrames(plant.world_frame(), 
        plant.GetFrameByName("base", models[0]),
        RigidTransform(locs[0]))
    plant.WeldFrames(plant.world_frame(), 
                    plant.GetFrameByName("iiwa_twoDOF_link_0", models[1]), 
                    RigidTransform(RollPitchYaw([0,0, -np.pi/2]).ToRotationMatrix(), locs[1]))
    plant.WeldFrames(plant.world_frame(), 
                    plant.GetFrameByName("iiwa_oneDOF_link_0", models[2]), 
                    RigidTransform(RollPitchYaw([0,0, -np.pi/2]).ToRotationMatrix(), locs[2]))

    plant.Finalize()
    inspector = scene_graph.model_inspector()

    meshcat_params = MeshcatVisualizerParams()
    meshcat_params.role = Role.kIllustration
    visualizer = MeshcatVisualizer.AddToBuilder(
            builder.builder(), scene_graph, meshcat, meshcat_params)
    # X_WC = RigidTransform(RollPitchYaw(0,0,0),np.array([5, 4, 2]) ) # some drake.RigidTransform()
    # meshcat.SetTransform("/Cameras/default", X_WC) 
    # meshcat.SetProperty("/Background", "top_color", [0.8, 0.8, 0.6])
    # meshcat.SetProperty("/Background", "bottom_color",
    #                                 [0.9, 0.9, 0.9])
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(diagram_context)
    diagram.ForcedPublish(diagram_context)
    print(meshcat.web_url())
    return plant, scene_graph, diagram, diagram_context, plant_context, meshcat if usemeshcat else None
    

def plant_builder_7dof_iiwa(usemeshcat = False):
    if usemeshcat:
        meshcat = StartMeshcat()
    builder = RobotDiagramBuilder()
    plant = builder.plant()
    scene_graph = builder.scene_graph()
    parser = builder.parser()
    #parser.package_map().Add("cvisirisexamples", missing directory)
    if usemeshcat:
        visualizer = MeshcatVisualizer.AddToBuilder(builder.builder(), scene_graph, meshcat)
    directives_file = os.path.dirname(os.path.abspath(__file__)) +"/directives/7_dof_directives_newshelf.yaml"#FindResourceOrThrow() 
    path_repo = os.path.dirname(os.path.abspath(__file__)) 
    parser.package_map().Add("iris_environments", path_repo+"/assets")
    directives = LoadModelDirectives(directives_file)
    models = ProcessModelDirectives(directives, plant, parser)
    plant.Finalize()
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(diagram_context)
    diagram.ForcedPublish(diagram_context)
    return plant, scene_graph, diagram, diagram_context, plant_context, meshcat if usemeshcat else None

def plant_builder_15_dof_allegro_hand(usemeshcat = False):
    if usemeshcat:
        meshcat = StartMeshcat()
    builder = RobotDiagramBuilder()
    plant = builder.plant()
    scene_graph = builder.scene_graph()
    parser = builder.parser()
    #parser.package_map().Add("cvisirisexamples", missing directory)
    directives_file = os.path.dirname(os.path.abspath(__file__)) +"/directives/allegro_hand_directives.yaml"#FindResourceOrThrow() 
    path_repo = os.path.dirname(os.path.abspath(__file__)) 
    parser.package_map().Add("iris_environments", path_repo+"/assets")
    directives = LoadModelDirectives(directives_file)
    models = ProcessModelDirectives(directives, plant, parser)
    plant.Finalize()
    if usemeshcat:
        meshcat_params = MeshcatVisualizerParams()
        meshcat_params.role = Role.kIllustration
        visualizer = AddDefaultVisualization(builder.builder(), meshcat)
   
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(diagram_context)
    diagram.ForcedPublish(diagram_context)
    return plant, scene_graph, diagram, diagram_context, plant_context, meshcat if usemeshcat else None


def plant_builder_mycobot(usemeshcat = False):
    if usemeshcat:
        #meshcat = StartMeshcat()
        meldis = Meldis()
        meshcat = meldis.meshcat
    builder = RobotDiagramBuilder()
    plant = builder.plant()
    scene_graph = builder.scene_graph()
    parser = builder.parser()
    #parser.package_map().Add("cvisirisexamples", missing directory)
    directives_file = os.path.dirname(os.path.abspath(__file__)) +"/directives/mycobot.yaml"#FindResourceOrThrow() 
    path_repo = os.path.dirname(os.path.abspath(__file__)) 
    parser.package_map().Add("mycobot_description", path_repo+"/assets/mycobot_description")
    parser.package_map().Add("iris_environments", path_repo+"/assets")
    directives = LoadModelDirectives(directives_file)
    models = ProcessModelDirectives(directives, plant, parser)
    plant.Finalize()
    if usemeshcat:
        visualizer = AddDefaultVisualization(builder.builder(), meshcat)
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(diagram_context)
    diagram.ForcedPublish(diagram_context)
    return plant, scene_graph, diagram, diagram_context, plant_context, meshcat if usemeshcat else None

def plant_builder_7dof_4shelves(usemeshcat = False):
    if usemeshcat:
        meld = Meldis()
        meshcat = meld.meshcat
    builder = RobotDiagramBuilder()
    plant = builder.plant()
    scene_graph = builder.scene_graph()
    parser = builder.parser()
    #parser.package_map().Add("cvisirisexamples", missing directory)
    
    directives_file = os.path.dirname(os.path.abspath(__file__)) +"/directives/7_dof_directives_4shelves.yaml"#FindResourceOrThrow() 
    path_repo = os.path.dirname(os.path.abspath(__file__))
    parser.package_map().Add("iris_environments", path_repo+"/assets")
    directives = LoadModelDirectives(directives_file)
    models = ProcessModelDirectives(directives, plant, parser)
    plant.Finalize()
    if usemeshcat:
        visualizer = AddDefaultVisualization(builder.builder(), meshcat)
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(diagram_context)
    diagram.ForcedPublish(diagram_context)
    return plant, scene_graph, diagram, diagram_context, plant_context, meshcat if usemeshcat else None

def plant_builder_7dof_bins(usemeshcat = False):
    if usemeshcat:
        #meshcat = StartMeshcat()
        meldis = Meldis()
        meshcat = meldis.meshcat
    builder = RobotDiagramBuilder()
    plant = builder.plant()
    scene_graph = builder.scene_graph()
    parser = builder.parser()
    #parser.package_map().Add("cvisirisexamples", missing directory)
    
    directives_file = os.path.dirname(os.path.abspath(__file__)) +"/directives/7dof_bins_example.yaml"#FindResourceOrThrow() 
    path_repo = os.path.dirname(os.path.abspath(__file__))
    parser.package_map().Add("iris_environments", path_repo+"/assets")
    directives = LoadModelDirectives(directives_file)
    models = ProcessModelDirectives(directives, plant, parser)
    plant.Finalize()
    if usemeshcat:
        #par = MeshcatVisualizerParams()
        #par.role = Role.kIllustration
        #visualizer = MeshcatVisualizer.AddToBuilder(builder.builder(), scene_graph, meshcat, par)
        visualizer = AddDefaultVisualization(builder.builder(), meshcat)
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(diagram_context)
    diagram.ForcedPublish(diagram_context)
    return plant, scene_graph, diagram, diagram_context, plant_context, meshcat if usemeshcat else None


def environment_builder_14dof_iiwas(usemeshcat = False):
    builder = RobotDiagramBuilder()
    if usemeshcat: meshcat = StartMeshcat()
    # if export_sg_input:
    #     scene_graph = builder.AddSystem(SceneGraph())
    #     plant = MultibodyPlant(time_step=0.0)
    #     plant.RegisterAsSourceForSceneGraph(scene_graph)
    #     builder.ExportInput(scene_graph.get_source_pose_port(plant.get_source_id()), "source_pose")
    # else:
    plant = builder.plant()
    scene_graph = builder.scene_graph()# AddMultibodyPlantSceneGraph(builder, time_step=0.0)

    parser = builder.parser()
    parser.package_map().Add("bimanual", os.path.dirname(os.path.dirname(os.path.realpath(__file__)))+"/iris_environments/assets_bimanual")

    directives = LoadModelDirectives(os.path.dirname(os.path.abspath(__file__)) +
                                     "/assets_bimanual/models/bimanual_iiwa_with_shelves.yaml")
    models = ProcessModelDirectives(directives, plant, parser)

    plant.Finalize()
    if usemeshcat:
        meshcat_visual_params = MeshcatVisualizerParams()
        meshcat_visual_params.delete_on_initialization_event = False
        meshcat_visual_params.role = Role.kIllustration
        meshcat_visual_params.prefix = "visual"
        meshcat_visual = MeshcatVisualizer.AddToBuilder(
            builder.builder(), scene_graph, meshcat, meshcat_visual_params)

        meshcat_collision_params = MeshcatVisualizerParams()
        meshcat_collision_params.delete_on_initialization_event = False
        meshcat_collision_params.role = Role.kProximity
        meshcat_collision_params.prefix = "collision"
        meshcat_collision_params.visible_by_default = False
        meshcat_collision = MeshcatVisualizer.AddToBuilder(
            builder.builder(), scene_graph, meshcat, meshcat_collision_params)


    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(diagram_context)
    diagram.ForcedPublish(diagram_context)
    scene_graph_context = scene_graph.GetMyMutableContextFromRoot(
        diagram_context)
    return plant, scene_graph, diagram, diagram_context, plant_context, meshcat if usemeshcat else None

def get_environment_builder(environment_name):
    valid_names = env_names + ['MYCOBOT', '2DOFBLOCKS']
    if not environment_name in valid_names:
        raise ValueError(f"Choose a valid environment {valid_names}")
    if environment_name == '2DOFBLOCKS':
        return plant_builder_2dof_blocks
    if environment_name == '2DOFFLIPPER':
        return plant_builder_2dof_flipper2_obs
    if environment_name == '3DOFFLIPPER':
        return plant_builder_3dof_flipper
    elif environment_name == '5DOFUR3':
        return plant_builder_5dof_ur3
    elif environment_name == '6DOFUR3':
        return plant_builder_6dof_ur3
    elif environment_name == 'MYCOBOT':
        return plant_builder_mycobot
    elif environment_name == '7DOFIIWA':
        return plant_builder_7dof_iiwa
    elif environment_name == '7DOFBINS':
        return plant_builder_7dof_bins
    elif environment_name == '7DOF4SHELVES':
        return plant_builder_7dof_4shelves
    elif environment_name == '14DOFIIWAS':
        return environment_builder_14dof_iiwas
    elif environment_name == '15DOFALLEGRO':
        return plant_builder_15_dof_allegro_hand
    return None

def get_robot_instance_names(environment_name):
    assert environment_name in env_names+['MYCOBOT', '2DOFBLOCKS']
    if environment_name == '2DOFBLOCKS':
        return ["ball_in_boxes"]
    if environment_name == '2DOFFLIPPER':
        return ["iiwa7_twoDOF"]
    if environment_name == '3DOFFLIPPER':
        return ['iiwa7_oneDOF', 'iiwa7_twoDOF']
    elif environment_name == '5DOFUR3':
        return ["ur3e", "Schunk_Gripper"]
    elif environment_name == '6DOFUR3':
        return ["ur3e", "Schunk_Gripper"]
    elif environment_name == '7DOFIIWA':
        return ["iiwa", "wsg"]
    elif environment_name == '7DOFBINS':
        return ["iiwa", "wsg"]
    elif environment_name == '7DOF4SHELVES':
        return ["iiwa", "wsg"]
    elif environment_name == '14DOFIIWAS':
        return ["iiwa_left", "wsg_left", "iiwa_right", "wsg_right"]
    elif environment_name == '15DOFALLEGRO':
        return ["allegro_hand"]