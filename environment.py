from omni.isaac.core.objects import FixedCuboid

from omni.isaac.core.objects.ground_plane import GroundPlane
import omni.isaac.core.utils.prims as prim_utils

from omni.isaac.sensor import Camera
import omni.isaac.core.utils.numpy.rotations as rot_utils

import numpy as np
import json
import os

from settings import SITUATION_NAME, SITUATIONS_PATH
from settings import actual_environment_size_x, actual_environment_size_y, actual_environment_x_min, actual_environment_x_max, actual_environment_y_min, actual_environment_y_max
from settings import show_walls, show_door, show_grid_vis, show_victim_cube, show_test_wall
from grid import normalized_x_steps, normalized_y_steps
from grid import number_of_rows, number_of_columns
import util

def create_cube(world, path:str, name:str, id:int, position:np.ndarray, scale:np.ndarray, color:np.ndarray):
    world.scene.add(
        FixedCuboid(
            prim_path=f"/World/{path}/{name}{id:02}",
            name=f"{name.lower()}{id:02}",
            translation=position,
            scale=scale,  
            color=color,
        )
    )

wall_index = 0
def add_wall(world, position:np.ndarray, scale:np.ndarray, color:np.ndarray):
    global wall_index
    create_cube(world, "Walls", "Cube", wall_index, position, scale, color)
    wall_index += 1

def load_situation():
    folder_address = os.path.join(os.path.dirname(__file__), SITUATIONS_PATH, SITUATION_NAME)
    file_address = f"{folder_address}.json"
    if not os.path.isfile(file_address):
        util.debug_log("Situation", f"Could not load situation from file address {file_address}")
        return
    
    with open(file_address, 'r') as file:
        situation = json.load(file)
    
    return situation

def create_situation_walls(world, walls_color:np.ndarray):
    situation = load_situation()
    for situationJSON in situation["wall"]:
        position = situationJSON['position']
        position = np.array([position['x'], position['z'], position['y']])
        scale = situationJSON['scale']
        scale = np.array([scale['x'], scale['z'], scale['y']])
        add_wall(world, position, scale, walls_color)

def create_walls(world, walls_color):
    Cube_00 = world.scene.add(
        FixedCuboid(
            prim_path="/World/Walls/Cube_00",
            name="cube_00",
            translation=np.array([0, -0.5, 0.25]),
            scale=np.array([3, 0.1, 0.5]),  
            color=walls_color,
            # visual_material=walls_visual_material,
            # physics_material=walls_physics_material,
        ))
    Cube_01 = world.scene.add(
        FixedCuboid(
            prim_path="/World/Walls/Cube_01",
            name="cube_01",
            translation=np.array([1.5, 0.2, 0.25]),
            scale=np.array([0.1, 1.5, 0.5]),  
            color=walls_color,
            # visual_material=walls_visual_material,
            # physics_material=walls_physics_material,
        ))
    Cube_02 = world.scene.add(
        FixedCuboid(
            prim_path="/World/Walls/Cube_02",
            name="cube_02",
            translation=np.array([-1.5, 0.2, 0.25]),
            scale=np.array([0.1, 1.5, 0.5]),  
            color=walls_color,
            # visual_material=walls_visual_material,
            # physics_material=walls_physics_material,
        ))
    Cube_03 = world.scene.add(
        FixedCuboid(
            prim_path="/World/Walls/Cube_03",
            name="cube_03",
            translation=np.array([1.4, 0.9, 0.25]),
            scale=np.array([1.1, 0.1, 0.5]),  
            color=walls_color,
            # visual_material=walls_visual_material,
            # physics_material=walls_physics_material,
        ))
    Cube_04 = world.scene.add(
        FixedCuboid(
            prim_path="/World/Walls/Cube_04",
            name="cube_04",
            translation=np.array([-1.4, 0.9, 0.25]),
            scale=np.array([1.1, 0.1, 0.5]),  
            color=walls_color,
            # visual_material=walls_visual_material,
            # physics_material=walls_physics_material,
        ))
    Cube_05 = world.scene.add(
        FixedCuboid(
            prim_path="/World/Walls/Cube_05",
            name="cube_05",
            translation=np.array([0.0, 0.9, 0.25]),
            scale=np.array([0.4, 0.1, 0.5]), #np.array([0.8, 0.1, 0.5]),  
            color=walls_color,
            # visual_material=walls_visual_material,
            # physics_material=walls_physics_material,
        ))
    Cube_06 = world.scene.add(
        FixedCuboid(
            prim_path="/World/Walls/Cube_06",
            name="cube_06",
            translation=np.array([0.0, 1.6, 0.25]),
            scale=np.array([0.1, 1.5, 0.5]),  
            color=walls_color,
            # visual_material=walls_visual_material,
            # physics_material=walls_physics_material,
        ))
    Cube_07 = world.scene.add(
        FixedCuboid(
            prim_path="/World/Walls/Cube_07",
            name="cube_07",
            translation=np.array([2.0, 1.6, 0.25]),
            scale=np.array([0.1, 1.5, 0.5]),  
            color=walls_color,
            # visual_material=walls_visual_material,
            # physics_material=walls_physics_material,
        ))
    Cube_08 = world.scene.add(
        FixedCuboid(
            prim_path="/World/Walls/Cube_08",
            name="cube_08",
            translation=np.array([-2.0, 1.6, 0.25]),
            scale=np.array([0.1, 1.5, 0.5]),  
            color=walls_color,
            # visual_material=walls_visual_material,
            # physics_material=walls_physics_material,
        ))
    Cube_09 = world.scene.add(
        FixedCuboid(
            prim_path="/World/Walls/Cube_09",
            name="cube_09",
            translation=np.array([0.0, 2.3, 0.25]),
            scale=np.array([4.0, 0.1, 0.5]),  
            color=walls_color,
            # visual_material=walls_visual_material,
            # physics_material=walls_physics_material,
        ))
    return

def create_grid_vis(world):
    base_grid_vis_prim_path = "/World/Grid_Vis/Grid_Vis_"
    base_grid_vis_name="grid_vis_"
    
    wall_color_1 = np.array([1, 1, 1])
    wall_color_2 = np.array([0.01, 1, 0.01])
    # wall_color = wall_color_1
    trans_height = -0.05

    low_x = np.floor(number_of_rows/2) - 1
    high_x = np.ceil(number_of_rows/2) + 1
    low_y = np.floor(number_of_columns/2) - 1
    high_y = np.ceil(number_of_columns/2) + 1

    for i in range(number_of_rows+1):
        x = actual_environment_x_min + i*normalized_x_steps
        if (low_x < i < high_x):
            wall_color = wall_color_2
        else:
            wall_color = wall_color_1
        world.scene.add(
            FixedCuboid(
                prim_path=f"{base_grid_vis_prim_path}x{i:01}",
                name=f"{base_grid_vis_name}x{i:01}",
                # translation=np.array([x, (0-actual_environment_y_min)/2, trans_height]),
                translation=np.array([x, 0, trans_height]),
                scale=np.array([0.01, actual_environment_size_y, 0.1]),  
                color=wall_color
            ))
    for j in range(number_of_columns+1):
        y = actual_environment_y_min + j*normalized_y_steps
        if (low_y < j < high_y):
            wall_color = wall_color_2
        else:
            wall_color = wall_color_1
        world.scene.add(
            FixedCuboid(
                prim_path=f"{base_grid_vis_prim_path}y{j:01}",
                name=f"{base_grid_vis_name}y{j:01}",
                translation=np.array([0, y, trans_height]),
                scale=np.array([actual_environment_size_x, 0.01, 0.1]),  
                color=wall_color,
        ))

def create_grid_vis_cells(world):
    base_grid_vis_prim_path = "/World/Grid_Vis/Grid_Vis_"
    base_grid_vis_name="grid_vis_"

    
    wall_color_1 = np.array([0, 0, 0])
    wall_color_2 = np.array([0.01, 1, 0.01])
    # wall_color = wall_color_1
    trans_height = -0.05

    low_x = np.floor(number_of_rows/2) - 1
    high_x = np.ceil(number_of_rows/2) + 1
    low_y = np.floor(number_of_columns/2) - 1
    high_y = np.ceil(number_of_columns/2) + 1

    for i in range(number_of_rows+1):
        x = actual_environment_x_min + i*normalized_x_steps
        if (low_x < i < high_x):
            wall_color = wall_color_2
        else:
            wall_color = wall_color_1
        world.scene.add(
            FixedCuboid(
                prim_path=f"{base_grid_vis_prim_path}x{i:01}",
                name=f"{base_grid_vis_name}x{i:01}",
                # translation=np.array([x, (0-actual_environment_y_min)/2, trans_height]),
                translation=np.array([x, 0, trans_height]),
                scale=np.array([0.01, actual_environment_size_y, 0.1]),  
                color=wall_color
            ))
    for j in range(number_of_columns+1):
        y = actual_environment_y_min + j*normalized_y_steps
        if (low_y < j < high_y):
            wall_color = wall_color_2
        else:
            wall_color = wall_color_1
        world.scene.add(
            FixedCuboid(
                prim_path=f"{base_grid_vis_prim_path}y{j:01}",
                name=f"{base_grid_vis_name}y{j:01}",
                translation=np.array([0, y, trans_height]),
                scale=np.array([actual_environment_size_x, 0.01, 0.1]),  
                color=wall_color,
        ))

def setup_environment(world):
    ### Ground planes ###
    # world.scene.add_default_ground_plane()                                                # Default
    GroundPlane(prim_path="/World/groundPlane", size=10, color=np.array([0.1, 0.1, 0.4]))   # Custom
    
    ### Lights ###
    light_1 = prim_utils.create_prim(
        "/World/Light_1",
        "SphereLight",
        position=np.array([0.0, 0.0, 2.5]),
        attributes={
            "inputs:radius": 0.25,
            "inputs:intensity": 30e3,
            "inputs:color": (1.0, 1.0, 1.0)
        }
    )
    light_2 = prim_utils.create_prim(
        "/World/Light_2",
        "SphereLight",
        position=np.array([0.0, 1.25, 2.5]),
        attributes={
            "inputs:radius": 0.25,
            "inputs:intensity": 30e3,
            "inputs:color": (1.0, 1.0, 1.0)
        }
    )

    ### Camera ###
    TopDownCamera = Camera(
        prim_path="/World/TopDownCamera",
        position=np.array([0.0, 1.25, 13.0]),
        frequency=20,
        resolution=(256, 256),
        orientation=[0.0, 0.70711, 0.0, -0.70711]
        # [0, 0, 90] is [0.0, 0.70711, 0.0, -0.70711] || OR || rot_utils.euler_angles_to_quats(np.array([180, -90, 0]), degrees=True, extrinsic=False)  
        
        # Trial and error to find correct orientation:    
        # [-90, 0, 0]   -> [0, -90, 0]
        # [90, 0, 0]    -> [-180, -90, 0]

        # [0, 180, 0]   -> [-90, 90, 0]
        # [0, 90, 0]    -> [0, 0, -90]      closest
        # [0, 0, 0]     -> [90, -90, 0]
        # [0, -90, 0]   -> [-180, 0, 90]
        # [0, -180, 0]  -> [-90, 90, 0]
        
        # [0, 90, 90]   -> [90, 0, -90]
        # [0, 90, -90]  -> [-90, 0, -90]
        # [180, -90, 0] -> [0, 0, 90]       desired found
        # [90, 90, 0]   -> [90, 0, -90]
        # [-90, 90, 0]  -> [-90, 0, -90]
    
    )
    
    ### Togglable Enviroment ###
    if show_walls:
        walls_color = np.array([1, 0.5, 0.5])
        # walls_visual_material = 
        # walls_physics_material = 
        create_walls(world, walls_color)    # Create Walls
        # create_situation_walls(world, walls_color) # Create walls from situation data
    if show_grid_vis:
        create_grid_vis(world)          # Create Grid Visualisation
    if show_victim_cube:
        victim_cube = world.scene.add(
            FixedCuboid(
                prim_path="/World/Victim_Cube",
                name="victim_cube",
                translation=np.array([-1.3, 1.6, 0.03]),
                scale=np.array([0.05, 0.05, 0.05]),  
                color=np.array([0.0, 1.0, 0.0]),
                # visual_material=walls_visual_material,
                # physics_material=walls_physics_material,
            )
        )
    if show_door:
        # Door
        Cube_10 = world.scene.add(
            FixedCuboid(
                prim_path="/World/Walls/Cube_10",
                name="cube_10",
                translation=np.array([-0.625, 0.9, 0.25]),
                scale=np.array([0.45, 0.1, 0.5]),  
                color=np.array([0.5, 0, 0]),
                # visual_material=walls_visual_material,
                # physics_material=walls_physics_material,
            )
        )
    if show_test_wall:
        # Test Wall for find_collision_points()
        Cube_11 = world.scene.add(
            FixedCuboid(
                prim_path="/World/Walls/Cube_11",
                name="cube_11",
                translation=np.array([-1.4, 0.0, 0.25]),
                scale=np.array([0.1, 0.45, 0.5]),  
                color=np.array([0.5, 0, 0]),
                # visual_material=walls_visual_material,
                # physics_material=walls_physics_material,
            )
        )

    return