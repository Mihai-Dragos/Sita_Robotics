from omni.isaac.core.objects import FixedCuboid

import numpy as np
import json
import os

from settings import SITUATION_NAME, SITUATIONS_PATH
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
    file_address = f"{os.path.dirname(__file__)}/{SITUATIONS_PATH}/{SITUATION_NAME}.json"
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

# def create_walls(world, walls_color):
#     Cube_00 = world.scene.add(
#         FixedCuboid(
#             prim_path="/World/Walls/Cube_00",
#             name="cube_00",
#             translation=np.array([0, -0.5, 0.25]),
#             scale=np.array([3, 0.1, 0.5]),  
#             color=walls_color,
#             # visual_material=walls_visual_material,
#             # physics_material=walls_physics_material,
#         ))
#     Cube_01 = world.scene.add(
#         FixedCuboid(
#             prim_path="/World/Walls/Cube_01",
#             name="cube_01",
#             translation=np.array([1.5, 0.2, 0.25]),
#             scale=np.array([0.1, 1.5, 0.5]),  
#             color=walls_color,
#             # visual_material=walls_visual_material,
#             # physics_material=walls_physics_material,
#         ))
#     Cube_02 = world.scene.add(
#         FixedCuboid(
#             prim_path="/World/Walls/Cube_02",
#             name="cube_02",
#             translation=np.array([-1.5, 0.2, 0.25]),
#             scale=np.array([0.1, 1.5, 0.5]),  
#             color=walls_color,
#             # visual_material=walls_visual_material,
#             # physics_material=walls_physics_material,
#         ))
#     Cube_03 = world.scene.add(
#         FixedCuboid(
#             prim_path="/World/Walls/Cube_03",
#             name="cube_03",
#             translation=np.array([1.4, 0.9, 0.25]),
#             scale=np.array([1.1, 0.1, 0.5]),  
#             color=walls_color,
#             # visual_material=walls_visual_material,
#             # physics_material=walls_physics_material,
#         ))
#     Cube_04 = world.scene.add(
#         FixedCuboid(
#             prim_path="/World/Walls/Cube_04",
#             name="cube_04",
#             translation=np.array([-1.4, 0.9, 0.25]),
#             scale=np.array([1.1, 0.1, 0.5]),  
#             color=walls_color,
#             # visual_material=walls_visual_material,
#             # physics_material=walls_physics_material,
#         ))
#     Cube_05 = world.scene.add(
#         FixedCuboid(
#             prim_path="/World/Walls/Cube_05",
#             name="cube_05",
#             translation=np.array([0.0, 0.9, 0.25]),
#             scale=np.array([0.4, 0.1, 0.5]), #np.array([0.8, 0.1, 0.5]),  
#             color=walls_color,
#             # visual_material=walls_visual_material,
#             # physics_material=walls_physics_material,
#         ))
#     Cube_06 = world.scene.add(
#         FixedCuboid(
#             prim_path="/World/Walls/Cube_06",
#             name="cube_06",
#             translation=np.array([0.0, 1.6, 0.25]),
#             scale=np.array([0.1, 1.5, 0.5]),  
#             color=walls_color,
#             # visual_material=walls_visual_material,
#             # physics_material=walls_physics_material,
#         ))
#     Cube_07 = world.scene.add(
#         FixedCuboid(
#             prim_path="/World/Walls/Cube_07",
#             name="cube_07",
#             translation=np.array([2.0, 1.6, 0.25]),
#             scale=np.array([0.1, 1.5, 0.5]),  
#             color=walls_color,
#             # visual_material=walls_visual_material,
#             # physics_material=walls_physics_material,
#         ))
#     Cube_08 = world.scene.add(
#         FixedCuboid(
#             prim_path="/World/Walls/Cube_08",
#             name="cube_08",
#             translation=np.array([-2.0, 1.6, 0.25]),
#             scale=np.array([0.1, 1.5, 0.5]),  
#             color=walls_color,
#             # visual_material=walls_visual_material,
#             # physics_material=walls_physics_material,
#         ))
#     Cube_09 = world.scene.add(
#         FixedCuboid(
#             prim_path="/World/Walls/Cube_09",
#             name="cube_09",
#             translation=np.array([0.0, 2.3, 0.25]),
#             scale=np.array([4.0, 0.1, 0.5]),  
#             color=walls_color,
#             # visual_material=walls_visual_material,
#             # physics_material=walls_physics_material,
#         ))
#     return

# def create_grid_vis(world):
#     base_grid_vis_prim_path = "/World/Grid_Vis/Grid_Vis_"
#     base_grid_vis_name="grid_vis_"
    
#     wall_color_1 = np.array([1, 1, 1])
#     wall_color_2 = np.array([0.01, 1, 0.01])
#     # wall_color = wall_color_1
#     trans_height = -0.05

#     low_x = np.floor(number_of_rows/2) - 1
#     high_x = np.ceil(number_of_rows/2) + 1
#     low_y = np.floor(number_of_columns/2) - 1
#     high_y = np.ceil(number_of_columns/2) + 1

#     for i in range(number_of_rows+1):
#         x = actual_environment_x_min + i*normalized_x_steps
#         if (low_x < i < high_x):
#             wall_color = wall_color_2
#         else:
#             wall_color = wall_color_1
#         create_cube(world, base_grid_vis_prim_path, 
#                     base_grid_vis_name, i,
#                     np.array([x, 0, trans_height]), 
#                     np.array([0.01, actual_environment_size_y, 0.1]), 
#                     wall_color)
#     for j in range(number_of_columns+1):
#         y = actual_environment_y_min + j*normalized_y_steps
#         if (low_y < j < high_y):
#             wall_color = wall_color_2
#         else:
#             wall_color = wall_color_1
#         world.scene.add(
#             FixedCuboid(
#                 prim_path=f"{base_grid_vis_prim_path}y{j:01}",
#                 name=f"{base_grid_vis_name}y{j:01}",
#                 translation=np.array([0, y, trans_height]),
#                 scale=np.array([actual_environment_size_x, 0.01, 0.1]),  
#                 color=wall_color,
#         ))

# def create_grid_vis_cells(world):
#     base_grid_vis_prim_path = "/World/Grid_Vis/Grid_Vis_"
#     base_grid_vis_name="grid_vis_"

    
#     wall_color_1 = np.array([0, 0, 0])
#     wall_color_2 = np.array([0.01, 1, 0.01])
#     # wall_color = wall_color_1
#     trans_height = -0.05

#     low_x = np.floor(number_of_rows/2) - 1
#     high_x = np.ceil(number_of_rows/2) + 1
#     low_y = np.floor(number_of_columns/2) - 1
#     high_y = np.ceil(number_of_columns/2) + 1

#     for i in range(number_of_rows+1):
#         x = actual_environment_x_min + i*normalized_x_steps
#         if (low_x < i < high_x):
#             wall_color = wall_color_2
#         else:
#             wall_color = wall_color_1
#         world.scene.add(
#             FixedCuboid(
#                 prim_path=f"{base_grid_vis_prim_path}x{i:01}",
#                 name=f"{base_grid_vis_name}x{i:01}",
#                 # translation=np.array([x, (0-actual_environment_y_min)/2, trans_height]),
#                 translation=np.array([x, 0, trans_height]),
#                 scale=np.array([0.01, actual_environment_size_y, 0.1]),  
#                 color=wall_color
#             ))
#     for j in range(number_of_columns+1):
#         y = actual_environment_y_min + j*normalized_y_steps
#         if (low_y < j < high_y):
#             wall_color = wall_color_2
#         else:
#             wall_color = wall_color_1
#         world.scene.add(
#             FixedCuboid(
#                 prim_path=f"{base_grid_vis_prim_path}y{j:01}",
#                 name=f"{base_grid_vis_name}y{j:01}",
#                 translation=np.array([0, y, trans_height]),
#                 scale=np.array([actual_environment_size_x, 0.01, 0.1]),  
#                 color=wall_color,
#         ))