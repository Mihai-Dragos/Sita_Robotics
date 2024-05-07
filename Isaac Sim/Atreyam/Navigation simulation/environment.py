from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.wheeled_robots.robots import WheeledRobot

from omni.isaac.core.objects import FixedCuboid

from omni.isaac.core.objects.ground_plane import GroundPlane
import omni.isaac.core.utils.prims as prim_utils

import numpy as np

from omni.isaac.examples.user_examples.settings import num_robots
from omni.isaac.examples.user_examples.settings import actual_environment_size_x, actual_environment_size_y
from omni.isaac.examples.user_examples.settings import actual_environment_x_min, actual_environment_x_max 
from omni.isaac.examples.user_examples.settings import actual_environment_y_min, actual_environment_y_max
from omni.isaac.examples.user_examples.grid import normalized_x_steps, normalized_y_steps
from omni.isaac.examples.user_examples.grid import number_of_rows, number_of_columns

def create_robots(world, typeRobot, lineMode):
    assets_root_path = get_assets_root_path()
    jetbot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
    kaya_asset_path = assets_root_path + "/Isaac/Robots/Kaya/kaya.usd"
    base_robot_prim_path = "/World/Fancy_Robot_"
    base_robot_name="fancy_robot_"
    # spawn_orientation = np.array([0.70711, 0, 0, -0.70711])
    spawn_orientation = np.array([1,0,0,0])

    if lineMode:
        robot_spawn_positions_x_limits = [-0.7 , 0.7] #[-3.5*3/11, 3.5*3/11]
        robot_spawn_positions_y_limits = [0, 0]
        robot_spawn_positions_x = np.linspace(robot_spawn_positions_x_limits[0], robot_spawn_positions_x_limits[1], num_robots)
        robot_spawn_positions_y = np.linspace(robot_spawn_positions_y_limits[0], robot_spawn_positions_y_limits[1], num_robots)

        for robot_index in range(num_robots):
                if typeRobot == 1:       # Jetbot
                    world.scene.add(
                    WheeledRobot(
                        prim_path=f"{base_robot_prim_path}{robot_index:02}",
                        name=f"{base_robot_name}{robot_index:02}",
                        wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                        create_robot=True,
                        usd_path=jetbot_asset_path,
                        position=[robot_spawn_positions_x[robot_index],robot_spawn_positions_y[robot_index],0],
                        orientation=spawn_orientation,
                    )
                )
                elif typeRobot ==  2:    # Kaya
                    world.scene.add(
                    WheeledRobot(
                        prim_path=f"{base_robot_prim_path}{robot_index:02}",
                        name=f"{base_robot_name}{robot_index:02}",
                        wheel_dof_names=["axle_0_joint", "axle_1_joint","axle_2_joint"],
                        create_robot=True,
                        usd_path=kaya_asset_path,
                        position=[robot_spawn_positions_x[robot_index],robot_spawn_positions_y[robot_index],0],
                        orientation=spawn_orientation,
                    )
                )    
                        
                    
    else:
        robot_spawn_positions_x_limits = [-1.3, 1.3] #[-3*3/11, 3*3/11]
        robot_spawn_positions_y_limits = [-0.3, 0.7] #[-1*3/11, 1*3/11]
        robot_cols = int(np.ceil(np.sqrt(num_robots)))
        robot_rows = int(np.ceil(num_robots / robot_cols))
        robot_spawn_positions_x = np.linspace(robot_spawn_positions_x_limits[0], robot_spawn_positions_x_limits[1], robot_cols)
        robot_spawn_positions_y = np.linspace(robot_spawn_positions_y_limits[0], robot_spawn_positions_y_limits[1], robot_rows)

        robot_index = 0
        for y in range(robot_rows):
            for x in range(robot_cols):
                if robot_index < num_robots:
                    if typeRobot == 1:       # Jetbot
                        world.scene.add(
                            WheeledRobot(
                                prim_path=f"{base_robot_prim_path}{robot_index:02}",
                                name=f"{base_robot_name}{robot_index:02}",
                                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                                create_robot=True,
                                usd_path=jetbot_asset_path,
                                position=[robot_spawn_positions_x[x % robot_cols],robot_spawn_positions_y[y % robot_rows],0],
                                orientation=spawn_orientation,
                            )
                        )
                    elif typeRobot ==  2:    # Kaya
                        world.scene.add(
                            WheeledRobot(
                                prim_path=f"{base_robot_prim_path}{robot_index:02}",
                                name=f"{base_robot_name}{robot_index:02}",
                                wheel_dof_names=["axle_0_joint", "axle_1_joint","axle_2_joint"],
                                create_robot=True,
                                usd_path=kaya_asset_path,
                                position=[robot_spawn_positions_x[x % robot_cols],robot_spawn_positions_y[y % robot_rows],0],
                                orientation=spawn_orientation,
                            )
                        )
                    robot_index += 1
    return

def create_lidars(lidarsDrawLines, lidarsDrawPoints):
    base_lidar_path = "/Lidar_"
    base_lidar_parent = "/World/Fancy_Robot_"
    for i in range(num_robots):
        omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path=f"{base_lidar_path}{i:02}",
            parent=f"{base_lidar_parent}{i:02}/chassis",
            min_range=0.1,
            max_range=15.0,
            draw_points=lidarsDrawPoints,
            draw_lines=lidarsDrawLines,
            horizontal_fov=360.0,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=4.0,
            rotation_rate=0.0,
            high_lod=False,
            yaw_offset=0.0,
            enable_semantics=False
        )

    return

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
            scale=np.array([0.8, 0.1, 0.5]),  
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
    # world.scene.add_default_ground_plane()
    
    # PhysicsContext()
    GroundPlane(prim_path="/World/groundPlane", size=10, color=np.array([0.1, 0.1, 0.4]))
    # Dark Grey: np.array([0.11, 0.11, 0.11])
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


    # Create Robots
    lineMode = True
    typeRobot = 1       # 1 -> Jetbot, 2 -> Kaya
    create_robots(world, typeRobot, lineMode)

    # Create Lidars
    lidarsDrawLines = False
    lidarsDrawPoints = False
    create_lidars(lidarsDrawLines, lidarsDrawPoints)

    # Create Walls
    walls_color = np.array([1, 0.5, 0.5])
    # walls_visual_material = 
    # walls_physics_material = 
    # create_walls(world, walls_color)

    # Create Grid Visualisation
    create_grid_vis(world)

    # victim_cube = world.scene.add(
    #     FixedCuboid(
    #         prim_path="/World/Victim_Cube",
    #         name="victim_cube",
    #         translation=np.array([-1.3, 1.6, 0.03]),
    #         scale=np.array([0.05, 0.05, 0.05]),  
    #         color=np.array([0.0, 1.0, 0.0]),
    #         # visual_material=walls_visual_material,
    #         # physics_material=walls_physics_material,
    #     )
    # )
    
    # # Door
    # Cube_10 = world.scene.add(
    #     FixedCuboid(
    #         prim_path="/World/Walls/Cube_10",
    #         name="cube_10",
    #         translation=np.array([-0.625, 0.9, 0.25]),
    #         scale=np.array([0.45, 0.1, 0.5]),  
    #         color=np.array([0.5, 0, 0]),
    #         # visual_material=walls_visual_material,
    #         # physics_material=walls_physics_material,
    #     )
    # )
    # # Test Wall for find_collision_points()
    # Cube_11 = world.scene.add(
    #     FixedCuboid(
    #         prim_path="/World/Walls/Cube_11",
    #         name="cube_11",
    #         translation=np.array([-1.4, 0.0, 0.25]),
    #         scale=np.array([0.1, 0.45, 0.5]),  
    #         color=np.array([0.5, 0, 0]),
    #         # visual_material=walls_visual_material,
    #         # physics_material=walls_physics_material,
    #     )
    # )

    return