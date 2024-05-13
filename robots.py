from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.objects import VisualSphere

import omni                                                     # Provides the core omniverse apis

import numpy as np

from omni.isaac.examples.user_examples.git_isaac_sim.settings import num_robots

def create_robots(world, typeRobot, lineMode):
    assets_root_path = get_assets_root_path()
    jetbot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
    kaya_asset_path = assets_root_path + "/Isaac/Robots/Kaya/kaya.usd"
    base_robot_prim_path = "/World/Robot_"
    base_robot_name="robot_"
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
        robot_spawn_positions_x_limits = [-0.7 , 0.7] #[-1.3, 1.3] #[-3*3/11, 3*3/11]
        robot_spawn_positions_y_limits = [-0.7 , 0.4] #[-0.3, 0.7] #[-1*3/11, 1*3/11]
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
    base_lidar_parent = "/World/Robot_"
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

def create_vel_vis(world):

    # base_entering_sphere_prim_path = "/World/Robot_"
    # base_entering_sphere_name = "entering_sphere_" 
    # i = 0
    # entering_color = np.array([0.0, 0.0, 0.0])
    # entering_trans = -0.05

    # world.scene.add(
    #                 VisualSphere(
    #                     prim_path=f"{base_entering_sphere_prim_path}{i:02}/chassis/Entering_Sphere_{i:02}",
    #                     name=f"{base_entering_sphere_name}{i:02}",
    #                     # translation=np.array([x, (0-actual_environment_y_min)/2, trans_height]),
    #                     translation=np.array([entering_trans, 0, 0.15]),
    #                     scale=np.array([0.02, 0.02, 0.02]),  
    #                     color=entering_color
    #                 ))

    base_sphere_prim_path = "/World/Robot_"

    base_entering_sphere_prim_path_suffix = "/Entering_Sphere_"
    base_exploration_sphere_prim_path_suffix = "/Exploration_Sphere_"
    base_interaction_sphere_prim_path_suffix = "/Interaction_Sphere_"

    base_sphere_prim_path_suffix = []
    base_sphere_prim_path_suffix.append(base_entering_sphere_prim_path_suffix)
    base_sphere_prim_path_suffix.append(base_exploration_sphere_prim_path_suffix)
    base_sphere_prim_path_suffix.append(base_interaction_sphere_prim_path_suffix)

    base_entering_sphere_name = "entering_sphere_"
    base_exploration_sphere_name = "exploration_sphere_"
    base_interaction_sphere_name = "interaction_sphere_"

    base_sphere_name = []
    base_sphere_name.append(base_entering_sphere_name)
    base_sphere_name.append(base_exploration_sphere_name)
    base_sphere_name.append(base_interaction_sphere_name)

    entering_trans = -0.05
    exploration_trans = 0.0
    interaction_trans = 0.05 

    vel_trans = []
    vel_trans.append(entering_trans)
    vel_trans.append(exploration_trans)
    vel_trans.append(interaction_trans)

    entering_color = np.array([2.0, 0.0, 0.0])
    exploration_color = np.array([0.0, 2.0, 0.0])
    interaction_color = np.array([0.0, 0.0, 2.0])

    vel_color = []
    vel_color.append(entering_color)
    vel_color.append(exploration_color)
    vel_color.append(interaction_color)

    for i in range(num_robots):
        for v in range(3):
            world.scene.add(
                    VisualSphere(
                        prim_path=f"{base_sphere_prim_path}{i:02}/chassis{base_sphere_prim_path_suffix[v]}{i:02}",
                        name=f"{base_sphere_name[v]}{i:02}",
                        # translation=np.array([x, (0-actual_environment_y_min)/2, trans_height]),
                        translation=np.array([vel_trans[v], 0, 0.15]),
                        scale=np.array([0.02, 0.02, 0.02]),  
                        color=vel_color[v]
                    ))
    # return base_sphere_prim_path, base_sphere_prim_path_suffix

def setup_robots(world):
    # Create Robots
    lineMode = True
    typeRobot = 1       # 1 -> Jetbot, 2 -> Kaya
    create_robots(world, typeRobot, lineMode)

    # Create Lidars
    lidarsDrawLines = False
    lidarsDrawPoints = False
    create_lidars(lidarsDrawLines, lidarsDrawPoints)

    create_vel_vis(world)