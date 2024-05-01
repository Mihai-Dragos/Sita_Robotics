# Shape exploration velocity:
#   Created psi_weight() to calculate the weight psi
#   Created in_shape_boundary() to determine if robot is inside or outside of shape boundary
#   Created occupied_cells() to determine if cells are occupied, including the size of the robot
#   Created neighbouring_cells() to determine black cells within r_sense, (incl occupied cells if in_shape_boundary() == False)
#   Started shape_exploration_velocity(), still needs to be implemented using functions created above
# Troubleshooting Additions:
#   New shape `shape_array_largerect`, large rectangle in bottom half to test `in_shape_boundary`, `occupied_cells()`, and `neighboring_cells()`
#   `r_sense` made into global variable
#   `num_robots` changed to 3
#    `lidarsDrawLines` set to False
# 29 Apr 2024 

from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.utils.types import ArticulationAction


from omni.isaac.core.controllers import BaseController
from omni.isaac.wheeled_robots.controllers.wheel_base_pose_controller import WheelBasePoseController
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.wheeled_robots.controllers.holonomic_controller import HolonomicController

import numpy as np

from omni.isaac.core.objects import FixedCuboid

import omni                                                     # Provides the core omniverse apis
import asyncio                                                  # Used to run sample asynchronously to not block rendering thread
from omni.isaac.range_sensor import _range_sensor               # Imports the python bindings to interact with lidar sensor
from pxr import UsdGeom, Gf, UsdPhysics                         # pxr usd imports used to create the cube

from omni.isaac.core.objects.ground_plane import GroundPlane
from omni.isaac.core.physics_context import PhysicsContext
import omni.isaac.core.utils.prims as prim_utils


stage = omni.usd.get_context().get_stage()                      # Used to access Geometry
timeline = omni.timeline.get_timeline_interface()               # Used to interact with simulation
lidarInterface = _range_sensor.acquire_lidar_sensor_interface() # Used to interact with the LIDAR

# Used for velocity control
class CoolController(BaseController):
    def __init__(self):
        super().__init__(name="my_cool_controller")
       # An open loop controller that uses a unicycle model
        self._wheel_radius = 0.03
        self._wheel_base = 0.1125
        return

    def forward(self, command):
        # command will have two elements, first element is the forward velocity
        # second element is the angular velocity (yaw only).
        joint_velocities = [0.0, 0.0]
        joint_velocities[0] = ((2 * command[0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[1] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        # A controller has to return an ArticulationAction
        return ArticulationAction(joint_velocities=joint_velocities)

# Trying to convert the input: vector velocity to output steps doable by robot
class MyHoloController(BaseController):
    def __init__(self):
        super().__init__(name="my_holo_controller")
        self._wheel_radius = 0.03
        self._wheel_base = 0.1125
        return

    def vector_vel(self, vel):
        # command will have 3 elements, velocity in [x,y,z].
        # second element is the angular velocity (yaw only).
        
        command = [0.0, 0.0]
        command[0] = ((vel[0] ** 2) + (vel[1] ** 2)) ** 0.5                     # robot velocity amplitude
        # command[1] = get_robot_ori_euler(robot_index) - np.arctan2(vel[1],vel[0])   # robot direction (rad)
        joint_velocities = [0.0, 0.0]
        joint_velocities[0] = ((2 * command[0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[1] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        # A controller has to return an ArticulationAction
        return ArticulationAction(joint_velocities=joint_velocities)

# Number of robots
num_robots = 3

# Hardcode Input shape
shape_array_0 = np.array([[1, 1, 1, 1, 1, 1, 1],
                                [1, 1, 1, 1, 1, 1, 1],
                                [1, 1, 1, 1, 1, 1, 1],
                                [1, 1, 1, 0, 1, 1, 1],
                                [1, 1, 1, 1, 1, 1, 1],
                                [1, 1, 1, 1, 1, 1, 1],
                                [1, 1, 1, 1, 1, 1, 1]])
shape_array_0 = np.pad(shape_array_0, [(2,2),(2,2)], mode='constant', constant_values=1)
# h = 1

shape_array_tri = np.array([[1, 1, 1, 1, 1, 1, 1],
                                [1, 1, 1, 1, 0, 1, 1],
                                [1, 1, 1, 0, 0, 1, 1],
                                [1, 1, 0, 0, 0, 1, 1],
                                [1, 1, 1, 0, 0, 1, 1],
                                [1, 1, 1, 1, 0, 1, 1],
                                [1, 1, 1, 1, 1, 1, 1]])
shape_array_tri = np.pad(shape_array_tri, [(2,2),(2,2)], mode='constant', constant_values=1)
# h = 2

shape_array_line = np.array([[1, 1, 1, 1, 1, 1, 1],
                                [1, 1, 1, 1, 1, 1, 1],
                                [1, 1, 0, 1, 1, 1, 1],
                                [1, 1, 1, 0, 1, 1, 1],
                                [1, 1, 1, 1, 0, 1, 1],
                                [1, 1, 1, 1, 1, 1, 1],
                                [1, 1, 1, 1, 1, 1, 1]])
shape_array_line = np.pad(shape_array_line, [(2,2),(2,2)], mode='constant', constant_values=1)
# h = 1

shape_array_donut = np.array([[1, 1, 1, 1, 1, 1, 1],
                                [1, 1, 1, 1, 1, 1, 1],
                                [1, 1, 1, 0, 1, 1, 1],
                                [1, 1, 0, 1, 0, 1, 1],
                                [1, 1, 1, 0, 1, 1, 1],
                                [1, 1, 1, 1, 1, 1, 1],
                                [1, 1, 1, 1, 1, 1, 1]])
shape_array_donut = np.pad(shape_array_donut, [(2,2),(2,2)], mode='constant', constant_values=1)
# h = 1

shape_array_corners = np.array([[1, 1, 1, 1, 1, 1, 1],
                                [1, 1, 1, 1, 1, 1, 1],
                                [1, 1, 0, 1, 0, 1, 1],
                                [1, 1, 1, 1, 1, 1, 1],
                                [1, 1, 0, 1, 0, 1, 1],
                                [1, 1, 1, 1, 1, 1, 1],
                                [1, 1, 1, 1, 1, 1, 1]])
shape_array_corners = np.pad(shape_array_corners, [(2,2),(2,2)], mode='constant', constant_values=1)

shape_array_largerect = np.array([[1, 1, 1, 1, 1, 1, 1],
                                [1, 1, 1, 1, 1, 1, 1],
                                [1, 1, 1, 1, 1, 1, 1],
                                [1, 0, 0, 0, 0, 0, 1],
                                [1, 0, 0, 0, 0, 0, 1],
                                [1, 0, 0, 0, 0, 0, 1],
                                [1, 1, 1, 1, 1, 1, 1]])
shape_array_largerect = np.pad(shape_array_largerect, [(2,2),(2,2)], mode='constant', constant_values=1)

input_shape = shape_array_largerect # shape_array_0

# Hardcode Number of iterations
h = 2

shape_array_2 = np.array([[1, 1, 1, 1, 1, 1, 1],
                        [1, 2/3, 2/3, 2/3, 2/3, 2/3, 1],
                        [1, 2/3, 1/3, 1/3, 1/3, 2/3, 1],
                        [1, 2/3, 1/3, 0, 1/3, 2/3, 1],
                        [1, 2/3, 1/3, 1/3, 1/3, 2/3, 1],
                        [1, 2/3, 2/3, 2/3, 2/3, 2/3, 1],
                        [1, 1, 1, 1, 1, 1, 1]])

# Hardcode avoid radius of robots, used for calculating l_cell
r_avoid = 0.4 # 1.5 in Paper
# Hardcode sense radius of robots, used for calculating neighboring_i() and neighbouring_cells()
r_sense = 1 # 2.5 in Paper

# Hardcode actual enviroment size (of biggest room) in meters
# actual_environment_x_min = -1.4
# actual_environment_x_max = 1.4
# actual_environment_y_min = -0.4
# actual_environment_y_max = 0.8

# Hardcode set enviroment size in meters to test shape_entering_velocity()
a_e_v_amount = 1.5
actual_environment_x_min = -a_e_v_amount
actual_environment_x_max = a_e_v_amount
actual_environment_y_min = -a_e_v_amount
actual_environment_y_max = a_e_v_amount

actual_environment_size_x = actual_environment_x_max - actual_environment_x_min
actual_environment_size_y = actual_environment_y_max - actual_environment_y_min

# Hardcode inital v_rho0_i
robs_initial_v_rho0_i = [[0,0,0] for _ in range(num_robots)]

class HelloWorld(BaseSample):
    
    def __init__(self) -> None:
        super().__init__()
        self.v_rho0_cache = {}
        return

    def mod(self, a, n):
        res = a - np.floor(a/n) * n
        return res

    def create_robots(self,typeRobot,lineMode):
        world = self.get_world()
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

    def create_lidars(self, lidarsDrawLines, lidarsDrawPoints):
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

    def create_walls(self, walls_color):
        world = self.get_world()
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

    def create_grid_vis(self):
        world = self.get_world()
        _, row, col, l_cell = self.calculate_rho_0()

        normalized_x_steps = actual_environment_size_x/row
        normalized_y_steps = actual_environment_size_y/col

        base_grid_vis_prim_path = "/World/Grid_Vis/Grid_Vis_"
        base_grid_vis_name="grid_vis_"

        
        wall_color_1 = np.array([1, 1, 1])
        wall_color_2 = np.array([0.01, 1, 0.01])
        # wall_color = wall_color_1
        trans_height = -0.05

        low_x = np.floor(row/2) - 1
        high_x = np.ceil(row/2) + 1
        low_y = np.floor(col/2) - 1
        high_y = np.ceil(col/2) + 1

        for i in range(row+1):
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
        for j in range(col+1):
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

    def create_grid_vis_cells(self):
        world = self.get_world()
        _, row, col, l_cell = self.calculate_rho_0()

        normalized_x_steps = actual_environment_size_x/row
        normalized_y_steps = actual_environment_size_y/col

        base_grid_vis_prim_path = "/World/Grid_Vis/Grid_Vis_"
        base_grid_vis_name="grid_vis_"

        
        wall_color_1 = np.array([0, 0, 0])
        wall_color_2 = np.array([0.01, 1, 0.01])
        # wall_color = wall_color_1
        trans_height = -0.05

        low_x = np.floor(row/2) - 1
        high_x = np.ceil(row/2) + 1
        low_y = np.floor(col/2) - 1
        high_y = np.ceil(col/2) + 1

        for i in range(row+1):
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
        for j in range(col+1):
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

    def setup_scene(self):
        world = self.get_world()
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
        self.create_robots(typeRobot,lineMode)

        # Create Lidars
        lidarsDrawLines = False
        lidarsDrawPoints = False
        self.create_lidars(lidarsDrawLines, lidarsDrawPoints)

        # Create Walls
        walls_color = np.array([1, 0.5, 0.5])
        # walls_visual_material = 
        # walls_physics_material = 
        # self.create_walls(walls_color)

        # Create Grid Visualisation
        self.create_grid_vis()

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

    async def setup_post_load(self):
        self.robots = [0 for _ in range(num_robots)]
        for robot_index in range(num_robots):
            base_robot_name="fancy_robot_"
            self.robots[robot_index] = self._world.scene.get_object(f"{base_robot_name}{robot_index:02}")
        self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)
        # Initialize our controller after load and the first reset
        self._Vel_controller = CoolController()
        self._WBP_controller = WheelBasePoseController(name="cool_controller",
                                                        open_loop_wheel_controller=
                                                            DifferentialController(name="simple_control",
                                                                                    wheel_radius=0.03, wheel_base=0.1125),
                                                    is_holonomic=False)
        # Made this to test difference if holonomic is set to true. To be tested
        self._WBPholo_controller = WheelBasePoseController(name="cool_controller",
                                                        open_loop_wheel_controller=
                                                            DifferentialController(name="simple_control",
                                                                                    wheel_radius=0.03, wheel_base=0.1125),
                                                    is_holonomic=True)
        
        # Can continue to implement holonomic controller if required.
        # https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.wheeled_robots/docs/index.html
        self._holo_controller = HolonomicController(name="holo_controller",
                                                    wheel_radius=np.array([0.04, 0.04, 0.04]),
                                                    wheel_positions=np.array([(-0.098043, 0.0006367, -0.050501),(0.05,-0.084525,-0.050501),(0.05,0.08569,-0.50501)]), 
                                                    wheel_orientations=np.array([(0,0,0,1), (0.866,0,0,-0.5), (0.866,0,0,0.5)]),
                                                    mecanum_angles=np.array([-90, -90, -90]),
                                                    wheel_axis=np.array([1,0,0]),
                                                    up_axis=np.array([0,0,1])
                                                    )

        # Changing inital v_rho0_i from [0,0,0] to become get_robot_vel() initially. 
        # Afterwards these values are overwritten in calculate_v_rho0_i() as intented
        for robvel in range(num_robots):
            # print("in loop")
            if (np.allclose(robs_initial_v_rho0_i[robvel], [0,0,0],atol=1e-08)):
                robs_initial_v_rho0_i[robvel] = self.get_robot_vel(robvel)
        return

    def get_lidar(self, robot_index):
        base_lidar_path = "/Lidar_"
        base_lidar_parent = "/World/Fancy_Robot_"

        depth = lidarInterface.get_linear_depth_data(f"{base_lidar_parent}{robot_index:02}/chassis{base_lidar_path}{robot_index:02}")
        azimuth = lidarInterface.get_azimuth_data(f"{base_lidar_parent}{robot_index:02}/chassis{base_lidar_path}{robot_index:02}")

        return np.array(depth), np.array(azimuth)

    def velocity_commands(self, v_ent_i, v_exp_i, v_int_i):
        v_i = v_ent_i + v_exp_i + v_int_i
        return v_i
    
    def get_robot_pos(self, robot_index):
        pos, ori = self.robots[robot_index].get_world_pose()
        return np.array(pos)
    
    def get_robot_ori(self, robot_index):    
        pos, ori = self.robots[robot_index].get_world_pose()
        return np.array(ori)

    def get_robot_ori_euler(self, robot_index):
        pos, ori = self.robots[robot_index].get_world_pose()
        w, x, y, z = ori

        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)


        roll_x_nrad, pitch_y_nrad, yaw_z_nrad = [roll_x % 2*np.pi,  pitch_y % 2*np.pi, yaw_z % 2*np.pi]
        roll_x_deg, pitch_y_deg, yaw_z_deg = [np.rad2deg(roll_x),np.rad2deg(pitch_y),np.rad2deg(yaw_z)]
        roll_x_ndeg, pitch_y_ndeg, yaw_z_ndeg = [roll_x_deg % 360,  pitch_y_deg % 360, yaw_z_deg % 360]

        return np.array(roll_x), np.array(pitch_y), np.array(yaw_z) # in radians -pi to pi

    def get_robot_vel(self, robot_index):
        vel = self.robots[robot_index].get_linear_velocity()
        return np.array(vel)

    def calculate_rho_0(self):
        grey_shape = self.greyscale(input_shape,h) # self.greyscale(shape_array_0,h) # shape_array_2
        nrow, ncol = grey_shape.shape

        # "n_cell is the number of black cells" - Supplementary Methods> 1 Additional Information of... > Page 3        n_cell = 0
        n_cell = 0
        for i in range(nrow):
            for j in range(ncol):
                if input_shape[i][j] == 0:
                    n_cell += 1
        l_cell = (((np.pi/4) * (num_robots/n_cell)) ** 0.5) * r_avoid

        # mid_point is rho_0
        mid_point = np.rint([nrow/2, ncol/2])

        # Returns the x and y indcies of the centerpoint of the greyscale shape, understood to be rho_0 
        # number rows and cols of greyscale shape passed so other functions can calculate and use.
        return mid_point, nrow, ncol, l_cell
    
    def get_robot_rho(self, robot_index):
        _, row, col, l_cell = self.calculate_rho_0()

        normalized_x_steps = actual_environment_size_x/row
        normalized_y_steps = actual_environment_size_y/col

        pos = self.get_robot_pos(robot_index)
        robot_rho = [-1,-1]
        for i in range(row):
            for j in range(col):
                if  (actual_environment_x_min + i*normalized_x_steps) < pos[0] and pos[0] < (actual_environment_x_min + (i+1)*normalized_x_steps):
                    if (actual_environment_y_min + j*normalized_y_steps) < pos[1] and pos[1] < (actual_environment_y_min + (j+1)*normalized_y_steps):
                        robot_rho = [i,j]
        # Returns the x and y indicies of the greyscale shape where the robot is currently located 
        if robot_rho == [-1,-1]:
            print("\n INDEX NOT PROPERLY ASSIGNED IN get_robot_rho(). robot_index:", robot_index,"\n")
        return robot_rho
            
    def get_robot_p_rho0(self, robot_index):
        _, row, col, l_cell = self.calculate_rho_0()

        normalized_x_steps = actual_environment_size_x/row
        normalized_y_steps = actual_environment_size_y/col

        robot_rho_indcies = self.get_robot_rho(robot_index)
        robot_p_rho_x = ((actual_environment_x_min + robot_rho_indcies[0]*normalized_x_steps) + (actual_environment_x_min + (robot_rho_indcies[0]+1)*normalized_x_steps)) /2
        robot_p_rho_y = ((actual_environment_y_min + robot_rho_indcies[1]*normalized_y_steps) + (actual_environment_y_min + (robot_rho_indcies[1]+1)*normalized_y_steps)) /2

        # Center point (in positional meters) of the cell robot i is currently occupying
        p_rho_i = [robot_p_rho_x, robot_p_rho_y, 0]

        return p_rho_i

    def get_robot_v_rho0(self, v_rho0, robot_index):
            v_rho0_j = v_rho0 - self.get_robot_vel(robot_index)
            return v_rho0_j

    def neighboring_i(self, robot_index):
        # # 1D List only containing Neighbour (Self not counted as Neighbour)
        N_list = []
        for j in range(num_robots):
            if robot_index != j:
                diff = ([a - b for a,b in zip(self.get_robot_pos(robot_index), self.get_robot_pos(j))])
                if (np.linalg.norm(diff) < r_sense):
                    N_list.append(j)

        return N_list

    def calculate_v_rho0_i(self, robot_index):
        """
        Calculate the local interpretation of the moving velocity of the entire shape.
        
        Parameters:
        # - p_rho_i: The position interpretation of robot i.
        N_i: The set of neighbors of robot i. And j is used to go through each neighboring robot
        # - neighbors_p_rho: The position interpretations of robot i's neighbors.
        # - neighbors_v_rho: The velocities of robot i's neighbors.
        - c1: Constant gain for the first term. 0 < c_1
        - alpha: Constant gain for the first term, 0 < alpha < 1.
        num_robots: Number of robots in scene
        i: Index of robot {1, ..., num_robot}
        v_rho_j: 
        
        Returns:
        - v_rho_i: The local interpretation of the moving velocity of the entire shape for robot i.
        """

        # Hardcode Assuming Parameters
        c_1 = 1.6       # 1.6 in Paper
        alpha = 0.8     # 0.8 in Paper

        N = self.neighboring_i(robot_index)

        term1 = (-1*c_1 / len(N)) * np.array(np.sum([np.multiply(np.sign([a - b for a,b in zip(self.get_robot_p_rho0(robot_index),self.get_robot_p_rho0(N[j]))]) , np.absolute([a - b for a,b in zip(self.get_robot_p_rho0(robot_index),self.get_robot_p_rho0(N[j]))]) ** alpha)
                        for j in range(len(N))], axis=0))

        term2 = (1 / len(N)) * np.array(np.sum([robs_initial_v_rho0_i[N[j]] for j in range(len(N))], axis=0))


        v_rho0_i = term1 + term2


        robs_initial_v_rho0_i[robot_index] = v_rho0_i 
        return v_rho0_i

    def greyscale(self, shape_array, h):
        grey_grid = np.copy(shape_array).astype(float)
        rows, cols = grey_grid.shape
        grey_grid = np.pad(grey_grid, [(1,1),(1,1)], mode='constant', constant_values=1)
        for hh in range(h):
            # Loop h times  
            for i in range(1,rows+1):
                for j in range(1,cols+1):
                    # For every cell in shape_array
                    local_min = np.min(grey_grid[i-1:i+2,j-1:j+2])
                    grey_grid[i,j] = min(local_min + 1/h, grey_grid[i, j])
            output_grey_grid = grey_grid[1:-1, 1:-1]
        
        return output_grey_grid
    
    def get_robot_xi_rho(self, robot_index): 
        # get color value of the cell the robot is in

        grey_grid = self.greyscale(input_shape, h)

        rho_i_x, rho_i_y = self.get_robot_rho(robot_index)
        xi_rho_i =  grey_grid[rho_i_x, rho_i_y]

        # Return color value of the cell the robot is in
        return xi_rho_i

    def get_robot_target_rho(self, robot_index):
        
        grey_grid = self.greyscale(input_shape, h)
        curr_rho_x, curr_rho_y = self.get_robot_rho(robot_index)

        curr_col = grey_grid[curr_rho_x, curr_rho_y]
    
        area = grey_grid[curr_rho_x-1:curr_rho_x+2, curr_rho_y-1:curr_rho_y+2]

        local_min = np.min(area)
        

        local_min_ind = np.unravel_index(area.argmin(), area.shape)
        # print("local min ind", local_min_ind)
        target_rho = [curr_rho_x + local_min_ind[0] -1, curr_rho_y + local_min_ind[1] -1]
        
        if (local_min == 0) & (np.array_equal(local_min_ind,[1,1])):
            print("\n",robot_index,"IN MIDDLE, STOP ! get_robot_target_rho()\n")

        return target_rho

    def get_robot_target_p_rho0(self, target_rho):
        _, row, col, l_cell = self.calculate_rho_0()

        normalized_x_steps = actual_environment_size_x/row
        normalized_y_steps = actual_environment_size_y/col

        target_p_rho_x = ((actual_environment_x_min + target_rho[0]*normalized_x_steps) + (actual_environment_x_min + (target_rho[0]+1)*normalized_x_steps)) /2
        target_p_rho_y = ((actual_environment_y_min + target_rho[1]*normalized_y_steps) + (actual_environment_y_min + (target_rho[1]+1)*normalized_y_steps)) /2

        # Center point (in positional meters) of the cell robot i is targeted to occupy
        target_p_rho_i = [target_p_rho_x, target_p_rho_y,0]
        
        return target_p_rho_i

    def shape_entering_velocity(self, robot_index):
        """
        Calculate the shape-entering velocity component for a robot.
        
        Parameters:
        robot_index: The index of robot i
        # - p_i: The current position of robot i (numpy array).
        # - p_t_i: The target position for robot i (numpy array).
        # - k_1: Constant gain (float).
        # - xi_rho_i: Grey level of that cell (float).
        # - v_rho0_i: The local interpretation of the moving velocity of the entire shape (numpy array).
        
        Returns:
        - v_ent_i: The shape-entering velocity component (numpy array).
        """
        # Hardcode
        k_1 = 10    # 10 in Paper

        # Selected the position of center of cell the robot is currently in
        p_i = self.get_robot_p_rho0(robot_index)
        
        p_t_i_ind = self.get_robot_target_rho(robot_index)
        p_t_i = self.get_robot_target_p_rho0(p_t_i_ind)

        xi_rho_i = self.get_robot_xi_rho(robot_index)
        v_rho0_i = self.calculate_v_rho0_i(robot_index)

        top = ([a - b for a,b in zip(p_t_i, p_i)])
        bottom = np.linalg.norm([a - b for a,b in zip(p_t_i, p_i)])
        unit_vector = np.divide(top, bottom, out=np.zeros_like(top), where=bottom!=0)
        
        firstpart = k_1 * xi_rho_i * unit_vector
        secondpart = v_rho0_i
        v_ent_i = ([a + b for a,b in zip(firstpart, secondpart)])
        
        return v_ent_i

    def mu_weight(self, arg):
        mu = 0
        r_check = 1
        if arg <= r_check: #r_avoid
            mu = (r_check/arg) - 1 #r_avoid
        else:
            mu = 0
        return mu

    def find_collision_points_index(self, robot_index):
        d, a = self.get_lidar(robot_index)
        wall_indices = []
        end_wall_index = []
        collision_points_index = []

        # Hardcode distance check
        r_check = 1
        
        # Copy relevant indicies into wall_indices 
        for i in range(len(d)):
            if d[i] < r_check:
                wall_indices.append(i)  
        
        # Check if next angle is sequential angle. 
        #   If not sequential then it's a gap in sensing, therefore 2 different walls. 
        #       Add the indicies of end of walls into end_wall_index
        expected_angle_difference = 0.00699
        for i in range(len(wall_indices)-1):
            if (a[wall_indices[i+1]] - a[wall_indices[i]] > expected_angle_difference):
                end_wall_index.append(wall_indices[i])


        # Find closest point of walls
        if len(end_wall_index) > 0:
            # Find closest point on every wall
            for i in range(len(end_wall_index)):
                if i == 0:
                    # Find the index of the minimum distance to robot from of all wall_indices of first wall and add it to collision_points_index
                    collision_points_index.append(np.argmin(np.array(d[wall_indices[0]:end_wall_index[0]]))) 
                else:
                    # Find the index of the minimum distance to robot from of all wall_indices of following walls + the index shift and add it to collision_points_index
                    collision_points_index.append(np.argmin(np.array(d[end_wall_index[i-1]+1:end_wall_index[i]])) + end_wall_index[i-1]+1) 
        # Find closest point of the 1 (one) wall
        else:
            # Find the index of the minimum distance to robot from of all wall_indices of the wall
            collision_points_index = wall_indices[np.argmin(np.array(d[wall_indices[0]:wall_indices[-1]]))] # -1 is last index
            
        return np.array(collision_points_index)

    def find_collision_points(self, robot_index):
        d, a = self.get_lidar(robot_index)
        curr_pos = self.get_robot_pos(robot_index)
        curr_ori = self.get_robot_ori_euler(robot_index)
        coll_ind = self.find_collision_points_index(robot_index)

        obstacle_pos = []

        for i in range(len(coll_ind)):
            obstacle_pos.append( [ float(curr_pos[0] + d[coll_ind[i]]*np.cos(curr_ori[2] + a[coll_ind[i]])) , float(curr_pos[1] + d[coll_ind[i]]*np.sin(curr_ori[2] + a[coll_ind[i]])) , 0.0 ] )

        return np.array(obstacle_pos)
 
    def interaction_velocity(self, robot_index):
        # Hardcode
        k_3 = 25    # 20,25,30 in Paper

        N = self.neighboring_i(robot_index)

        # Selected the position of center of cell the robot is currently in
        p_i = self.get_robot_pos(robot_index) # get_robot_p_rho0

        v_i = self.get_robot_vel(robot_index)

        O = self.find_collision_points(robot_index)

        p = []
        for j in range(len(N)+len(O)-1):
            if j < len(N):
                p.append(self.get_robot_pos(N[j])) # get_robot_p_rho0
            else:
                p.append(O[j-len(N)])

        p = np.array(p)
       

        # Includes O_i, the set of collision points of robot i.
        term1 = np.array(np.sum([
                                np.multiply(
                                    self.mu_weight( 
                                        np.linalg.norm( [a - b for a,b in zip(p_i, p[j])] ) 
                                    ) 
                                    , [a - b for a,b in zip(p_i, p[j])]
                                )
                        for j in range(len(N)+len(O)-1)], axis=0)
                )

        term2 = np.array(np.sum([
                            np.multiply(
                                (1 / len(N)) , [a - b for a,b in zip(v_i, self.get_robot_vel(N[j]))]
                            )
                        for j in range(len(N))], axis=0)
                        )
        

        firstterm = np.multiply(term1,k_3)
        v_int_i = ([a - b for a,b in zip(firstterm, term2)])

        return np.array(v_int_i)

    def psi_weight(self, arg):
        if arg <= 0:
            return 1
        elif arg < 1:
            return 0.5 * (1 + np.cos(np.pi * arg))
        else:
            return 0

    def in_shape_boundary(self, robot_index):

        grey_grid = self.greyscale(input_shape, h)
        curr_rho_x, curr_rho_y = self.get_robot_rho(robot_index)
        _, row, col, l_cell = self.calculate_rho_0()
        normalized_x_steps = actual_environment_size_x/row
        normalized_y_steps = actual_environment_size_y/col

        in_shape = False
        
        # # Radius r_sense means this number of cells 

        # Value too large for now
        # r_sense_cell_x = np.int(r_sense/normalized_x_steps)
        # r_sense_cell_y = np.int(r_sense/normalized_y_steps)
        
        # Using radius of 1 cell for now
        r_sense_cell_x = np.int(1)
        r_sense_cell_y = np.int(1)


        # neighbouring cells within radius r_sense 
        area = grey_grid[curr_rho_x-r_sense_cell_x:curr_rho_x+r_sense_cell_x+1, curr_rho_y-r_sense_cell_y:curr_rho_y+r_sense_cell_y+1]
        
        # Find max value of neighbouring cells within radius r_sense 
        local_max_color = np.max(area)

        # If max value is black, then inside shape boundary, else not inside shape boundary
        if local_max_color > 0:
            in_shape = False
        elif local_max_color == 0:
            in_shape = True
        
        return in_shape

    def occupied_cells(self):
        # return rho of occupied cells, considering radius of robot r_body
        
        occupied = set()  # Using a set to avoid duplicate entries

        _, row, col, l_cell = self.calculate_rho_0()
        normalized_x_steps = actual_environment_size_x/row
        normalized_y_steps = actual_environment_size_y/col
        x_numcells = row
        y_numcells = col
        x_cellsize = normalized_x_steps
        y_cellsize = normalized_y_steps

        # Center cell coordinates
        center_i = (x_numcells - 1) // 2 
        center_j = (y_numcells - 1) // 2

        r_body = 0.16 # Robot dimensions 0.160 x 0.135 x 0.260 [X,Y,Z]

        # Process each robot
        for n in range(num_robots):
            x_center, y_center, _ = self.get_robot_pos(n) # Center of each robot

            # Calculate the indices of the grid that intersect the bounding box of the robot
            min_i = max(0, int((x_center - r_body) / x_cellsize + center_i))
            max_i = min(x_numcells - 1, int((x_center + r_body) / x_cellsize + center_i))
            min_j = max(0, int((y_center - r_body) / y_cellsize + center_j))
            max_j = min(y_numcells - 1, int((y_center + r_body) / y_cellsize + center_j))

            # Iterate over the cells within the bounding box
            for i in range(min_i, max_i + 1):
                for j in range(min_j, max_j + 1):
                    # Calculate the real-world (x, y) coordinates of the center of each cell
                    cell_x_center = (i - center_i) * x_cellsize
                    cell_y_center = (j - center_j) * y_cellsize

                    # Check if any part of the robot covers the center of the cell. Factor sqrt(2) added to cover edge cases if robot perfectly between 4 cells
                    if np.sqrt((cell_x_center - x_center) ** 2 + (cell_y_center - y_center) ** 2) <= np.sqrt(2)*r_body: 
                        occupied.add((i, j))

        return list(occupied)    

    def neighbouring_cells(self, robot_index):
        
        in_shape = self.in_shape_boundary(robot_index)
        M_cells = []

        grey_grid = self.greyscale(input_shape, h)
        curr_rho_x, curr_rho_y = self.get_robot_rho(robot_index)
        _, row, col, l_cell = self.calculate_rho_0()
        normalized_x_steps = actual_environment_size_x/row
        normalized_y_steps = actual_environment_size_y/col

        # radius r_sense means this number of cells 
        # r_sense_cell = np.int(np.ceil(r_sense/normalized_x_steps))
        # r_sense_cell = np.int(r_sense/normalized_x_steps)
        r_sense_cell = np.int(1)

        # neighbouring cells within radius r_sense 
        area = grey_grid[curr_rho_x-r_sense_cell:curr_rho_x+r_sense_cell+1, curr_rho_y-r_sense_cell:curr_rho_y+r_sense_cell+1]

        
        occupied_cells = self.occupied_cells()
        occupied_x, occupied_y = [1,2]

        M_cells_debug = []

        if in_shape == False:
            for i in range(len(area)):
                for j in range(len(area)):
                    # If neighbouring cell within radius r_sense is black, append it to M_cells 
                    if area[i , j] == 0:
                        M_cells.append([curr_rho_x-r_sense_cell+i , curr_rho_y-r_sense_cell+j])

        elif in_shape == True:
            for i in range(len(area)):
                for j in range(len(area)):
                    # If neighbouring cell within radius r_sense is black...
                    if area[i , j] == 0:
                            # AND if neighbouring cell within radius r_sense is unoccupied, append it to M_cells
                            M_cells_debug.append([curr_rho_x-r_sense_cell+i , curr_rho_y-r_sense_cell+j])
                            if ((curr_rho_x-r_sense_cell+i, curr_rho_y-r_sense_cell+j) not in occupied_cells):
                                M_cells.append([curr_rho_x-r_sense_cell+i , curr_rho_y-r_sense_cell+j])
        
        return M_cells

    def shape_exploration_velocity(self, robot_index):
        # Hardcode
        sigma_1 = 10    # 10 or 5 in paper
        sigma_2 = 20    # 20 or 15 in paper
        k_2 = sigma_1 if (not self.in_shape_boundary(robot_index)) else sigma_2
        M_i_neigh = self.neighbouring_cells(robot_index)
        
        p_i = self.get_robot_pos(robot_index)

        # numerator = sum([k2 * psi(np.linalg.norm(pp - robot_position) / r_sense) * (pp - robot_position) for pp in M_i_neigh])
        top = sum([k_2 * self.psi_weight(np.linalg.norm(pp - p_i) / r_sense) * (pp - p_i) for pp in M_i_neigh])

        # denominator = sum([self.psi_weight(np.linalg.norm(pp - p_i) / r_sense) for pp in M_i_neigh])
        bottom = sum([self.psi_weight(np.linalg.norm(pp - p_i) / r_sense) for pp in M_i_neigh])

        # Compute the exploration velocity v_exp_i
        # v_exp_i = numerator / denominator if denominator != 0 else np.zeros_like(p_i)
        v_exp_i = top / bottom
        return v_exp_i

    def send_robot_actions(self, step_size):

    # Start Interaction:  
        # v_x, v_y, _ = self.interaction_velocity(0)
        # curr_rot = self.get_robot_ori_euler(0)
        # kf = 0.02
        # forward = kf * (((v_x ** 2) + (v_y ** 2)) ** 0.5)
        # ang = self.mod((np.rad2deg(np.arctan2(v_y,v_x) - curr_rot[2]) + 180) , 360) - 180
        # ang = np.deg2rad(ang)
        # ka = 0.8
        # angle = ka * (ang) 
        # self.robots[0].apply_action(self._Vel_controller.forward(command=[forward, angle]))
    # End Interaction

    # Start Entering:
        for robot_index in range(1): # num_robots
            v_x, v_y, _ = self.shape_entering_velocity(robot_index)
            kf = 0.02
            forward = kf * (((v_x ** 2) + (v_y ** 2)) ** 0.5)
            ka = 0.8
            curr_rot = self.get_robot_ori_euler(robot_index)

            ang = self.mod((np.rad2deg(np.arctan2(v_y,v_x) - curr_rot[2]) + 180) , 360) - 180
            ang = np.deg2rad(ang)
            angle = ka * (ang) # np.arctan2(v_y,v_x) - curr_rot[2]
            self.robots[robot_index].apply_action(self._Vel_controller.forward(command=[forward, angle]))      
    # End Entering 

        return