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

import numpy as np

import omni                                                     # Provides the core omniverse apis
import asyncio                                                  # Used to run sample asynchronously to not block rendering thread
from omni.isaac.range_sensor import _range_sensor               # Imports the python bindings to interact with lidar sensor
from pxr import UsdGeom, Gf, UsdPhysics                         # pxr usd imports used to create the cube

from omni.isaac.core.physics_context import PhysicsContext
import omni.isaac.core.utils.prims as prim_utils

from omni.isaac.examples.user_examples.git_isaac_sim.controllers import DiffDriveController, HoloController

from omni.isaac.wheeled_robots.controllers.wheel_base_pose_controller import WheelBasePoseController
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.wheeled_robots.controllers.holonomic_controller import HolonomicController

stage = omni.usd.get_context().get_stage()                      # Used to access Geometry
timeline = omni.timeline.get_timeline_interface()               # Used to interact with simulation
lidarInterface = _range_sensor.acquire_lidar_sensor_interface() # Used to interact with the LIDAR

from omni.isaac.examples.user_examples.git_isaac_sim.settings import num_robots, r_avoid, r_sense, input_shape, num_iterations 
from omni.isaac.examples.user_examples.git_isaac_sim.settings import actual_environment_size_x, actual_environment_size_y
from omni.isaac.examples.user_examples.git_isaac_sim.settings import actual_environment_x_min, actual_environment_x_max 
from omni.isaac.examples.user_examples.git_isaac_sim.settings import actual_environment_y_min, actual_environment_y_max
from omni.isaac.examples.user_examples.git_isaac_sim.grid import number_of_rows, number_of_columns
from omni.isaac.examples.user_examples.git_isaac_sim.grid import normalized_x_steps, normalized_y_steps
from omni.isaac.examples.user_examples.git_isaac_sim.grid import grey_grid, get_grid_rho, get_xi_rho, get_pos_of_rho
from omni.isaac.examples.user_examples.git_isaac_sim.environment import setup_environment
from omni.isaac.examples.user_examples.git_isaac_sim.robots import setup_robots
from omni.isaac.examples.user_examples.git_isaac_sim.util import log, performance_timestamp, mod

# Hardcode inital v_rho0_i
robs_initial_v_rho0_i = [[0,0,0] for _ in range(num_robots)]

class Main(BaseSample):
    
    def __init__(self) -> None:
        super().__init__()
        self.v_rho0_cache = {}
        return
    
    def setup_scene(self):
        self.world = self.get_world()
        setup_environment(self.world)
        setup_robots(self.world)

    async def setup_post_load(self):
        self._world = self.get_world()
        self.robots = [0 for _ in range(num_robots)]
        for robot_index in range(num_robots):
            base_robot_name="robot_"
            self.robots[robot_index] = self._world.scene.get_object(f"{base_robot_name}{robot_index:02}")
            
        self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)
        # Initialize our controller after load and the first reset
        self._Vel_controller = DiffDriveController()
        self._WBP_controller = WheelBasePoseController(name="wheel_base_pose_controller",
                                                        open_loop_wheel_controller=
                                                            DifferentialController(name="diff_controller",
                                                                                    wheel_radius=0.03, wheel_base=0.1125),
                                                    is_holonomic=False)
        # Made this to test difference if holonomic is set to true. To be tested
        self._WBPholo_controller = WheelBasePoseController(name="wheel_base_pose_controller",
                                                        open_loop_wheel_controller=
                                                            DifferentialController(name="diff_controller",
                                                                                    wheel_radius=0.03, wheel_base=0.1125),
                                                    is_holonomic=True)
        
        # Can continue to implement holonomic controller if required.
        # https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.wheeled_robots/docs/index.html
        self._holo_controller = HolonomicController(name="holonomic_controller",
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
        base_lidar_parent = "/World/Robot_"

        depth = lidarInterface.get_linear_depth_data(f"{base_lidar_parent}{robot_index:02}/chassis{base_lidar_path}{robot_index:02}")
        azimuth = lidarInterface.get_azimuth_data(f"{base_lidar_parent}{robot_index:02}/chassis{base_lidar_path}{robot_index:02}")

        return np.array(depth), np.array(azimuth)

    def velocity_commands(self, robot_index):
        raw_entering = self.shape_entering_velocity(robot_index)[0:2]
        raw_exploration = self.shape_exploration_velocity(robot_index)
        raw_interaction = self.interaction_velocity(robot_index)[0:2]
        
        # Hardcode
        entering_weight = 1
        exploration_weight = 2
        interaction_weight = 0.31 # 0.05

        applied_entering = np.multiply(entering_weight, raw_entering) 
        applied_exploration = np.multiply(exploration_weight, raw_exploration)
        applied_interaction = np.multiply(interaction_weight, raw_interaction)
        
        v = []
        v.append(applied_entering)
        # print(f"Test if weights works | Before: {v_ent_i} >> After: {np.multiply(v_ent_weight, v_ent_i)}")
        v.append(applied_exploration)
        v.append(applied_interaction)
        v_i = np.sum([v[j] for j in range(len(v))], axis=0)
        
        log("velocity_commands()", f"Rob: {robot_index} | Velocity commands: {np.round(v_i, decimals=2)}")
        log("raw", f"Ent:{np.round(raw_entering, decimals=2)} | Exp:{np.round(raw_exploration, decimals=2)} | Int:{np.round(raw_interaction, decimals=2)}", True)
        log("applied", f"Ent:{np.round(v[0], decimals=2)} | Exp:{np.round(v[1], decimals=2)} | Int:{np.round(v[2], decimals=2)}", True)

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
    
    def get_robot_rho(self, robot_index):
        pos = self.get_robot_pos(robot_index)
        robot_rho = get_grid_rho(pos)
        return robot_rho
    
    def get_robot_xi_rho(self, robot_index):
        rho_x, rho_y = self.get_robot_rho(robot_index)
        robot_xi_rho = get_xi_rho(rho_x, rho_y)
        return robot_xi_rho
            
    def get_robot_p_rho0(self, robot_index):
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

        if len(N) == 0:
            N.append(robot_index) 

        p_rho_i = self.get_robot_p_rho0(robot_index)
        
        p_rho_ = []
        for j in range(len(N)):
            p_rho_.append(self.get_robot_p_rho0(N[j]))

        term1 = (-1*c_1 / len(N)) * np.array(np.sum([np.multiply(np.sign([a - b for a,b in zip(p_rho_i, p_rho_[j])]) , np.absolute([a - b for a,b in zip(p_rho_i, p_rho_[j])]) ** alpha)
                        for j in range(len(N))], axis=0))
        
        term2 = (1 / len(N)) * np.array(np.sum([robs_initial_v_rho0_i[N[j]] for j in range(len(N))], axis=0))

        v_rho0_i = term1 + term2

        robs_initial_v_rho0_i[robot_index] = v_rho0_i 
        return v_rho0_i

    def get_robot_target_rho(self, robot_index):
        curr_rho_x, curr_rho_y = self.get_robot_rho(robot_index)
       
        area = grey_grid[curr_rho_x-1:curr_rho_x+2, curr_rho_y-1:curr_rho_y+2]
        
        local_min = np.min(area)
        
        local_min_ind = np.unravel_index(area.argmin(), area.shape)
        # print("local min ind", local_min_ind)
        target_rho = [curr_rho_x + local_min_ind[0] -1, curr_rho_y + local_min_ind[1] -1]
        
        if (local_min == 0) & (np.array_equal(local_min_ind,[1,1])):
            log("get_robot_target_rho()", f"robot {robot_index} IN MIDDLE, STOP !")

        return target_rho

    def get_robot_target_p_rho0(self, target_rho):
        target_p_rho_x = actual_environment_x_min + target_rho[0] * normalized_x_steps + normalized_x_steps / 2
        target_p_rho_y = actual_environment_y_min + target_rho[1] * normalized_y_steps + normalized_y_steps / 2

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
        distance, angle = self.get_lidar(robot_index)
        wall_indices = []

        # Hardcode distance check
        r_check = 1
        
        # Copy relevant indicies into wall_indices 
        for i in range(len(distance)):
            if distance[i] < r_check:
                wall_indices.append(i)  
        
        if len(wall_indices) <= 0:          # If no walls, no calculations needed return empty list, wall_indices = [] 
            log("find_collision_points_index()", f"No collision index found, wall_indicies: {wall_indices} output set to []:{np.array([])}")
            return np.array([])
        else:                               # If walls, do calculations
            end_wall_index = []
            collision_points_index = []

            # Check if next angle is sequential angle. 
            #   If not sequential then it's a gap in sensing, therefore 2 different walls. 
            #       Add the indicies of end of walls into end_wall_index
            expected_angle_difference = 0.00699
            for i in range(len(wall_indices)-1):
                if (angle[wall_indices[i+1]] - angle[wall_indices[i]] > expected_angle_difference):
                    end_wall_index.append(wall_indices[i]) 
            # print("end_wall_index",end_wall_index)  


            # Find closest point of walls
            if len(end_wall_index) > 0:
                # Find closest point on every wall
                for i in range(len(end_wall_index)):
                    if i == 0:
                        if distance[wall_indices[0]:end_wall_index[0]].size: # Added to solve "attempt to get argmin of an empty sequence" error which happens sometimes
                            # Find the index of the minimum distance to robot from of all wall_indices of first wall and add it to collision_points_index
                            collision_points_index.append(np.argmin(np.array(distance[wall_indices[0]:end_wall_index[0]]))) 
                    else:
                        # Find the index of the minimum distance to robot from of all wall_indices of following walls + the index shift and add it to collision_points_index
                        collision_points_index.append(np.argmin(np.array(distance[end_wall_index[i-1]+1:end_wall_index[i]])) + end_wall_index[i-1]+1) 
            # Find closest point of the 1 (one) wall
            else:
                # Find the index of the minimum distance to robot from of all wall_indices of the wall
                collision_points_index = wall_indices[np.argmin(np.array(distance[wall_indices[0]:wall_indices[-1]]))] # -1 is last index
                
            # print("collision_points_index", collision_points_index) 
            return np.array(collision_points_index)
        
    def find_collision_points(self, robot_index):
        coll_ind = np.array(self.find_collision_points_index(robot_index))
        obstacle_pos = []

        # print(f"find_collision_points()       | coll_ind: {coll_ind} | coll_ind.size {coll_ind.size}")
        if not coll_ind.size:           # If no walls, return empty array # Same as if len(coll_ind) <= 0
            log("find_collision_points()", "No collision points found, output set to []")
            return np.array(obstacle_pos)
        
        # If walls only then do calculations
        distance, angle = self.get_lidar(robot_index)
        curr_pos = self.get_robot_pos(robot_index)
        curr_ori = self.get_robot_ori_euler(robot_index)

        if coll_ind.size == 1: # Added to prevent error where sometimes coll_ind does exist but is a float instead of an array of size 1
            obstacle_pos.append( [ float(curr_pos[0] + distance[coll_ind]*np.cos(curr_ori[2] + angle[coll_ind])) , float(curr_pos[1] + distance[coll_ind]*np.sin(curr_ori[2] + angle[coll_ind])) , 0.0 ] )
        else:
            for i in range(coll_ind.size): # Should work same as for i in range(len(coll_ind)):
                obstacle_pos.append( [ float(curr_pos[0] + distance[coll_ind[i]]*np.cos(curr_ori[2] + angle[coll_ind[i]])) , float(curr_pos[1] + distance[coll_ind[i]]*np.sin(curr_ori[2] + angle[coll_ind[i]])) , 0.0 ] )
                
        log("find_collision_points()", f"Obstacles:\n{np.array(obstacle_pos).round(decimals=2)}")
        
        return np.array(obstacle_pos)
 
    def interaction_velocity(self, robot_index):
        
        N = self.neighboring_i(robot_index)
        O = self.find_collision_points(robot_index)
        
        if (len(O) <= 0) and (len(N) <= 0):     # Neither N nor O so no calculations needed return [0.0, 0.0, 0.0]
            log("interaction_velocity()", f"Neither Neighbours NOR Collision points, v_int_i set to [0.0, 0.0, 0.0]")
            v_int_i = [0.0, 0.0, 0.0]
            return np.array(v_int_i)
        
        # At least one of N exist O so calculate values
        # Hardcode
        k_3 = 25    # 20,25,30 in Paper
        
        p = []
        
        if (len(N) <= 0):               # If no N, only calculate term1 and set term2 to [0.0, 0.0, 0.0]
            log("interaction_velocity()", f"No Neighbours, only Collision points found, v_int_i term2 set to [0.0, 0.0, 0.0]")
            length_sum = len(O) # used to be len(O)-1 
            for j in range(length_sum):
                p.append(O[j]) 
            term2 = [0.0, 0.0, 0.0]
        else:                           # If N, calculate both term1 and term2
            if (len(O) <= 0):               # If no O and only N, change length of sum, length_sum, and collision point positions, p.
                log("interaction_velocity()", f"Only Neighbours, no Collision points")
                length_sum = len(N)-1
                for j in range(length_sum):
                    p.append(self.get_robot_pos(N[j])) 
            else:                           # If both O and N, change length of sum, length_sum, and collision point positions, p.
                log("interaction_velocity()", f"Neighbours AND Collision points found")
                length_sum = len(N)+len(O)-1
                for j in range(length_sum):
                    if j < len(N):
                        p.append(self.get_robot_pos(N[j]))
                    else:
                        p.append(O[j-len(N)])

            v_i = self.get_robot_vel(robot_index)

            term2 = np.array(np.sum([
                                np.multiply(
                                    (1 / len(N)) , [a - b for a,b in zip(v_i, self.get_robot_vel(N[j]))]
                                )
                            for j in range(len(N))], axis=0)
                            )

        p = np.array(p)

        if len(p) <= 0:         # If error with find_collision_points_index, set term1 to [0.0, 0.0, 0.0]
            log("interaction_velocity()", f"Error with calculating term1, v_int_i term1 set to [0.0, 0.0, 0.0]")
            term1 = [0.0, 0.0, 0.0]
        else:                   # If no error calculate term1
            # Selected the position of center of cell the robot is currently in
            p_i = self.get_robot_pos(robot_index) # get_robot_p_rho0

            # print(f"len_sum: {length_sum} | p:{p} ")
            # print(f"mu {self.mu_weight(np.linalg.norm( [a - b for a,b in zip(p_i, p[0])] ))} | subtraction: {[a - b for a,b in zip(p_i, p[0])]}")
            term1 = np.array(np.sum([
                                    np.multiply(
                                        self.mu_weight( 
                                            np.linalg.norm( [a - b for a,b in zip(p_i, p[j])] ) 
                                        ) 
                                        , [a - b for a,b in zip(p_i, p[j])]
                                    )
                            for j in range(length_sum)], axis=0)
                    )
        
        firstterm = np.multiply(term1, k_3)
        v_int_i = ([a - b for a,b in zip(firstterm, term2)])
        # print(f"                              | v_int_i: {np.round(v_int_i, decimals=2)} |")
        # print(f"                              | term1: {np.round(term1, decimals=2)} | term2: {np.round(term2, decimals=2)}")
        return np.array(v_int_i)

    def psi_weight(self, arg):
        if arg <= 0:
            return 1
        elif arg < 1:
            return 0.5 * (1 + np.cos(np.pi * arg))
        else:
            return 0

    def in_shape_boundary(self, robot_index):
        # if: robot i is close to the boundary of the shape so that there are non-black cells within the sensing radius r_sense
            # return False
        # else:
            # return True 

        curr_rho_x, curr_rho_y = self.get_robot_rho(robot_index)
    
        in_shape = False
        
        # # Radius r_sense means this number of cells 

        # Real value too large for now
        # r_sense_cell_x = np.int(r_sense/normalized_x_steps)
        # r_sense_cell_y = np.int(r_sense/normalized_y_steps)
        # print(f"r_sense_cell x:{r_sense_cell_x} y:{r_sense_cell_x}")
        
        # Hardcode - using radius of cell_sense for now
        cell_sense = 1
        r_sense_cell_x = np.int(cell_sense)
        r_sense_cell_y = np.int(cell_sense)
        # print(f"r_sense_cell x:{r_sense_cell_x} y:{r_sense_cell_x}")

        # neighbouring cells within radius r_sense 
        area = grey_grid[curr_rho_x-r_sense_cell_x:curr_rho_x+r_sense_cell_x+1, curr_rho_y-r_sense_cell_y:curr_rho_y+r_sense_cell_y+1]
        
        # Find max value of neighbouring cells within radius r_sense 
        local_max_color = np.max(area)

        # If max value is black, then inside shape boundary, else not inside shape boundary
        if local_max_color > 0:
            in_shape = False
        elif local_max_color == 0:
            in_shape = True
        
        log("in_shape_boundary()", f"Rob: {robot_index} | in_shape?: {in_shape} | max_color: {np.round(local_max_color,2)} | r_sense_cells x:{r_sense_cell_x} y:{r_sense_cell_x} ")
        log("", f" | area:\n{np.round(area,2)}\n")
        return in_shape, r_sense_cell_x, r_sense_cell_y, area

    def occupied_cells(self):
        # return rho of occupied cells, considering radius of robot r_body
        
        occupied = set()  # Using a set to avoid duplicate entries

        x_numcells = number_of_rows
        y_numcells = number_of_columns
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
                    # Paper wants to count occupied if cell_center within r_avoid/2 of any robot. If want cells occupied by body: np.sqrt(2)*r_body
                    if np.sqrt((cell_x_center - x_center) ** 2 + (cell_y_center - y_center) ** 2) <= r_avoid/2: 
                        occupied.add((i, j))

        log("occupied_cells()", f"Occupied cells:\n {list(occupied)}\n")
        return list(occupied)
    
    def neighbouring_cells(self, robot_index):
        in_shape, r_sense_cell_x, r_sense_cell_y, area  = self.in_shape_boundary(robot_index)
        M_cells = []

        curr_rho_x, curr_rho_y = self.get_robot_rho(robot_index)

        if in_shape == False:
            for i in range(len(area)):
                for j in range(len(area)):
                    # If neighbouring cell within radius r_sense is black, append it to M_cells 
                    if area[i , j] == 0:
                        M_cells.append([curr_rho_x-r_sense_cell_x+i , curr_rho_y-r_sense_cell_y+j])

        elif in_shape == True:
            
            occupied_cells = self.occupied_cells()

            M_cells_debug = []

            for i in range(len(area)):
                for j in range(len(area)):
                    # If neighbouring cell within radius r_sense is black...
                    if area[i , j] == 0:
                            # AND if neighbouring cell within radius r_sense is unoccupied, append it to M_cells
                            M_cells_debug.append([curr_rho_x-r_sense_cell_x+i , curr_rho_y-r_sense_cell_y+j])
                            if ((curr_rho_x-r_sense_cell_x+i, curr_rho_y-r_sense_cell_y+j) not in occupied_cells):
                                M_cells.append([curr_rho_x-r_sense_cell_x+i , curr_rho_y-r_sense_cell_y+j])

        log("neighbouring_cells()", f"Rob: {robot_index} | include occupied?: {in_shape} |  M_cells: {M_cells}")
        # print("M_cells_Debug:\n",M_cells_debug)
        return M_cells
    
    def shape_exploration_velocity(self, robot_index):
        # To optimize code, first check if any neighboring valid cells. If found, only then set parameters, call nessesary functions, and perform any calculations
        
        M_i_neigh = self.neighbouring_cells(robot_index)
        
        if len(M_i_neigh) <= 0:
            log("shape_exploration_velocity()", f"No valid cells found, v_exp_i set to [0.0, 0.0]")
            v_exp_i = [0.0, 0.0]
            return v_exp_i
        
        # Hardcode
        sigma_1 = 10    # 10 or 5 in paper
        sigma_2 = 20    # 20 or 15 in paper
        in_shape, _, _, _  = self.in_shape_boundary(robot_index)
        k_2 = sigma_1 if (not in_shape) else sigma_2
    
        p_rhos = []
        for j in range(len(M_i_neigh)):
            p_rhos.append(get_pos_of_rho(M_i_neigh[j]))
        
        p_i = self.get_robot_pos(robot_index)[0:2]
        
        top = sum([k_2 * np.multiply(
                            self.psi_weight(np.linalg.norm([a - b for a,b in zip(p_rhos[rho], p_i)]) / r_sense ) 
                            , ([a - b for a,b in zip(p_rhos[rho], p_i)])    
                        ) 
                for rho in range(len(M_i_neigh))])
        
        bottom = sum([self.psi_weight( np.linalg.norm([a - b for a,b in zip(p_rhos[rho], p_i)]) / r_sense ) 
                    for rho in range(len(M_i_neigh))])
        
        v_exp_i = np.divide(np.array(top), np.array(bottom), out=np.zeros_like(top), where=bottom!=0.0)
         
        return v_exp_i

    def send_robot_actions(self, step_size):
    ##### Start of Print Test Area #####

    # Start Velocity command:

        for robot_index in range(num_robots): # num_robots
            v_x, v_y = self.velocity_commands(robot_index) 
            curr_rot = self.get_robot_ori_euler(robot_index)

            kf = 0.02
            forward = kf * (((v_x ** 2) + (v_y ** 2)) ** 0.5)
            ka = 0.8
            ang = mod((np.rad2deg(np.arctan2(v_y,v_x) - curr_rot[2]) + 180) , 360) - 180
            ang = np.deg2rad(ang)
            angle = ka * (ang) # np.arctan2(v_y,v_x) - curr_rot[2]
            self.robots[robot_index].apply_action(self._Vel_controller.forward(command=[forward, angle]))

    # End Velocity command


    # Start Exploration:

        # for robot_index in range(1):  
        #     performance_timestamp("") # Reset the timestamp time for time stamp measure section

        #     exp_v_x, exp_v_y = self.shape_exploration_velocity(robot_index)
        #     ent_v_x, ent_v_y, _ = self.shape_entering_velocity(robot_index)
        #     v_x = exp_v_x + ent_v_x
        #     v_y = exp_v_y + ent_v_y

        #     performance_timestamp("shape exploration")

        #     kf = 0.02
        #     forward = kf * (((v_x ** 2) + (v_y ** 2)) ** 0.5)
        #     ka = 0.8
        #     curr_rot = self.get_robot_ori_euler(robot_index)
            
        #     performance_timestamp("robot ori euler")            
            
        #     ang = mod((np.rad2deg(np.arctan2(v_y,v_x) - curr_rot[2]) + 180) , 360) - 180
        #     ang = np.deg2rad(ang)
        #     angle = ka * (ang) # np.arctan2(v_y,v_x) - curr_rot[2]
        #     self.robots[robot_index].apply_action(self._Vel_controller.forward(command=[forward, angle]))
            
        #     performance_timestamp("apply robot action")

    # End Exploration:

    # Start Interaction: 
     
        # for robot_index in range(num_robots):
        #     v_x, v_y, _ = self.interaction_velocity(robot_index)
        #     curr_rot = self.get_robot_ori_euler(robot_index)
        #     kf = 0.02
        #     forward = kf * (((v_x ** 2) + (v_y ** 2)) ** 0.5)
        #     ang = mod((np.rad2deg(np.arctan2(v_y,v_x) - curr_rot[2]) + 180) , 360) - 180
        #     ang = np.deg2rad(ang)
        #     ka = 0.8
        #     angle = ka * (ang) 
        #     self.robots[robot_index].apply_action(self._Vel_controller.forward(command=[forward, angle]))
    
    # End Interaction

    # Start Entering:

        # for robot_index in range(num_robots): # 
        #     performance_timestamp("") # Reset the timestamp time for time stamp measure section

        #     v_x, v_y, _ = self.shape_entering_velocity(robot_index)

        #     performance_timestamp("shape entering")

        #     kf = 0.02
        #     forward = kf * (((v_x ** 2) + (v_y ** 2)) ** 0.5)
        #     ka = 0.8
        #     curr_rot = self.get_robot_ori_euler(robot_index)
            
        #     performance_timestamp("robot ori euler")
            
        #     # a = targetA - sourceA
        #     # Version 1:
        #     # a = np.arctan2(v_y,v_x) - curr_rot[2]
        #     # if a > np.pi:
        #     #     a -= 2*np.pi 
        #     # if a < -np.pi:
        #     #     a += 2*np.pi
        #     # Version 2:
        #     # Custom mod function: mod(a, n) -> a - floor(a/n) * n
        #     ang = mod((np.rad2deg(np.arctan2(v_y,v_x) - curr_rot[2]) + 180) , 360) - 180
        #     ang = np.deg2rad(ang)
        #     angle = ka * (ang) # np.arctan2(v_y,v_x) - curr_rot[2]
        #     # if robot_index == 2:
        #         # print("Rob", robot_index, ":\n Target angle", np.arctan2(v_y,v_x).round(decimals=2),"rads", np.rad2deg(np.arctan2(v_y,v_x)).round(decimals=2),"deg","\n Current angle:", np.array(curr_rot[2]).round(decimals=2),"rads", np.rad2deg(curr_rot[2]).round(decimals=2),"deg", "\n Difference:", np.array((np.arctan2(v_y,v_x) - curr_rot[2])).round(decimals=2), "rads", np.rad2deg(np.arctan2(v_y,v_x) - curr_rot[2]).round(decimals=2),"deg")
        #         # print("Rob", robot_index, "Shp Ent Vel:", v_x, v_y)
        #         # print("Rob", robot_index, "PD ctrllr:", forward, angle)

        #     # b = self.in_shape_boundary(robot_index) # in_shape_boundary() works
        #     # # nc = self.neighbouring_cells(robot_index) # neighbouring_cells() if in_shape_boundary() == False works
        #     # oc = self.occupied_cells() # occupied_cells() works
        #     # # nc = self.neighbouring_cells(robot_index) # neighbouring_cells() if in_shape_boundary() == True works
        #     # nc = self.neighbouring_cells(robot_index) # neighbouring_cells() works
        #     self.robots[robot_index].apply_action(self._Vel_controller.forward(command=[forward, angle]))      

        #     performance_timestamp("apply action")
         
    # End Entering 
        return
    