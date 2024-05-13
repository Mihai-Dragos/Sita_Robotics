import numpy as np

from omni.isaac.examples.user_examples.git_isaac_sim.shapes import *

# Number of robots
num_robots = 3

# Hardcode avoid radius of robots, used for calculating l_cell
r_avoid = 0.4 # 1.5 in Paper
# Hardcode sense radius of robots, used for calculating neighboring_i() and neighbouring_cells()
r_sense = 0.7 # 2.5 in Paper

# Hardcode check radius for walls
r_check = 1 # is equal to r_avoid in paper

# Navigation shape that robots should explore.
input_shape = shape_array_floorplan

# Hardcode Number of iterations
h = 4 #2 # num_iterations
# h = np.ceil((n_cell ** 0.5)/2)

# Hardcode velocity commands weigths


# Hardcode parameters for calculate_v_rho0_i()
c_1 = 1.6       # 1.6 in Paper
alpha = 0.8     # 0.8 in Paper

# Hardcode parmeters shape_entering_velocity()


# Hardcode parameters interaction_velocity()



# Hardcode actual enviroment size (of biggest room) in meters
# actual_environment_x_min = -1.4
# actual_environment_x_max = 1.4
# actual_environment_y_min = -0.4
# actual_environment_y_max = 0.8
# Hardcode set enviroment size in meters to test shape_entering_velocity()
a_e_v_amount = 3 #1.5
actual_environment_x_min = -a_e_v_amount
actual_environment_x_max = a_e_v_amount
actual_environment_y_min = -a_e_v_amount
actual_environment_y_max = a_e_v_amount

actual_environment_size_x = actual_environment_x_max - actual_environment_x_min
actual_environment_size_y = actual_environment_y_max - actual_environment_y_min

LOG_CONTEXT_SPACE = 29
MEASURE_PERFORMANCE = True