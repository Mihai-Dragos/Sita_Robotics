from omni.isaac.examples.user_examples.git_isaac_sim.shapes import *

##### Research Paper Parameters #####
r_avoid = 0.4 # 1.5 in Paper 				# Hardcode avoid radius of robots, used for calculating l_cell
r_sense = 0.7 # 2.5 in Paper				# Hardcode sense radius of robots, used for calculating neighboring_i() and neighbouring_cells()
r_check = 1.2 # equal to r_avoid in Paper		# Hardcode check radius for walls

entering_weight = 0.9							# Hardcode velocity commands weigths
exploration_weight = 4.0					# | 	2.5 for floorplan # 2 for video
interaction_weight = 0.35					# | 	0.4 for floorplan # 0.31 for video # 0.05

c_1 = 1.6       # 1.6 in Paper				# Hardcode parameters for calculate_v_rho0_i()
alpha = 0.8     # 0.8 in Paper				# |
k_1 = 10		# 10 in Paper				# Hardcode parameter shape_entering_velocity()
k_3 = 25    	# 20,25,30 in Paper			# Hardcode parameter interaction_velocity()

forward_gain = 0.02							# Hardcode parameters for differential drive
angle_gain = 0.8							# |
######################################


##### Visualisation Parameters #####
show_vel_spheres = True						# Show Velocity command contributions visually?
#####################################


##### Robot Parameters #####
num_robots = 3 					# Number of robots
lineMode = True					# True: Robots spawn in a line; False: Robots spawn in a grid
typeRobot = 1       			# 1 -> Jetbot, 2 -> Kaya
lidarsDrawLines = False			# Show lidar lines?
lidarsDrawPoints = False		# Show lidar points of surface?
############################


##### Environment Parameters #####
input_shape = shape_array_floorplan		# Navigation shape that robots should explore.
# h = np.ceil((n_cell ** 0.5)/2)			# Calculated number of iterations for greyscale()
h = 4 #2 								# Hardcode Number of iterations for greyscale()

# actual_environment_x_min = -1.4		# Hardcode actual enviroment size (of biggest room) in meters
# actual_environment_x_max = 1.4
# actual_environment_y_min = -0.4
# actual_environment_y_max = 0.8

a_e_v_amount = 3 #1.5					# Hardcode set enviroment size in meters to test shape_entering_velocity()
actual_environment_x_min = -a_e_v_amount
actual_environment_x_max = a_e_v_amount
actual_environment_y_min = -a_e_v_amount
actual_environment_y_max = a_e_v_amount
actual_environment_size_x = actual_environment_x_max - actual_environment_x_min
actual_environment_size_y = actual_environment_y_max - actual_environment_y_min
##################################

##### Utility Parameters #####
LOG_CONTEXT_SPACE = 29
MEASURE_PERFORMANCE = True
###############################