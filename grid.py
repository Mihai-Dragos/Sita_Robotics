import numpy as np

from settings import input_shape, h, num_robots, r_avoid
from settings import actual_environment_size_x, actual_environment_size_y, actual_environment_x_min, actual_environment_x_max, actual_environment_y_min, actual_environment_y_max
from util import log
from settings import show_log_grid
from environment import create_cube

def greyscale(shape_array, h):
    grey_grid = np.copy(shape_array).astype(float)
    rows, cols = grey_grid.shape
    grey_grid = np.pad(grey_grid, [(1,1),(1,1)], mode='constant', constant_values=1)
    for _ in range(h):
        # Loop number of iteration (h) times  
        for i in range(1,rows+1):
            for j in range(1,cols+1):
                # For every cell in shape_array
                local_min = np.min(grey_grid[i-1:i+2,j-1:j+2])
                grey_grid[i,j] = min(local_min + 1/h, grey_grid[i, j])
        output_grey_grid = grey_grid[1:-1, 1:-1]
    
    return output_grey_grid

def reverse_greyscale(shape_array):
    rev_grey_grid = np.copy(shape_array).astype(float)
    rows, cols = rev_grey_grid.shape
    rev_grey_grid = np.pad(rev_grey_grid, [(1,1),(1,1)], mode='constant', constant_values=0)
    h = np.int32((rows+cols)/2 - 1)
    for _ in range(h):
        for i in range(1,rows+1):
            for j in range(1,cols+1):
                local_max = np.max(rev_grey_grid[i-1:i+2,j-1:j+2])
                rev_grey_grid[i,j] = max(local_max - 2/h, rev_grey_grid[i,j])
        output_rev_grey_grid = rev_grey_grid[1:-1, 1:-1]
    
    return output_rev_grey_grid


def calculate_rho_0():
    grey_shape = greyscale(input_shape, h) # grayscale(shape_array_0,h) # shape_array_2
    number_of_rows, number_of_columns = grey_shape.shape

    # mid_point is rho_0
    mid_point = np.rint([number_of_rows/2, number_of_columns/2])

    # Returns the x and y indcies of the centerpoint of the greyscale shape, understood to be rho_0 
    # number rows and cols of greyscale shape passed so other functions can calculate and use.
    return grey_shape, mid_point, number_of_rows, number_of_columns

grey_grid, mid_point, number_of_rows, number_of_columns = calculate_rho_0()
rev_grey_grid = reverse_greyscale(input_shape)
# print(np.round(rev_grey_grid,decimals=2))
# grey_grid = rev_grey_grid
if show_log_grid:
    log("grid.py", f"grey_grid:\n{np.array(grey_grid).round(decimals=2)}")
normalized_x_steps = actual_environment_size_x/number_of_rows
normalized_y_steps = actual_environment_size_y/number_of_columns

def get_n_cell_l_cell():
    # "n_cell is the number of black cells" - Supplementary Methods> 1 Additional Information of... > Page 3        n_cell = 0
    n_cell = 0
    for i in range(number_of_rows):
        for j in range(number_of_columns):
            if input_shape[i][j] == 0:
                n_cell += 1
    l_cell = (((np.pi/4) * (num_robots/n_cell)) ** 0.5) * r_avoid
    return n_cell, l_cell

n_cell, l_cell = get_n_cell_l_cell()

def get_xi_rho(rho_x, rho_y) -> float: 
    # Get color value of the cell
    xi_rho_i =  grey_grid[rho_x, rho_y]
    return xi_rho_i # Return color value of the cell

def get_grid_rho(pos):
    '''Returns the x and y indicies of the grid cell where the pos is '''
    if not (actual_environment_x_min <= pos[0] <= actual_environment_x_max and 
            actual_environment_y_min <= pos[1] <= actual_environment_y_max):
        print("\n Position out of grid bounds in get_grid_rho(). pos:", pos,"\n")

    cell_x = int((pos[0] - actual_environment_x_min) // normalized_x_steps)
    cell_y = int((pos[1] - actual_environment_y_min) // normalized_y_steps)
    return [cell_x, cell_y]

def get_pos_of_rho(rho):
    # Coordinates of the center cell
    center_x = (number_of_rows - 1) / 2
    center_y = (number_of_columns - 1) / 2

    # Calculate the difference in cell indices from the center cell
    delta_x = rho[0] - center_x
    delta_y = rho[1] - center_y
    
    # Calculate the coordinates of the center of the cell (i, j)
    center_of_cell_x = delta_x * normalized_x_steps
    center_of_cell_y = delta_y * normalized_y_steps
    
    cell_pos = [center_of_cell_x, center_of_cell_y]
    # print(f"Cell pos: {cell_pos}")
    return cell_pos

def get_wall_color(row, column):
    cell_value = grey_grid[row, column]
    return np.array([cell_value, cell_value, cell_value])

def create_grid_vis(world):
    base_grid_vis_prim_path = "Grid_Vis/Grid_Vis_"
    base_grid_vis_name="grid_vis_"

    for row_index in range(number_of_rows):
        for column_index in range(number_of_columns):
            create_cube(world, base_grid_vis_prim_path, base_grid_vis_name, 
                        row_index * number_of_columns + column_index,
                        np.array([actual_environment_x_min + (row_index + 0.5) * normalized_x_steps, 
                                  actual_environment_y_min + (column_index + 0.5) * normalized_y_steps, 
                                  -0.04999]), 
                        np.array([normalized_x_steps, normalized_y_steps, 0.1]), 
                        get_wall_color(row_index, column_index))