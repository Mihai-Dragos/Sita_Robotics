import numpy as np

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

shape_array_2 = np.array([[1, 1, 1, 1, 1, 1, 1],
                        [1, 2/3, 2/3, 2/3, 2/3, 2/3, 1],
                        [1, 2/3, 1/3, 1/3, 1/3, 2/3, 1],
                        [1, 2/3, 1/3, 0, 1/3, 2/3, 1],
                        [1, 2/3, 1/3, 1/3, 1/3, 2/3, 1],
                        [1, 2/3, 2/3, 2/3, 2/3, 2/3, 1],
                        [1, 1, 1, 1, 1, 1, 1]])
