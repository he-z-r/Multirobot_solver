import numpy as np


def array2digit(ap_arr, num_cells):
    d = 0
    m = 1
    for i in ap_arr:
        d += i * m
        m *= num_cells
    return d


def digit2array(state, num_cells, num_robots):
    ap_arr = np.zeros(num_robots, dtype=int)
    for i in range(num_robots):
        ap_arr[i] = state % num_cells
        state = state // num_cells
    return ap_arr


def array2coord(states_arr, num_cells, num_robots, num_cols):
    coord_tot = []
    for i in states_arr:
        ap_arr = digit2array(i, num_cells, num_robots);
        coord = []
        for j in ap_arr:
            coord.append([j // num_cols, j % num_cols])
        coord_tot.append(coord)
    return coord_tot
