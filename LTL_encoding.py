import numpy as np
import itertools
import State_encoding


def cltl_conversion(ap, c, num_robots, num_cells):
    # The parameters (ap, c) represent a cLTL logic formula
    # ap: np.array, which is the atomic proposition given in an array of cells, c: int, sys_prog: an empty set
    # First generate outer logic
    robots = np.arange(num_robots, dtype=int)
    ap_tot = np.arange(num_cells, dtype=int)
    states = set()
    temp_pos = np.zeros(num_robots, dtype=int)
    for r in itertools.combinations(robots, c):
        # this loop: the robots which need to satisfy constraints are fixed
        rest_of_robots = np.copy(robots)
        rest_of_robots = np.delete(rest_of_robots, np.asarray(r, dtype=int))
        for l in itertools.permutations(ap, c):
            # this loop: locations of the chosen robots which need to satisfy constraints are fixed
            for i in range(c):
                temp_pos[r[i]] = l[i]
            for rest_l in itertools.permutations(ap_tot, len(rest_of_robots)):
                # this loop: locations of the unchosen robots are fixed (doesn't check validity of the locations,
                # because it's already ensured by system dynamics)
                for rest_i in range(num_robots - c):
                    temp_pos[rest_of_robots[rest_i]] = rest_l[rest_i]
                if len(np.unique(temp_pos)) == len(temp_pos):
                    states.add(State_encoding.array2digit(temp_pos, num_cells))
    ltl_formula = '( loc='
    for state in states:
        ltl_formula = ltl_formula + str(state) + ' || loc='
        # print(State_encoding.digit2array(state, num_cells, num_robots))
    # print(len(states))
    ltl_formula = ltl_formula[:-7]
    ltl_formula = ltl_formula + ')'
    return ltl_formula


def ltl_negate(ltl_formula):
    return '!' + ltl_formula


def ltl_conversion_ap(ap, robot_index, num_robots, num_cells):
    robots = np.arange(num_robots, dtype=int)
    ap_tot = np.arange(num_cells, dtype=int)
    states = set()
    temp_pos = np.zeros(num_robots, dtype=int)
    for l in ap:
        temp_pos[robot_index] = l
        rest_of_robots = np.copy(robots)
        rest_of_robots = np.delete(rest_of_robots, robot_index)
        rest_of_ap = np.copy(ap_tot)
        rest_of_ap = np.delete(rest_of_ap, l)
        for rest_l in itertools.permutations(rest_of_ap, num_robots-1):
            for i in range(num_robots-1):
                temp_pos[rest_of_robots[i]] = rest_l[i]
            states.add(State_encoding.array2digit(temp_pos, num_cells))
    ltl_formula = '( loc='
    for state in states:
        ltl_formula = ltl_formula + str(state) + ' || loc='
        # print(State_encoding.digit2array(state, num_cells, num_robots))
    # print(len(states))
    ltl_formula = ltl_formula[:-7]
    ltl_formula = ltl_formula + ')'
    return ltl_formula
