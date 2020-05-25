""" This script tries only using the TuLiP toolbox to solve a multirobot motion planning problem.
Specifically, a field of cells and a number of robots are specified along with the system dynamics,
and constraints are written in monolithic LTL syntax.
"""


# Import the packages that we need
from __future__ import print_function
import numpy as np
from tulip import spec
from tulip.transys import machines
from tulip import synth
import State_encoding
import LTL_encoding

# Workspace parameters
num_rows = 3
num_cols = 3
num_robots = 2
robot_init_pos = np.array([[0, 0], [1, 0], [2, 0]])
# The environment variables and specifications are none
env_vars = set()
env_init = set()
env_prog = set()
env_safe = set()

# Generate robot system dynamics, encoded in sys_safe
#   1. Can move up, down, left, and right
#   2. Can't collide
num_cells = num_rows*num_cols
num_states = num_cells**num_robots
sys_vars = {}
sys_vars['loc'] = (0, num_states - 1)
sys_init_pos = np.array([0, 3])
sys_init = {'loc='+str(State_encoding.array2digit(sys_init_pos, num_cells))}
sys_safe = set()
sys_prog = set()

curr_pos = np.zeros(num_robots, dtype=int)
iter_num_robot = 0
for curr_state in range(num_states):
    LTL = '' # the LTL formula representing the dynamics of this state
    next_state = np.array([], dtype=int)
    if len(np.unique(curr_pos)) == len(curr_pos):
        for next_movement in range(4**num_robots):
            is_valid = True
            temp_curr_pos = np.copy(curr_pos)
            temp_next_movement = next_movement
            for curr_robot in range(num_robots):
                next_move = temp_next_movement % 4
                temp_next_movement //= 4
                # check if this move is valid, i.e., no collision and no out of grid
                if next_move == 0:
                    # move right
                    if curr_pos[curr_robot] % num_cols < num_cols - 1:
                        temp_curr_pos[curr_robot] += 1
                    else:
                        is_valid = False
                        break
                elif next_move == 1:
                    # move up
                    if curr_pos[curr_robot] >= num_cols:
                        temp_curr_pos[curr_robot] -= num_cols
                    else:
                        is_valid = False
                        break
                elif next_move == 2:
                    # move left
                    if curr_pos[curr_robot] % num_cols > 0:
                        temp_curr_pos[curr_robot] -= 1
                    else:
                        is_valid = False
                        break
                elif next_move == 3:
                    # move down
                    if curr_pos[curr_robot] < num_cols * (num_rows - 1):
                        temp_curr_pos[curr_robot] += num_cols
                    else:
                        is_valid = False
                        break
            # check the following circumstance: two robots exchange their positions
            if is_valid:
                for i in range(num_robots):
                    for j in range(i+1, num_robots):
                        if temp_curr_pos[i] == curr_pos[j] and temp_curr_pos[j] == curr_pos[i]:
                            is_valid = False
                            break
                    if not is_valid:
                        break
            if is_valid:
                # check if there is duplicate in the next state
                if len(np.unique(temp_curr_pos)) == len(temp_curr_pos):
                    next_state = np.append(next_state, State_encoding.array2digit(temp_curr_pos, num_cells))

    # It should be the case that there exist some next states
        if len(next_state) > 0:
            LTL = 'loc=' + str(curr_state) + ' -> X (loc=' + str(next_state[0])
            for i in range(1, len(next_state)):
                LTL = LTL + ' || loc=' + str(next_state[i])
            LTL = LTL + ')'
            sys_safe |= {LTL}
    for i in range(num_robots):
        if curr_pos[i] < num_cells - 1:
            curr_pos[i] += 1
            break
        else:
            curr_pos[i] = 0

# Generate inner nad outer logic
# Constraint 1:
# []<> ([6,7,8], 2)
# Constraint 2:
# ([]<> [2], 2)
# Constraint 3:
# []! ([1], 1)
ltl_formula_1 = LTL_encoding.cltl_conversion(np.array([6, 7, 8]), 2, num_robots, num_cells)
sys_prog |= {ltl_formula_1}
for i in range(num_robots):
    ltl_formula_2 = LTL_encoding.ltl_conversion_ap(np.array([2]), i, num_robots, num_cells)
    sys_prog |= {ltl_formula_2}
ltl_formula_3 = LTL_encoding.ltl_negate(LTL_encoding.cltl_conversion(np.array([1]), 1, num_robots, num_cells))
sys_safe |= {ltl_formula_3}

# Create a GR(1) specification
specs = spec.GRSpec(env_vars, sys_vars, env_init, sys_init,
                    env_safe, sys_safe, env_prog, sys_prog)
specs.qinit = '\E \A'  # Moore initial condition synthesized too

ctrl = synth.synthesize(specs)
assert ctrl is not None, 'unrealizable'


# Generate a graphical representation of the controller for viewing
if not ctrl.save('gr1_set.png'):
    print(ctrl)
machines.random_run(ctrl, N=30)
