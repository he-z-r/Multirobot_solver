""" This script tries only using the TuLiP toolbox to solve a multirobot path planning (MrPP) problem.
As opposed to multirobot path execution (MrPE) problem, predefined paths are not given
"""

# Import the packages that we need
from __future__ import print_function
import numpy as np
from tulip import spec
from tulip.transys import machines
from tulip import synth
import gr1_fragment

# Define the parameters & desired trajectories
num_robots = 2  # must be > 1
num_cells = 5
init = np.array([1, 4])
dest = np.array([3, 5])

# Define the env_vars based on the params % trajectories
env_vars = set()
env_vars_list = []
for r in range(1, num_robots + 1):
    env_vars_list.append([])
    for p in range(1, num_cells + 1):
        new_env_var = 's' + str(r) + 'a' + str(p)
        env_vars |= {new_env_var}
        env_vars_list[r - 1].append(new_env_var)

# Define the initial conditions
env_init = set()
for r in range(num_robots):
    env_init |= {env_vars_list[r][init[r] - 1]}

env_safe = set()
env_prog = set()
# Encode the constraint that each robot can only appear in one position
env_unique = set()
for r in range(1, num_robots + 1):
    for i in range(len(env_vars_list[r - 1])):
        new_env_unique = '( ' + env_vars_list[r - 1][i] + ' ) <-> ( '
        for j in range(len(env_vars_list[r - 1])):
            if j != i:
                new_env_unique += '!' + env_vars_list[r - 1][j] + ' && '
        new_env_unique = new_env_unique[:-3] + ')'
        env_unique |= {new_env_unique}
env_safe |= env_unique

# Encode the transition system relations
# transition = np.array([[0, 2, 7, 5], [1, 3, 0, 0], [2, 4, 0, 6], [3, 0, 0, 8], [0, 0, 1, 0], [0, 0, 3, 0], [0, 0, 0, 1], [0, 0, 4, 0]])
# transition = np.array([[0, 2, 0, 0], [1, 0, 0, 0]])
# transition = np.array([[0, 2, 0, 0], [1, 3, 0, 0], [0, 2, 0, 0]])
transition = np.array([[0, 0, 0, 2], [4, 5, 1, 3], [0, 0, 2, 0], [0, 2, 0, 0], [2, 0, 0, 0]])
# transition = np.array([[0, 2, 0, 0], [1, 3, 0, 4], [2, 0, 0, 0], [0, 0, 2, 5], [9, 0, 4, 6], [0, 0, 5, 7], [8, 10, 6, 0], [0, 7, 0, 0], [0, 5, 4, 6], [7, 0, 0, 0]])
actions = ['up', 'down', 'left', 'right']
env_trans = set()
sys_bad_act = set()
# env_trans |= {'( s1a1 ) -> ( stop1 || down1 )'}
# env_trans |= {'( s1a2 ) -> ( stop1 || up1 )'}
for i in range(1, num_cells + 1):
    for r in range(num_robots):
        for d in range(4):
            if transition[i - 1][d] > 0:
                env_trans |= {'( ' + env_vars_list[r][i - 1] + ' && ' + actions[d] + str(r + 1) + ' ) -> X( ' +
                              env_vars_list[r][i - 1] + ' || ' + env_vars_list[r][transition[i - 1][d] - 1] + ' )'}
            else:
                sys_bad_act |= {'( ' + env_vars_list[r][i - 1] + ' ) -> ( !' + actions[d] + str(r + 1) + ' )'}
        env_trans |= {'( ' + env_vars_list[r][i - 1] + ' && ' + 'stop' + str(r + 1) + ' ) -> X( ' +
                      env_vars_list[r][i - 1] + ' )'}

env_safe |= env_trans

# Encode the constraint that each robot should either not go or eventually not in the current cell
env_go = set()
for r in range(1, num_robots + 1):
    for i in range(num_cells):
        if i != dest[r - 1] - 1:
            env_go |= {'( !' + env_vars_list[r - 1][i] + ' || ' + 'stop' + str(r) + ' )'}
env_prog |= env_go

# Define sys_vars
sys_vars = set()
for r in range(1, num_robots + 1):
    sys_vars |= {'stop' + str(r)}
    sys_vars |= {'left' + str(r)}
    sys_vars |= {'right' + str(r)}
    sys_vars |= {'up' + str(r)}
    sys_vars |= {'down' + str(r)}

# Define initial command. Assume nothing.
sys_init = set()

sys_safe = set()
sys_prog = set()

# Encode the constraint that every robot should stay after reaching its destination
sys_dest = set()
for r in range(num_robots):
    sys_dest |= {env_vars_list[r][dest[r] - 1]}
sys_prog |= sys_dest

# Encode the constraint that each time one action must be selected
sys_good_act = set()
for r in range(1, num_robots + 1):
    sys_good_act |= {'( left' + str(r) + ' ) <-> ( ' + '!right' + str(r) + ' && ' + '!up' + str(r) + ' && ' + '!down' +
                     str(r) + ' && ' + '!stop' + str(r) + ' )'}
    sys_good_act |= {'( right' + str(r) + ' ) <-> ( ' + '!left' + str(r) + ' && ' + '!up' + str(r) + ' && ' + '!down' +
                     str(r) + ' && ' + '!stop' + str(r) + ' )'}
    sys_good_act |= {'( up' + str(r) + ' ) <-> ( ' + '!right' + str(r) + ' && ' + '!left' + str(r) + ' && ' + '!down' +
                     str(r) + ' && ' + '!stop' + str(r) + ' )'}
    sys_good_act |= {'( down' + str(r) + ' ) <-> ( ' + '!right' + str(r) + ' && ' + '!up' + str(r) + ' && ' + '!left' +
                     str(r) + ' && ' + '!stop' + str(r) + ' )'}
    sys_good_act |= {'( stop' + str(r) + ' ) <-> ( ' + '!right' + str(r) + ' && ' + '!up' + str(r) + ' && ' + '!down' +
                     str(r) + ' && ' + '!left' + str(r) + ' )'}

sys_safe |= sys_good_act
sys_safe |= sys_bad_act

# Encode the collision avoidance constraints
sys_collision = set()
for i in range(num_cells):
    new_sys_collision = '!( '
    for r in range(num_robots):
        new_sys_collision += env_vars_list[r][i] + ' && '
    new_sys_collision = new_sys_collision[:-3] + ')'
    sys_collision |= {new_sys_collision}
sys_safe |= sys_collision
# Create a GR(1) specification
specs = spec.GRSpec(env_vars, sys_vars, env_init, sys_init,
                    env_safe, sys_safe, env_prog, sys_prog)
print(specs)
specs.qinit = '\A \E'  # Moore initial condition synthesized too
specs.moore = False
specs.plus_one = False

print('Start synthesis')
ctrl = synth.synthesize(specs)
print('End synthesis')
assert ctrl is not None, 'unrealizable'

machines.random_run(ctrl, N=20)
