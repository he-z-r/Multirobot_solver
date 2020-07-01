""" This script tries only using the TuLiP toolbox to solve a multirobot motion planning problem.
Specifically, a field of cells and a number of robots are specified along with the system dynamics,
and constraints are written in monolithic LTL syntax.
The difference between this script and TuLiP_multirobot_solver.py
"""

# Import the packages that we need
from __future__ import print_function
import numpy as np
from tulip import spec
from tulip.transys import machines
from tulip import synth
import gr1_fragment
import State_encoding
import LTL_encoding

# Workspace parameters
num_rows = 3
num_cols = 2
num_robots = 2
delay = 2
robot_init_pos = np.array([[0, 1], [1, 0]])
# The environment variables
env_vars = {}
env_init = set()
env_prog = set()
env_safe = set()

for r in range(1, num_robots + 1):
    env_vars[('x' + str(r))] = (0, num_cols - 1)
    env_vars[('y' + str(r))] = (0, num_rows - 1)
    env_vars[('delay' + str(r))] = 'boolean'
    env_vars[('d' + str(r))] = (-1, delay + 1)
print(env_vars)

for r in range(1, num_robots + 1):
    env_init |= {'x' + str(r) + '=' + str(robot_init_pos[r - 1][0])}
    env_init |= {'y' + str(r) + '=' + str(robot_init_pos[r - 1][1])}
    env_init |= {'d' + str(r) + '=0'}
    env_init |= {'!delay' + str(r)}
print(env_init)

# Now no need to encode that each robot should only appear in one position
# Encode the transition relations
env_trans_x = set()
env_trans_y = set()
for r in range(1, num_robots + 1):
    for col in range(num_cols):
        if col != num_cols - 1:
            #env_trans_x |= {'( x' + str(r) + '=' + str(col) + ' && right' + str(r) + ' ) -> ( ( X(x' + str(r) + '=' +
            #                str(col+1) + ') && !delay' + str(r) + ' ) || ( X(x' + str(r) + '=' + str(col) +
            #                ') && delay' + str(r) + ' ) )'}
            env_trans_x |= {'( x' + str(r) + '=' + str(col) + ' && right' + str(r) + ' ) -> ( ( X(x' + str(r) + '=' +
                            str(col+1) + ' && !delay' + str(r) + ') ) || ( X(x' + str(r) + '=' + str(col) +
                            ' && delay' + str(r) + ') ) )'}
        if col != 0:
            env_trans_x |= {'( x' + str(r) + '=' + str(col) + ' && left' + str(r) + ' ) -> ( ( X(x' + str(r) + '=' +
                            str(col - 1) + ' && !delay' + str(r) + ') ) || ( X(x' + str(r) + '=' + str(col) +
                            ' && delay' + str(r) + ') ) )'}
        env_trans_x |= {'( x' + str(r) + '=' + str(col) + ' && up' + str(r) + ' ) -> X( x' + str(r) + '=' + str(col) +
                        ' )'}
        env_trans_x |= {'( x' + str(r) + '=' + str(col) + ' && down' + str(r) + ' ) -> X( x' + str(r) + '=' + str(col) +
                        ' )'}
        env_trans_x |= {'( x' + str(r) + '=' + str(col) + ' && stop' + str(r) + ' ) -> ( X( x' + str(r) + '=' + str(col) +
                        ' )' + ' )'}
    for row in range(num_rows):
        if row != num_rows - 1:
            env_trans_y |= {'( y' + str(r) + '=' + str(row) + ' && up' + str(r) + ' ) -> ( ( X(y' + str(r) + '=' +
                            str(row + 1) + ' && !delay' + str(r) + ') ) || ( X(y' + str(r) + '=' + str(row) +
                            ' && delay' + str(r) + ') ) )'}
        if row != 0:
            env_trans_y |= {'( y' + str(r) + '=' + str(row) + ' && down' + str(r) + ' ) -> ( ( X(y' + str(r) + '=' +
                            str(row - 1) + ' && !delay' + str(r) + ') ) || ( X(y' + str(r) + '=' + str(row) +
                            ' && delay' + str(r) + ') ) )'}
        env_trans_y |= {'( y' + str(r) + '=' + str(row) + ' && left' + str(r) + ' ) -> X( y' + str(r) + '=' + str(row) +
                        ' )'}
        env_trans_y |= {'( y' + str(r) + '=' + str(row) + ' && right' + str(r) + ' ) -> X( y' + str(r) + '=' + str(row) +
                        ' )'}
        env_trans_y |= {'( y' + str(r) + '=' + str(row) + ' && stop' + str(r) + ' ) -> ( X( y' + str(r) + '=' + str(row) +
                        ' )' + ' )'}
print(env_trans_x)
print(env_trans_y)
env_safe |= env_trans_x
env_safe |= env_trans_y

# Relate the delay variables (delay#) to the delay-clock variables (d#)
delay2clock_minus = '( '
delay2clock_plus = ''
for r in range(1, num_robots + 1):
    delay2clock_minus += '( X(delay' + str(r) + ') && d' + str(r) + '=' + str(delay) + ' ) || '
delay2clock_plus = '( !' + delay2clock_minus[:-3] + ') ) -> ('
delay2clock_minus = delay2clock_minus[:-3] + ') -> ( '
for r in range(1, num_robots + 1):
    for d in range(delay + 1):
        delay2clock_minus += '( ( X(!delay' + str(r) + ') && d' + str(r) + '=' + str(d) + ' ) -> X(d' + str(r) + '=' + \
                             str(d - 1) + ') ) && '
        delay2clock_minus += '( ( X(delay' + str(r) + ') && d' + str(r) + '=' + str(d) + ' ) -> X(d' + str(r) + '=' + \
                             str(d) + ') ) && '
        delay2clock_plus += '( ( X(delay' + str(r) + ') && d' + str(r) + '=' + str(d) + ' ) -> X(d' + str(r) + '=' + \
                            str(d + 1) + ') ) && '
        delay2clock_plus += '( ( X(!delay' + str(r) + ') && d' + str(r) + '=' + str(d) + ' ) -> X(d' + str(r) + '=' + \
                            str(d) + ') ) && '
delay2clock_minus = delay2clock_minus[:-3] + ')'
delay2clock_plus = delay2clock_plus[:-3] + ')'
print(delay2clock_minus)
print(delay2clock_plus)
env_safe |= {delay2clock_minus}
env_safe |= {delay2clock_plus}

# Constraint on the maximum delay
env_max_delay = set()
env_not_delay = set()
for r in range(1, num_robots + 1):
    env_max_delay |= {'!(d' + str(r) + '=' + '-1)'}
    env_max_delay |= {'!(d' + str(r) + '=' + str(delay + 1) + ')'}
    env_not_delay |= {'!(delay' + str(r) + ')'}
print(env_max_delay)
print(env_not_delay)
env_safe |= env_max_delay
env_prog |= env_not_delay

# Define sys_vars
sys_vars = {}
for r in range(1, num_robots + 1):
    sys_vars['stop' + str(r)] = 'boolean'
    sys_vars['left' + str(r)] = 'boolean'
    sys_vars['right' + str(r)] = 'boolean'
    sys_vars['up' + str(r)] = 'boolean'
    sys_vars['down' + str(r)] = 'boolean'
print(sys_vars)

# Define initial command. Assume nothing.
sys_init = set()
sys_safe = set()
sys_prog = set()

# Avoid bad actions
sys_bad_act = set()
for r in range(1, num_robots + 1):
    sys_bad_act |= {'x' + str(r) + '=0 -> !left' + str(r)}
    sys_bad_act |= {'x' + str(r) + '=' + str(num_cols - 1) + ' -> !right' + str(r)}
    sys_bad_act |= {'y' + str(r) + '=0 -> !down' + str(r)}
    sys_bad_act |= {'y' + str(r) + '=' + str(num_rows - 1) + ' -> !up' + str(r)}
    # sys_bad_act |= {'delay' + str(r) + ' -> ' + '!stop' + str(r)}
print(sys_bad_act)
sys_safe |= sys_bad_act

# Encode the constraint that each time exactly one action must be chosen
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
print(sys_good_act)

# Encode the constraint that if the last action was not executed yet, the same action should the next action
sys_same_act = set()
for r in range(1, num_robots + 1):
    sys_same_act |= {'( delay' + str(r) + ' && left' + str(r) + ' ) -> X( left' + str(r) + ' )'}
    sys_same_act |= {'( delay' + str(r) + ' && right' + str(r) + ' ) -> X( right' + str(r) + ' )'}
    sys_same_act |= {'( delay' + str(r) + ' && up' + str(r) + ' ) -> X( up' + str(r) + ' )'}
    sys_same_act |= {'( delay' + str(r) + ' && down' + str(r) + ' ) -> X( down' + str(r) + ' )'}
print(sys_same_act)
#sys_safe |= sys_same_act

# User-defined system requirements
# Constraint 1: []<>([[0, 0], [0, 1], [0, 2]], 2)
# Constraint 2: []!([1, 1], 1)
# Constraint 3: ([]<>[1, 2], 2)

#sys_prog |= {'(x1=0)'}
#sys_safe |= {'!(x1=1 && y1=1)'}
sys_prog |= {'(x1=0) && (x2=0)'}
#sys_prog |= {'x1=1 && y1=0'}
#sys_prog |= {'x2=1 && y2=0'}
#sys_prog |= {'(x1=1) && (x2=1)'}
sys_safe |= {'!(x1=1 && y1=1) && !(x2=1 && y2=1)'}
sys_prog |= {'x1=1 && y1=2'}
sys_prog |= {'x2=1 && y2=2'}

# Create a GR(1) specification
specs = spec.GRSpec(env_vars, sys_vars, env_init, sys_init,
                    env_safe, sys_safe, env_prog, sys_prog)
specs.qinit = '\E \A'  # Moore initial condition synthesized too
specs.moore = False
specs.plus_one = False
print(specs)

specs.check_syntax()

print('Start synthesis')
ctrl = synth.synthesize(specs)
print('End synthesis')
assert ctrl is not None, 'unrealizable'

machines.random_run(ctrl, N=100)
