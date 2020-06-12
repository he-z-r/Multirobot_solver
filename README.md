# Multirobot_solver
There are 2 robots in the problem. Robot #1 goes from: init1 -> 1 -> 2 -> 3.
Robot #2 goes from: init2 -> 4 -> 2 -> 5.
The states are encoded as:
init# / s#a#a#
For init#, # means the robot number.
For s#a#a#,
The first one is the # of robots.
The second one is the cell #.
The third one is the number of times that the robot has visited this cell.
For example, s1a4a2 means the Robot #1 is in cell #4 for the 2nd time.
