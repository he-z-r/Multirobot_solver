time_stats = [];

num_states = 4;
while true
    time_it = 0;
    num_robots = 2;
    transition = randi([1 num_states], 4, num_states);
    init = randperm(num_states);
    init = init(1:num_robots);
    dest = randperm(num_states);
    dest = dest(1:num_robots);
    [W, Cv, Cont, time] = multirobot_coord(num_states, num_robots, transition, init, dest, false);
    disp(time);
    time_it = time;
    
%     num_robots = 3;
%     transition = randi([1 num_states], 4, num_states);
%     init = randperm(num_states);
%     init = init(1:num_robots);
%     dest = randperm(num_states);
%     dest = dest(1:num_robots);
%     [W, Cv, Cont, time] = multirobot_coord(num_states, num_robots, transition, init, dest, false);
%     disp(time);
%     time_it(2) = time;
    
    time_stats = [time_stats, time_it];
    num_states = num_states + 1;
end