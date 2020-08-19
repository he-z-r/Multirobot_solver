function [W, Cv, Cont, time] = multirobot_coord(num_states, num_robots, transition, init, dest, show_sim)
% transition: a 4-by-num_states matrix, 1st row: up action, 2nd row: down
% action, 3rd row: left, 4th row: right. For example, if a transition
% system has the relations: 1--right-->2 and 2--left-->1, the transition
% matrix is [1 2; 1 2; 1 1; 2 2]. The 1st & 2nd rows are [1 2] because the
% does not go or has nowhere to go under up and down actions.

% init: a 1-by-num_robots matrix. For example, if robot #1 starts at cell
% #3 and robot #2 starts at cell #8, init = [3, 8];

% dest: analogous to init

%% implement fully the transition relations to account for uncertainties
ts = TransSyst(num_states^(num_robots), 5^(num_robots));
% 5 actions: 1 up, 2 down, 3 left, 4 right, 5 stop
s1_up = zeros(1, num_states*2);
s1_up(1:2:num_states*2) = 1:num_states;
s1_up(2:2:num_states*2) = 1:num_states;
s2_up = zeros(1, num_states*2);
s2_up(1:2:num_states*2) = transition(1, :);
s2_up(2:2:num_states*2) = 1:num_states;
a_up = ones(1, num_states*2);

s1_down = zeros(1, num_states*2);
s1_down(1:2:num_states*2) = 1:num_states;
s1_down(2:2:num_states*2) = 1:num_states;
s2_down = zeros(1, num_states*2);
s2_down(1:2:num_states*2) = transition(2, :);
s2_down(2:2:num_states*2) = 1:num_states;
a_down = 2*ones(1, num_states*2);

s1_left = zeros(1, num_states*2);
s1_left(1:2:num_states*2) = 1:num_states;
s1_left(2:2:num_states*2) = 1:num_states;
s2_left = zeros(1, num_states*2);
s2_left(1:2:num_states*2) = transition(3, :);
s2_left(2:2:num_states*2) = 1:num_states;
a_left = 3*ones(1, num_states*2);

s1_right = zeros(1, num_states*2);
s1_right(1:2:num_states*2) = 1:num_states;
s1_right(2:2:num_states*2) = 1:num_states;
s2_right = zeros(1, num_states*2);
s2_right(1:2:num_states*2) = transition(4, :);
s2_right(2:2:num_states*2) = 1:num_states;
a_right = 4*ones(1, num_states*2);

s1_stop = 1:num_states;
s2_stop = 1:num_states;
a_stop = 5*ones(1, num_states);

s1 = [s1_up, s1_down, s1_left, s1_right, s1_stop];
s2 = [s2_up, s2_down, s2_left, s2_right, s2_stop];
a = [a_up, a_down, a_left, a_right, a_stop];

%% Calculate product of transition systems and get the safe set (i.e., without collision)
s1_prod = s1;
s2_prod = s2;
a_prod = a;
for i = 2:num_robots
   [s1_prod, s2_prod, a_prod] = product_of_sys({s1_prod, s2_prod, a_prod}, [num_states^(i-1), num_states], [5^(i-1) 5], {s1, s2, a}); 
end
safe = get_safe(num_states, num_robots);

%% Add transition relations to the AFTS
ts.add_transition(s1_prod, s2_prod, a_prod); % add transitions

%% Add progress group
for i = 1:5^(num_robots)-1
    act = decode_act(i, num_robots);
    for j = 1:(num_states)^(num_robots)
        state = decode(j, num_states, num_robots);
        belong = false;
        for k = 1:numel(act)
            switch act(k)
                case 5
                    continue;
                case 1
                    if s1_up(2*state(k)-1) == s2_up(2*state(k)-1)
                        continue;
                    end
                case 2
                    if s1_down(2*state(k)-1) == s2_down(2*state(k)-1)
                        continue;
                    end
                case 3
                    if s1_left(2*state(k)-1) == s2_left(2*state(k)-1)
                        continue;
                    end
                case 4
                    if s1_right(2*state(k)-1) == s2_right(2*state(k)-1)
                        continue;
                    end
            end
            belong = true;
        end
        if belong
            ts.add_progress_group(i, j);
        end
    end
end

%% Synthesize based on the destination
init_state = encode(init, num_states);
dest_state = encode(dest, num_states);
tic;
[W, Cv, Cont] = win_primal(ts, safe, dest_state, [], 'exists', 'forall'); % compute winning set (edited)
time = toc;

%% Synthesis finished. Simulation part.
if show_sim
    disp('Synthesis finished. Simulation starts.');
    state = init_state;
    states = decode(state, num_states, num_robots);
    disp(states);
    while state ~= dest_state
        action = Cont(state);
        actions = decode_act(action, num_robots);
        disp('Actions taken:')
        disp(actions)
        for i = 1:numel(actions)
            switch actions(i)
                case 1
                    if randi([0 1])
                        states(i) = s2_up(2*states(i)-1);
                    else
                        continue;
                    end
                case 2
                    if randi([0 1])
                        states(i) = s2_down(2*states(i)-1);
                    else
                        continue;
                    end
                case 3
                    if randi([0 1])
                        states(i) = s2_left(2*states(i)-1);
                    else
                        continue;
                    end
                case 4
                    if randi([0 1])
                        states(i) = s2_right(2*states(i)-1);
                    else
                        continue;
                    end
                case 5
                    continue;
            end
        end
        disp('Current states:');
        disp(states);
        state = encode(states, num_states);
    end
else
    disp('Synthesis finished.')
end
end

function [s1_prod, s2_prod, a_prod] = product_of_sys(sys1, num_states, num_actions, sys2)
% calculate the product of 2 systems. Example: calculate the product of a
% 2-robot system and a 1-robot system (namely, a 3-robot systems),
% num_states = [s^2, s], num_actions = [25 5], sys1 = {s1, s2, a}
if isempty(sys2)
    sys2 = sys1;
end
s1_prod = [];
s2_prod = [];
a_prod = [];
for i = 1:numel(sys1{1})
    for j = 1:numel(sys2{1})
        % The states are encoded as follows
        s1_prod = [s1_prod, (sys1{1}(i)-1)*num_states(2) + sys2{1}(j)];
        s2_prod = [s2_prod, (sys1{2}(i)-1)*num_states(2) + sys2{2}(j)];
        a_prod = [a_prod (sys1{3}(i)-1)*num_actions(2) + sys2{3}(j)];
    end
end
end

function safe = get_safe(num_states, num_robots)
% calculate the safe states where there is no collision
safe = [];
num_states_tot = num_states^(num_robots);
for i = 1:num_states_tot
    dec_state = decode(i, num_states, num_robots);
    dec_state = sort(dec_state);
    if numel(unique(dec_state)) == numel(dec_state)
        % no collision
        safe = [safe, i];
    end
end
end

function dec_state = decode(state, num_states, num_robots)
% decode an encoded state
dec_state = mod(state, num_states);
if abs(dec_state) < 0.0001
    dec_state = num_states;
end
state = round((state-dec_state)/num_states);
for i = 2:num_robots
    dec_state = [mod(state, num_states)+1 , dec_state];
    if abs(dec_state(1)) < 0.0001
        dec_state(1) = num_states;
    end
    state = round((state-dec_state(1))/num_states);
end
end

function dec_act = decode_act(action, num_robots)
% decode an encoded action
dec_act = decode(action, 5, num_robots);
end

function enc_state = encode(states, num_states)
% encode the states represented by an array, convert it into a scalar
enc_state = states(1);
for i = 2:numel(states)
    enc_state = (enc_state-1)*num_states+states(i);
end
end