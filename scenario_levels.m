function S = scenario_levels(traj_type)
% SCENARIO_LEVELS Ladder of six flight scenarios for one trajectory shape, defined
% by how much the reference demands relative to the physical actuator limits of
% the platform (quad_params.m, [S2] Table I), not by hand-picked factors:
%
%   S1 nominal     : aggressiveness 1 (reference well inside the rotor limits)
%   S2 half-way    : aggressiveness midway between S1 and S3
%   S3 at limit    : largest aggressiveness whose feed-forward rotor thrusts never
%                    leave [T_min, T_max] (0 % of the time beyond the limits)
%   S4 beyond 10%  : feed-forward beyond the rotor limits 10 % of the time
%   S5 beyond 20%  : feed-forward beyond the rotor limits 20 % of the time
%   S6 beyond 30%  : feed-forward beyond the rotor limits 30 % of the time
%
% Aggressiveness scales the trajectory's angular frequency (generate_trajectories.m).
% Levels are found by bisection on a 30 s reference at Ts = 25 ms and cached.
% Disturbance and CPU load are separate experiment axes (simulate_workload.m).

persistent cache
if isempty(cache), cache = containers.Map(); end
key = [lower(traj_type), '_', quad_params().name];
if isKey(cache, key)
    S = cache(key);
    return;
end

over = @(a) local_over(traj_type, a);
a_lim = bisect(@(a) over(a) > 0, 1.0, 8.0);       % edge of the actuator envelope
a_10  = bisect(@(a) over(a) >= 10, a_lim, 8.0);
a_20  = bisect(@(a) over(a) >= 20, a_10, 8.0);
a_30  = bisect(@(a) over(a) >= 30, a_20, 8.0);
a_mid = 0.5 * (1.0 + a_lim);
aggr = {1.0, a_mid, a_lim, a_10, a_20, a_30};

S = struct('name', {'S1 nominal', 'S2 half-way', 'S3 at limit', 'S4 beyond 10%', 'S5 beyond 20%', 'S6 beyond 30%'}, ...
           'aggr', aggr, 'ff_over', cellfun(over, aggr, 'UniformOutput', false));
cache(key) = S;
end

function v = local_over(traj_type, a)
[~, ~, st] = generate_trajectories(traj_type, 0.025, 30, a, false);
v = st.ff_over;
end

function a = bisect(pred, lo, hi)
% last a in [lo, hi] for which the monotone predicate pred(a) is still false
if pred(lo), a = lo; return; end
for it = 1:30
    mid = 0.5 * (lo + hi);
    if pred(mid), hi = mid; else, lo = mid; end
end
a = lo;
end
