function [cpu_load, a_dist] = simulate_workload(M, Ts, seed, cpu_mean, dist_std)
% SIMULATE_WORKLOAD Background CPU load and external disturbance. Both are
% experiment axes swept in experiment_config.m.
%
%   cpu_load(k) : background CPU utilisation of the flight computer's core,
%                 cpu_mean + first-order Gauss-Markov fluctuation
%                 (std 0.1, correlation time 0.5 s), clipped to [0, 0.95].
%   a_dist(:,k) : external acceleration disturbance in the world frame [m/s^2],
%                 3 independent first-order Gauss-Markov processes with
%                 stationary std dist_std and correlation time 1 s.
% The Standard Quad of [S2] Table I has no drag coefficient, so a wind-velocity
% model cannot be mapped to a force; the disturbance is modelled directly as an
% acceleration.
% The two processes use independent random streams.

if nargin < 4 || isempty(cpu_mean), cpu_mean = 0.5; end
if nargin < 5 || isempty(dist_std), dist_std = 0.5; end
rs_c = RandStream('mt19937ar', 'Seed', seed);
rs_d = RandStream('mt19937ar', 'Seed', seed + 7919);

cpu_load = cpu_mean + gauss_markov(rs_c, 1, M, Ts, 0.5, 0.1);
cpu_load = min(0.95, max(0.0, cpu_load));
a_dist = gauss_markov(rs_d, 3, M, Ts, 1.0, dist_std);
end

function y = gauss_markov(rs, n, M, Ts, tau, sigma)
% x_{k+1} = a x_k + sqrt(1 - a^2) sigma w_k, a = exp(-Ts / tau), stationary std sigma
a = exp(-Ts / tau);
y = zeros(n, M);
y(:, 1) = sigma * randn(rs, n, 1);
for k = 1:M-1
    y(:, k+1) = a * y(:, k) + sqrt(1 - a^2) * sigma * randn(rs, n, 1);
end
end
