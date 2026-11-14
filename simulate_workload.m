function [cpu_load, wind_dist] = simulate_workload(M, Ts, seed)
% SIMULATE_WORKLOAD Generates realistic bursty onboard CPU utilization
% and a statistically INDEPENDENT periodic wind gust disturbance profile.

if nargin >= 3
    rng(seed);
end

t = (0:M-1) * Ts;

% 1. Onboard CPU workload: baseline + background vision bursts + stochastic jitter
base_cpu = 0.28 + 0.14 * sin(0.8 * t) + 0.08 * cos(2.1 * t);
bursts = zeros(1, M);
num_bursts = max(3, round(M * Ts / 2.5));
burst_centers = randi([max(1, round(0.05*M)), max(2, round(0.95*M))], 1, num_bursts);
for b = 1:num_bursts
    width = round(0.4 / Ts);
    idx1 = max(1, burst_centers(b) - width);
    idx2 = min(M, burst_centers(b) + width);
    bursts(idx1:idx2) = bursts(idx1:idx2) + 0.35 * exp(-((idx1:idx2) - burst_centers(b)).^2 / (2*(width/2)^2));
end
cpu_load = min(0.95, max(0.10, base_cpu + bursts + 0.03 * randn(1, M)));

% 2. Physical wind disturbance: completely independent of cpu_load (repeats every 10s window)
rng(seed + 999);
wind_dist = zeros(12, M);
t_mod = mod(t, 10.0);
gust_t1 = (t_mod > 2.5 & t_mod < 4.2);
gust_t2 = (t_mod > 7.0 & t_mod < 8.8);
wind_dist(4, gust_t1) = 1.6 * sin(pi * (t_mod(gust_t1) - 2.5) / 1.7);
wind_dist(5, gust_t2) = -1.8 * sin(pi * (t_mod(gust_t2) - 7.0) / 1.8);
wind_dist(4:6, :) = wind_dist(4:6, :) + 0.08 * randn(3, M);
end
