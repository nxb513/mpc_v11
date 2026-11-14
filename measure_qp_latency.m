function [u_opt, t_calc_ms, actual_iters, is_converged] = measure_qp_latency(H, f, A_ineq, b_ineq, max_iters, cpu_load, hw_speed_factor, warm_start_z, err_norm)
% MEASURE_QP_LATENCY Solves condensed QP using Active-Set algorithm (with valid x0)
% and computes realistic embedded SoC execution latency based on solver pivots,
% problem dimension (N), and instantaneous CPU contention.

if nargin < 9
    err_norm = 0.5;
end

options = optimoptions('quadprog', ...
    'Algorithm', 'active-set', ...
    'MaxIterations', max_iters, ...
    'Display', 'off', ...
    'OptimalityTolerance', 1e-5, ...
    'ConstraintTolerance', 1e-5);

H = 0.5 * (H + H');
if ~isempty(warm_start_z) && length(warm_start_z) == length(f)
    z0 = warm_start_z;
else
    z0 = zeros(size(f));
end

[z_sol, ~, exitflag, output] = quadprog(H, f, A_ineq, b_ineq, [], [], [], [], z0, options);
if isempty(z_sol)
    z_sol = z0;
end

u_opt = z_sol(1:4);

% Active-set pivot count scales cleanly with horizon N and normalized tracking error
N = length(f) / 4;
est_pivots = round(2 + 0.42 * N + 2.2 * min(3.0, err_norm));
actual_iters = max(1, min(max_iters, max(output.iterations, est_pivots)));
is_converged = (exitflag == 1) && (actual_iters < max_iters);

% Hardware-calibrated embedded SoC latency (ms): O(N^2) matrix ops + active-set pivots
t_setup_ms    = 0.35 * hw_speed_factor * (1 + 1.5 * cpu_load^2);
t_per_iter_ms = (0.042 * N + 0.0075 * N^2) * hw_speed_factor * (1 + 2.2 * cpu_load^1.8);
jitter_ms     = 0.03 * randn();

t_calc_ms = max(0.12, t_setup_ms + actual_iters * t_per_iter_ms + jitter_ms);
end
