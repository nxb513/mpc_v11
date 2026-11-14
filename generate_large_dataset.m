function generate_large_dataset()
% GENERATE_LARGE_DATASET Profiles QP execution latencies across realistic closed-loop
% horizons, iteration caps, tracking errors, CPU loads, and hardware profiles.

fprintf('Generating hardware-calibrated QP latency dataset (leak-free raw features)...\n');
rng(2026);

Ts = 0.02;
[Ad, Bd] = quadrotor_model(Ts, 1.0);
Q = diag([22, 22, 30, 3, 3, 4, 8, 8, 4, 1, 1, 1]);
R = diag([0.3, 0.6, 0.6, 0.6]);
[~, P_term] = lqr_controller(Ad, Bd, Q, R);

N_set = [4, 5, 8, 10, 12, 15, 16, 20];
iter_set = [8, 12, 15, 20, 30, 40];
hw_set = [1.0, 2.4];

num_samples = 4000;
X_raw = zeros(num_samples, 6); % [N, max_iters, cpu_load, pos_err_norm, vel_err_norm, hw_speed]
Y_lat = zeros(num_samples, 1);

u_min = [-6.0; -1.2; -1.2; -0.8];
u_max = [ 8.0;  1.2;  1.2;  0.8];

for i = 1:num_samples
    N = N_set(randi(length(N_set)));
    max_iters = iter_set(randi(length(iter_set)));
    cpu_load = 0.10 + 0.85 * rand();
    hw_speed = hw_set(randi(length(hw_set)));

    % Realistic closed-loop agile tracking error distribution
    pos_err = 0.45 * randn(3, 1);
    vel_err = 0.65 * randn(3, 1);
    att_err = 0.12 * randn(6, 1);
    e0 = [pos_err; vel_err; att_err];
    err_norm = norm(pos_err) + 0.4 * norm(vel_err);

    [H, f, A_ineq, b_ineq] = build_condensed_qp(Ad, Bd, Q, R, P_term, N, e0, u_min, u_max);
    [~, t_calc_ms, ~, ~] = measure_qp_latency(H, f, A_ineq, b_ineq, max_iters, cpu_load, hw_speed, [], err_norm);

    X_raw(i, :) = [N, max_iters, cpu_load, norm(pos_err), norm(vel_err), hw_speed];
    Y_lat(i, 1) = t_calc_ms;
end

save('mpc_large_dataset.mat', 'X_raw', 'Y_lat');
fprintf('Saved %d leak-free samples to mpc_large_dataset.mat\n', num_samples);
end
