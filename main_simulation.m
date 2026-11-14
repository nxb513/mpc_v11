%% Computation-Aware Multi-Parameter Adaptive MPC for Agile UAVs
clear; clc; close all;
rng(2026);

fprintf('=================================================================\n');
fprintf(' Computation-Aware Multi-Parameter Adaptive MPC Benchmark Suite\n');
fprintf('=================================================================\n');

if ~isfile('trained_predictor.mat')
    train_model();
end

Q = diag([22, 22, 30, 3, 3, 4, 8, 8, 4, 1, 1, 1]);
R = diag([0.3, 0.6, 0.6, 0.6]);

%% PART 1: 3-Trajectory Tracking Evaluation at 40 Hz (Resource-Constrained SoC, 50s Horizon)
freq_nominal = 40;
Ts = 1 / freq_nominal;
T_budget_ms = 0.60 * (Ts * 1000); % 15.0 ms real-time control budget
hw_speed = 2.4;
T_sim = 50.0;

[Ad, Bd] = quadrotor_model(Ts, 1.0);
[K_lqr, P_term] = lqr_controller(Ad, Bd, Q, R);

trajectories = {'lemniscate', 'helix', 'racing'};
controllers  = {'fixed_5', 'fixed_15', 'anytime', 'rule_based', 'proposed'};
ctrl_labels  = {'Fixed-MPC (N=5)', 'Fixed-MPC (N=15)', 'Anytime MPC', 'Rule-Based MPC', 'Proposed Adaptive'};

fprintf('\n--- PART 1: 3-Trajectory Benchmark (40 Hz, Constrained SoC, T_sim = %.1f s) ---\n', T_sim);
for tr = 1:length(trajectories)
    traj_name = trajectories{tr};
    [x_ref, u_ref, vel_stats] = generate_trajectories(traj_name, Ts, T_sim);
    M = size(x_ref, 2);
    [cpu_load, wind_dist] = simulate_workload(M, Ts, 100 + tr);

    fprintf('\nTrajectory: %s | Speed [min, mean, max] = [%.2f, %.2f, %.2f] m/s\n', ...
        upper(traj_name), vel_stats.min, vel_stats.mean, vel_stats.max);
    fprintf('%-20s | %-10s | %-12s | %-12s\n', 'Controller', 'RMSE (m)', 'Miss Rate(%)', 'Mean Lat(ms)');
    fprintf('---------------------------------------------------------------\n');

    for c = 1:length(controllers)
        mode = controllers{c};
        x = x_ref(:, 1);
        u_prev = zeros(4, 1);
        state_mem.last_N = 10;

        pos_err_sq = zeros(1, M);
        misses = zeros(1, M);
        latencies = zeros(1, M);

        for k = 1:M
            win_end = min(M, k + 20);
            [du, info] = adaptive_mpc_solver(mode, x, x_ref(:, k:win_end), u_ref(:, k:win_end), u_prev, Ad, Bd, Q, R, P_term, K_lqr, T_budget_ms, cpu_load(k), hw_speed, state_mem);
            x = Ad * x + Bd * (u_ref(:, k) + du) + wind_dist(:, k) * Ts;

            u_prev = du;
            state_mem.last_N = info.N_sel;
            pos_err_sq(k) = sum((x(1:3) - x_ref(1:3, k)).^2);
            misses(k) = info.deadline_missed;
            latencies(k) = info.t_calc_ms;
        end

        fprintf('%-20s | %10.4f | %12.2f | %12.3f\n', ctrl_labels{c}, sqrt(mean(pos_err_sq)), 100*mean(misses), mean(latencies));

        if tr == 1 && c == 5
            fig1_data.t = (0:M-1)*Ts;
            fig1_data.cpu = cpu_load;
            fig1_data.lat = latencies;
            fig1_data.budget = T_budget_ms;
        end
    end
end

%% PART 2: Full Multi-Frequency Sweep (10, 20, 30, 40, 50 Hz) for BOTH Hardware Profiles
freqs = [10, 20, 30, 40, 50];
hw_profiles = [1.0, 2.4];
hw_names = {'High-Perf SoC (1.0x)', 'Constrained SoC (2.4x)'};

rmse_sweep = zeros(length(hw_profiles), length(controllers), length(freqs));
miss_sweep = zeros(length(hw_profiles), length(controllers), length(freqs));
lat_sweep  = zeros(length(hw_profiles), length(controllers), length(freqs));

fprintf('\n--- PART 2: Full Multi-Frequency Sweep (10 -> 50 Hz) across Hardware Profiles ---\n');
for h = 1:length(hw_profiles)
    hw_s = hw_profiles(h);
    for f_idx = 1:length(freqs)
        Ts_f = 1 / freqs(f_idx);
        T_bud = 0.60 * (Ts_f * 1000);
        [Ad_f, Bd_f] = quadrotor_model(Ts_f, 1.0);
        [K_f, P_f] = lqr_controller(Ad_f, Bd_f, Q, R);
        [x_ref, u_ref, ~] = generate_trajectories('lemniscate', Ts_f, 8.0);
        M_f = size(x_ref, 2);
        [cpu_f, wind_f] = simulate_workload(M_f, Ts_f, 500 + f_idx);

        for c = 1:length(controllers)
            x = x_ref(:, 1);
            u_prev = zeros(4, 1);
            state_mem.last_N = 10;
            err_sq = zeros(1, M_f);
            ms_arr = zeros(1, M_f);
            lt_arr = zeros(1, M_f);

            for k = 1:M_f
                win_end = min(M_f, k + 20);
                [du, info] = adaptive_mpc_solver(controllers{c}, x, x_ref(:, k:win_end), u_ref(:, k:win_end), u_prev, Ad_f, Bd_f, Q, R, P_f, K_f, T_bud, cpu_f(k), hw_s, state_mem);
                x = Ad_f * x + Bd_f * (u_ref(:, k) + du) + wind_f(:, k) * Ts_f;
                u_prev = du;
                state_mem.last_N = info.N_sel;
                err_sq(k) = sum((x(1:3) - x_ref(1:3, k)).^2);
                ms_arr(k) = info.deadline_missed;
                lt_arr(k) = info.t_calc_ms;
            end
            rmse_sweep(h, c, f_idx) = sqrt(mean(err_sq));
            miss_sweep(h, c, f_idx) = 100 * mean(ms_arr);
            lat_sweep(h, c, f_idx)  = mean(lt_arr);
        end
    end

    fprintf('\n>>> Hardware Profile: %s <<<\n', hw_names{h});
    fprintf('Table 2A - Position Tracking RMSE (m) from 10 Hz to 50 Hz:\n');
    fprintf('%-20s | %8s | %8s | %8s | %8s | %8s\n', 'Controller', '10 Hz', '20 Hz', '30 Hz', '40 Hz', '50 Hz');
    fprintf('-------------------------------------------------------------------------\n');
    for c = 1:length(controllers)
        fprintf('%-20s | %8.4f | %8.4f | %8.4f | %8.4f | %8.4f\n', ...
            ctrl_labels{c}, rmse_sweep(h, c, 1), rmse_sweep(h, c, 2), rmse_sweep(h, c, 3), rmse_sweep(h, c, 4), rmse_sweep(h, c, 5));
    end

    fprintf('\nTable 2B - Deadline Miss Rate (%%) from 10 Hz to 50 Hz:\n');
    fprintf('%-20s | %8s | %8s | %8s | %8s | %8s\n', 'Controller', '10 Hz', '20 Hz', '30 Hz', '40 Hz', '50 Hz');
    fprintf('-------------------------------------------------------------------------\n');
    for c = 1:length(controllers)
        fprintf('%-20s | %8.2f | %8.2f | %8.2f | %8.2f | %8.2f\n', ...
            ctrl_labels{c}, miss_sweep(h, c, 1), miss_sweep(h, c, 2), miss_sweep(h, c, 3), miss_sweep(h, c, 4), miss_sweep(h, c, 5));
    end
end

%% PART 3: 30-Trial Monte Carlo Robustness Test (Mass Uncertainty +/- 15%)
num_mc = 30;
mc_rmse = zeros(num_mc, length(controllers));
Ts_mc = 1 / 40;
T_bud_mc = 0.60 * (Ts_mc * 1000);
[Ad_nom, Bd_nom] = quadrotor_model(Ts_mc, 1.0);
[K_nom, P_nom] = lqr_controller(Ad_nom, Bd_nom, Q, R);

fprintf('\n--- PART 3: 30-Trial Monte Carlo Robustness (Delta m in [-15%%, +15%%]) ---\n');
for mc = 1:num_mc
    rng(1000 + mc);
    mass_scale = 0.85 + 0.30 * rand();
    [Ad_true, Bd_true] = quadrotor_model(Ts_mc, mass_scale);
    [x_ref, u_ref, ~] = generate_trajectories('lemniscate', Ts_mc, 6.0);
    M_mc = size(x_ref, 2);
    [cpu_mc, wind_mc] = simulate_workload(M_mc, Ts_mc, 2000 + mc);

    for c = 1:length(controllers)
        x = x_ref(:, 1);
        u_prev = zeros(4, 1);
        state_mem.last_N = 10;
        err_sq = zeros(1, M_mc);
        for k = 1:M_mc
            win_end = min(M_mc, k + 20);
            [du, info] = adaptive_mpc_solver(controllers{c}, x, x_ref(:, k:win_end), u_ref(:, k:win_end), u_prev, Ad_nom, Bd_nom, Q, R, P_nom, K_nom, T_bud_mc, cpu_mc(k), 2.4, state_mem);
            x = Ad_true * x + Bd_true * (u_ref(:, k) + du) + wind_mc(:, k) * Ts_mc;
            u_prev = du;
            state_mem.last_N = info.N_sel;
            err_sq(k) = sum((x(1:3) - x_ref(1:3, k)).^2);
        end
        mc_rmse(mc, c) = sqrt(mean(err_sq));
    end
end

for c = 1:length(controllers)
    fprintf('%-20s | Mean RMSE = %.4f m | Std = %.4f m\n', ...
        ctrl_labels{c}, mean(mc_rmse(:, c)), std(mc_rmse(:, c)));
end

%% Generate IEEE-Style Evaluation Figures & Export All as Vector PDFs (Fig_1.pdf - Fig_4.pdf)
colors = [0.25 0.55 0.85; 0.85 0.35 0.25; 0.90 0.65 0.15; 0.50 0.35 0.70; 0.15 0.68 0.38];
markers = {'o', 's', '^', 'd', 'p'};

% Fig 1: Real-time Latency vs Dynamic Budget under Bursty CPU Load (0 - 50s)
f1 = figure('Name', 'Fig 1: Latency vs Budget', 'Color', 'w', 'Position', [80, 80, 960, 400]);
yyaxis left
plot(fig1_data.t, fig1_data.lat, 'b-', 'LineWidth', 1.2); hold on;
yline(fig1_data.budget, 'r--', 'LineWidth', 1.5, 'Label', 'Time Budget (15 ms)');
ylabel('Solve Latency (ms)');
ylim([0, 18]);
yyaxis right
plot(fig1_data.t, fig1_data.cpu * 100, 'k:', 'LineWidth', 1.0);
ylabel('CPU Utilization (%)');
ylim([0, 105]);
xlabel('Time (s)');
xlim([0, T_sim]);
title('Fig 1: Proposed Adaptive MPC Latency Adherence under 50s Bursty CPU Load');
grid on;

% Fig 2: RMSE across Control Frequencies (10 - 50 Hz) for BOTH Hardware Profiles
f2 = figure('Name', 'Fig 2: Frequency Sweep RMSE (Both SoCs)', 'Color', 'w', 'Position', [100, 100, 1080, 440]);
for h = 1:2
    subplot(1, 2, h); hold on;
    for c = 1:length(controllers)
        plot(freqs, squeeze(rmse_sweep(h, c, :)), ['-' markers{c}], 'Color', colors(c, :), 'LineWidth', 1.6, 'MarkerSize', 6);
    end
    for c = 1:length(controllers)
        y_pos = 1.02 - (c - 1) * 0.065;
        plot([33, 36.5], [y_pos, y_pos], ['-' markers{c}], 'Color', colors(c, :), 'LineWidth', 1.6, 'MarkerSize', 5);
        text(37.2, y_pos, ctrl_labels{c}, 'FontSize', 8.5, 'VerticalAlignment', 'middle');
    end
    xlabel('Control Frequency (Hz)');
    ylabel('Position Tracking RMSE (m)');
    title(['Tracking RMSE: ', hw_names{h}]);
    ylim([0.25, 1.12]);
    grid on;
end

% Fig 3: Deadline Miss Rate across Control Frequencies (10 - 50 Hz) for BOTH Hardware Profiles
f3 = figure('Name', 'Fig 3: Deadline Miss Rate (Both SoCs)', 'Color', 'w', 'Position', [120, 120, 1080, 440]);
for h = 1:2
    subplot(1, 2, h); hold on;
    for c = 1:length(controllers)
        plot(freqs, squeeze(miss_sweep(h, c, :)), ['-' markers{c}], 'Color', colors(c, :), 'LineWidth', 1.6, 'MarkerSize', 6);
    end
    for c = 1:length(controllers)
        y_pos = 68 - (c - 1) * 6.5;
        plot([11, 14.5], [y_pos, y_pos], ['-' markers{c}], 'Color', colors(c, :), 'LineWidth', 1.6, 'MarkerSize', 5);
        text(15.2, y_pos, ctrl_labels{c}, 'FontSize', 8.5, 'VerticalAlignment', 'middle');
    end
    xlabel('Control Frequency (Hz)');
    ylabel('Deadline Miss Rate (%)');
    title(['Deadline Miss Rate: ', hw_names{h}]);
    ylim([-4, 105]);
    grid on;
end

% Fig 4: 30-Trial Monte Carlo Robustness (Pure-MATLAB Box-and-Whisker Plot)
f4 = figure('Name', 'Fig 4: Monte Carlo Robustness', 'Color', 'w', 'Position', [140, 140, 820, 440]);
hold on;
box_w = 0.42;
for c = 1:length(controllers)
    col_data = sort(mc_rmse(:, c));
    n_d = length(col_data);
    q25 = col_data(max(1, round(0.25 * n_d)));
    q50 = col_data(max(1, round(0.50 * n_d)));
    q75 = col_data(max(1, round(0.75 * n_d)));
    y_min = col_data(1);
    y_max = col_data(end);

    plot([c, c], [y_min, q25], 'k-', 'LineWidth', 1.2);
    plot([c, c], [q75, y_max], 'k-', 'LineWidth', 1.2);
    plot([c - box_w*0.4, c + box_w*0.4], [y_min, y_min], 'k-', 'LineWidth', 1.2);
    plot([c - box_w*0.4, c + box_w*0.4], [y_max, y_max], 'k-', 'LineWidth', 1.2);

    patch([c - box_w/2, c + box_w/2, c + box_w/2, c - box_w/2], ...
          [q25, q25, q75, q75], colors(c, :), 'FaceAlpha', 0.65, 'EdgeColor', 'k', 'LineWidth', 1.2);
    plot([c - box_w/2, c + box_w/2], [q50, q50], 'r-', 'LineWidth', 2.0);
end
set(gca, 'XTick', 1:length(controllers), 'XTickLabel', ctrl_labels);
xlim([0.4, length(controllers) + 0.6]);
ylabel('Position RMSE (m)');
title('Fig 4: 30-Trial Monte Carlo Robustness (\Delta m \in [-15%, +15%])');
grid on;

% Automatically crop paper bounds and export every figure as cropped vector PDF
fig_handles = [f1, f2, f3, f4];
for idx_fig = 1:length(fig_handles)
    fh = fig_handles(idx_fig);
    pdf_name = sprintf('Fig_%d.pdf', idx_fig);
    try
        exportgraphics(fh, pdf_name, 'ContentType', 'vector');
    catch
        set(fh, 'Units', 'inches');
        pos_in = get(fh, 'Position');
        set(fh, 'PaperPositionMode', 'auto', 'PaperUnits', 'inches', 'PaperSize', [pos_in(3), pos_in(4)]);
        print(fh, pdf_name, '-dpdf', '-painters');
    end
end

fprintf('\nSimulation complete. Exported Fig_1.pdf through Fig_4.pdf (Vector PDF).\n');
