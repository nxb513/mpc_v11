function cfg = experiment_config()
% EXPERIMENT_CONFIG Single place for every experiment setting and the list of
% independent benchmark cases (one case = one closed-loop setting, all
% controllers). Used by bench_case.m (one CI job per case), report_results.m and
% main_simulation.m (sequential local run).

cfg.T_sim = 30.0;
cfg.preview_pad = 20;           % >= largest candidate horizon (16)
cfg.freq0 = 50;                 % nominal control rate [Hz] (middle of the 20-100 Hz sweep)
cfg.cpu0 = 0.5;                 % nominal mean background CPU load
cfg.dist0 = 0.5;                % nominal disturbance std [m/s^2]
cfg.controllers = {'fixed_5', 'fixed_10', 'nominal_max', 'proposed'};
cfg.labels = {'Fixed-MPC (N=5)', 'Fixed-MPC (N=10)', 'Nominal-model scheduler', 'Proposed (Tiny-ML scheduler)'};
cfg.n_seeds = 20;               % independent disturbance / CPU-load realisations per case
cfg.seed_stride = 1000;         % seed of repetition r = case seed + (r-1) * seed_stride
cfg.trajectories = {'lemniscate', 'helix', 'racing'};
cfg.levels = {'S1 nominal', 'S2 at limit', 'S3 beyond 10%', 'S4 beyond 30%'};
cfg.freqs = [20, 40, 60, 80, 100];
cfg.cpu_set = [0.2, 0.5, 0.8];
cfg.dist_set = [0, 0.5, 1.0];

% ---- case list ------------------------------------------------------------------
c = struct('part', {}, 'traj', {}, 'level', {}, 'freq', {}, 'cpu', {}, 'dist', {}, 'weights', {}, 'seed', {});
for tr = 1:3                                   % Part 1: scenario ladder
    for s = 1:4
        c(end+1) = struct('part', 1, 'traj', tr, 'level', s, 'freq', cfg.freq0, 'cpu', cfg.cpu0, ...
            'dist', cfg.dist0, 'weights', 'tight', 'seed', 100 + tr); %#ok<AGROW>
    end
end
for fi = 1:numel(cfg.freqs)                    % Part 2: control-rate sweep at S3
    c(end+1) = struct('part', 2, 'traj', 1, 'level', 3, 'freq', cfg.freqs(fi), 'cpu', cfg.cpu0, ...
        'dist', cfg.dist0, 'weights', 'tight', 'seed', 500 + fi); %#ok<AGROW>
end
for i = 1:3                                    % Part 3: CPU load x disturbance at S3
    for j = 1:3
        c(end+1) = struct('part', 3, 'traj', 1, 'level', 3, 'freq', cfg.freq0, 'cpu', cfg.cpu_set(i), ...
            'dist', cfg.dist_set(j), 'weights', 'tight', 'seed', 700 + 10 * i + j); %#ok<AGROW>
    end
end
for tr = 1:3                                   % Part 4: loose Bryson weights at S3
    c(end+1) = struct('part', 4, 'traj', tr, 'level', 3, 'freq', cfg.freq0, 'cpu', cfg.cpu0, ...
        'dist', cfg.dist0, 'weights', 'loose', 'seed', 100 + tr); %#ok<AGROW>
end
cfg.cases = c;
cfg.n_cases = numel(c);
end
