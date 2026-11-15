function cfg = experiment_config()
% EXPERIMENT_CONFIG Every experiment setting and the list of independent
% benchmark cases (one case = one closed-loop setting, all controllers, n_seeds
% paired realisations). Cases are distributed over n_jobs CI jobs (bench_job.m).
%
% Grid:
%   A  full factorial  : 3 trajectories x 6 scenario levels x 4 control rates
%                        x 3 mean CPU loads, disturbance 0.5 m/s^2, tight weights
%   B  disturbance     : 3 trajectories x 6 levels x disturbance {0, 1.0} m/s^2
%                        (50 Hz, CPU 50 %)
%   C  weights         : 3 trajectories x 6 levels with the 'loose' Bryson weights
%                        (50 Hz, CPU 50 %, disturbance 0.5 m/s^2)

cfg.T_sim = 30.0;
cfg.preview_pad = 20;           % >= largest candidate horizon (16)
cfg.controllers = {'fixed_5', 'fixed_10', 'nominal_max', 'proposed'};
cfg.labels = {'Fixed-MPC (N=5)', 'Fixed-MPC (N=10)', 'Nominal-model scheduler', 'Proposed (Tiny-ML scheduler)'};
cfg.n_seeds = 30;               % independent disturbance / CPU-load realisations per case
cfg.seed_stride = 1000003;      % seed of repetition r = case seed + (r-1) * seed_stride
cfg.n_jobs = 60;                % CI jobs (cases are dealt round-robin to jobs)
cfg.trajectories = {'lemniscate', 'helix', 'racing'};
cfg.levels = {'S1 nominal', 'S2 half-way', 'S3 at limit', 'S4 beyond 10%', 'S5 beyond 20%', 'S6 beyond 30%'};
cfg.freqs = [25, 50, 75, 100];
cfg.cpu_set = [0.2, 0.5, 0.8];
cfg.dist_set = [0, 0.5, 1.0];
cfg.freq0 = 50; cfg.cpu0 = 0.5; cfg.dist0 = 0.5;
nL = numel(cfg.levels);

c = struct('part', {}, 'traj', {}, 'level', {}, 'freq', {}, 'cpu', {}, 'dist', {}, 'weights', {}, 'seed', {});
add = @(c, part, tr, s, f, u, d, w) [c, struct('part', part, 'traj', tr, 'level', s, 'freq', f, ...
    'cpu', u, 'dist', d, 'weights', w, 'seed', 0)];
for tr = 1:3
    for s = 1:nL
        for f = cfg.freqs
            for u = cfg.cpu_set
                c = add(c, 'A', tr, s, f, u, cfg.dist0, 'tight');
            end
        end
    end
end
for tr = 1:3
    for s = 1:nL
        for d = [0, 1.0]
            c = add(c, 'B', tr, s, cfg.freq0, cfg.cpu0, d, 'tight');
        end
    end
end
for tr = 1:3
    for s = 1:nL
        c = add(c, 'C', tr, s, cfg.freq0, cfg.cpu0, cfg.dist0, 'loose');
    end
end
for i = 1:numel(c)
    c(i).seed = 100000 + 97 * i;   % distinct per case; shared by all controllers of the case
end
cfg.cases = c;
cfg.n_cases = numel(c);
end
