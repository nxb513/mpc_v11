function out = bench_case(id, out_dir, n_seeds)
% BENCH_CASE Runs ONE benchmark case of experiment_config.m: all controllers on
% cfg.n_seeds independent realisations of the disturbance and CPU load (the SAME
% realisations for every controller, so results are paired). Saves
% results/bench_case_<id>.mat. Needs trained_predictor.mat (Tiny-ML).
% Independent of every other case, so the cases run as parallel CI jobs.

if nargin < 2 || isempty(out_dir), out_dir = 'results'; end
if ischar(id) || isstring(id), id = str2double(id); end
if ~isfolder(out_dir), mkdir(out_dir); end
cfg = experiment_config();
if nargin < 3 || isempty(n_seeds), n_seeds = cfg.n_seeds; end
cs = cfg.cases(id);
p = quad_params();
[Q, R] = mpc_weights(cs.weights, p);

Ts = 1 / cs.freq; T_bud = 1000 * Ts; M = round(cfg.T_sim / Ts) + 1;
[Ad, Bd] = quadrotor_model(Ts, 1.0);
[K_lqr, P_term] = lqr_controller(Ad, Bd, Q, R);
traj = cfg.trajectories{cs.traj};
S = scenario_levels(traj);
[xr, ur, st] = generate_trajectories(traj, Ts, cfg.T_sim + cfg.preview_pad * Ts, S(cs.level).aggr, false);
trj = ltv_model(xr, ur, Ts, p);

fprintf('Case %d (part %s): %s, %s, %d Hz, CPU %.0f%%, disturbance %.1f m/s^2, %s weights, %d seeds\n', ...
    id, cs.part, traj, cfg.levels{cs.level}, cs.freq, 100 * cs.cpu, cs.dist, cs.weights, n_seeds);
fprintf('  reference: speed max %.1f m/s, tilt max %.0f deg, beyond rotor limits %.1f%% of the time\n', ...
    st.max, st.tilt_max_deg, st.ff_over);

nC = numel(cfg.controllers);
z = nan(n_seeds, nC);
r = struct('rmse', z, 'crash', false(n_seeds, nC), 'miss', z, 'cpu_ms_per_s', z, 'N_mean', z, ...
    'shift', z, 'lqr', z, 'seeds', cs.seed + (0:n_seeds-1)' * cfg.seed_stride);
traces = cell(1, nC);
for sd = 1:n_seeds
    [cpu, ad] = simulate_workload(M, Ts, r.seeds(sd), cs.cpu, cs.dist);
    for c = 1:nC
        o = run_episode(cfg.controllers{c}, trj, M, cpu, ad, Q, R, P_term, K_lqr, T_bud);
        r.rmse(sd, c) = o.rmse; r.crash(sd, c) = o.crashed; r.miss(sd, c) = o.miss;
        r.cpu_ms_per_s(sd, c) = o.cpu_ms_per_s; r.N_mean(sd, c) = o.N_mean;
        r.shift(sd, c) = o.fallback_shift; r.lqr(sd, c) = o.fallback_lqr;
        if sd == 1
            traces{c} = struct('lat', o.lat, 'N', o.Nsel, 'n_done', o.n_done, 'cpu', cpu(1:M-1));
        end
    end
end

fprintf('  %-30s | %-18s | %7s | %8s | %10s | %6s\n', 'Controller', 'RMSE (m) mean+-sd', 'crashes', 'miss (%)', 'CPU (ms/s)', 'mean N');
for c = 1:nC
    ok = ~r.crash(:, c);
    fprintf('  %-30s | %8.4f +- %6.4f | %4d/%-2d | %8.2f | %10.1f | %6.2f\n', cfg.labels{c}, ...
        mean(r.rmse(ok, c)), std(r.rmse(ok, c)), sum(~ok), n_seeds, mean(r.miss(:, c)), ...
        mean(r.cpu_ms_per_s(:, c)), mean(r.N_mean(:, c)));
end

kk = 1:M-1;
ff = p.Mix_inv * (trj.u_ref(:, kk) + p.u_hover);
out.case = cs; out.id = id; out.res = r; out.ref_stats = st;
out.traces = traces; out.t = (0:M-2) * Ts; out.budget = T_bud;
out.ref_over = any(ff > p.T_max | ff < p.T_min, 1);
save(fullfile(out_dir, sprintf('bench_case_%03d.mat', id)), 'out');
end
