function out = bench_case(id, out_dir)
% BENCH_CASE Runs ONE benchmark case of experiment_config.m (all controllers) and
% saves results/bench_case_<id>.mat. Needs trained_predictor.mat (Tiny-ML).
% Independent of every other case, so the cases run as parallel CI jobs.

if nargin < 2, out_dir = 'results'; end
if ischar(id) || isstring(id), id = str2double(id); end
if ~isfolder(out_dir), mkdir(out_dir); end
cfg = experiment_config();
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
[cpu, ad] = simulate_workload(M, Ts, cs.seed, cs.cpu, cs.dist);

fprintf('Case %d (part %d): %s, %s, %d Hz, CPU %.0f%%, disturbance %.1f m/s^2, %s weights\n', ...
    id, cs.part, traj, cfg.levels{cs.level}, cs.freq, 100 * cs.cpu, cs.dist, cs.weights);
fprintf('  reference: speed max %.1f m/s, tilt max %.0f deg, beyond rotor limits %.1f%% of the time\n', ...
    st.max, st.tilt_max_deg, st.ff_over);
fprintf('  %-26s | %8s | %8s | %10s | %6s | %8s | %8s\n', 'Controller', 'RMSE (m)', 'Miss (%)', 'CPU (ms/s)', 'mean N', 'shift %', 'LQR %');

nC = numel(cfg.controllers);
r = struct('rmse', nan(1, nC), 'crash', false(1, nC), 'miss', nan(1, nC), 'cpu_ms_per_s', nan(1, nC), ...
    'N_mean', nan(1, nC), 'shift', nan(1, nC), 'lqr', nan(1, nC));
traces = cell(1, nC);
for c = 1:nC
    o = run_episode(cfg.controllers{c}, trj, M, cpu, ad, Q, R, P_term, K_lqr, T_bud);
    r.rmse(c) = o.rmse; r.crash(c) = o.crashed; r.miss(c) = o.miss; r.cpu_ms_per_s(c) = o.cpu_ms_per_s;
    r.N_mean(c) = o.N_mean; r.shift(c) = o.fallback_shift; r.lqr(c) = o.fallback_lqr;
    traces{c} = struct('lat', o.lat, 'N', o.Nsel, 'n_done', o.n_done);
    if o.crashed, rs = '   CRASH'; else, rs = sprintf('%8.4f', o.rmse); end
    fprintf('  %-26s | %s | %8.2f | %10.1f | %6.2f | %8.2f | %8.2f\n', cfg.labels{c}, rs, o.miss, ...
        o.cpu_ms_per_s, o.N_mean, o.fallback_shift, o.fallback_lqr);
end
kk = 1:M-1;
ff = p.Mix_inv * (trj.u_ref(:, kk) + p.u_hover);
out.case = cs; out.id = id; out.res = r; out.ref_stats = st;
out.traces = traces; out.t = (0:M-2) * Ts; out.cpu = cpu(1:M-1); out.budget = T_bud;
out.ref_over = any(ff > p.T_max | ff < p.T_min, 1);
save(fullfile(out_dir, sprintf('bench_case_%02d.mat', id)), 'out');
end
