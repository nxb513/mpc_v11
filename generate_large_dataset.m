function data = generate_large_dataset(opts)
% GENERATE_LARGE_DATASET Latency dataset from CLOSED-LOOP rollouts of the LTV-MPC
% on the nonlinear quadrotor (same plant, references, QP and timing model as the
% benchmark).
%
% At every control step a random (N, I_max) pair is drawn from the controllers'
% candidate sets, the QP is solved, and the features observed BEFORE the solve,
%   [N, I_max, cpu_load, ||e_pos||, ||e_vel||],
% are stored with the resulting latency.
%
% BLACK-BOX PROTOCOL: the latency generator (true_latency.m, hidden parameters in
% latency_context.m) is only called here. The learner (train_mlp.m) receives
% nothing but X_raw / Y_lat. G_roll stores the rollout id of every sample.
%
% Each rollout draws: a trajectory shape, a scenario level S1..S4
% (scenario_levels.m), a control frequency, a mean CPU load and a disturbance
% level from the experiment grids below.
%
% opts (all optional): n_samples (24000), seed (2026), seed_workload (9000),
%   out_file ('mpc_large_dataset.mat'; '' = do not save), traj_set, freq_set,
%   cpu_set, dist_set, dist_scale (1), init_err (0.3), latency ('nominal' or a
%   latency_context struct), verbose (true)

if nargin < 1, opts = struct(); end
def = struct('n_samples', 24000, 'seed', 2026, 'seed_workload', 9000, ...
    'out_file', 'mpc_large_dataset.mat', 'traj_set', {{'lemniscate', 'helix', 'racing'}}, ...
    'freq_set', [20, 40, 60, 80, 100], 'cpu_set', [0.2, 0.5, 0.8], 'dist_set', [0, 0.5, 1.0], ...
    'dist_scale', 1.0, 'init_err', 0.3, 'latency', 'nominal', 'verbose', true);
f = fieldnames(def);
for k = 1:numel(f)
    if ~isfield(opts, f{k}), opts.(f{k}) = def.(f{k}); end
end

prev_ctx = latency_context();
latency_context(opts.latency);
c = latency_context();
if opts.verbose
    fprintf('Generating closed-loop QP latency dataset (latency context: %s)...\n', c.name);
end
rng(opts.seed);

pq = quad_params();
[Q, R] = mpc_weights('tight', pq);
N_set    = [4, 6, 8, 10, 12, 16];
iter_set = [10, 20, 40];
num_samples = opts.n_samples;
T_roll = 6.0;
X_raw = zeros(num_samples, 5);
Y_lat = zeros(num_samples, 1);
G_roll = zeros(num_samples, 1);

i = 0; r = 0;
while i < num_samples
    r = r + 1;
    freq = opts.freq_set(randi(numel(opts.freq_set)));
    Ts = 1 / freq;
    T_bud = Ts * 1000;
    [Ad, Bd] = quadrotor_model(Ts, 1.0);
    [K_lqr, P_term] = lqr_controller(Ad, Bd, Q, R);

    traj = opts.traj_set{randi(numel(opts.traj_set))};
    SL = scenario_levels(traj);
    lev = SL(randi(numel(SL)));
    t_off = 30 * rand();            % random phase: rollouts do not replay the evaluation runs
    [x_full, u_full] = generate_trajectories(traj, Ts, t_off + T_roll + 20 * Ts, lev.aggr, false);
    k0 = round(t_off / Ts) + 1;
    x_ref = x_full(:, k0:end); u_ref = u_full(:, k0:end);
    trj = ltv_model(x_ref, u_ref, Ts, pq);
    M = round(T_roll / Ts) + 1;
    cpu_mean = opts.cpu_set(randi(numel(opts.cpu_set)));
    dist_std = opts.dist_scale * opts.dist_set(randi(numel(opts.dist_set)));
    [cpu_load, a_dist] = simulate_workload(M, Ts, opts.seed_workload + r, cpu_mean, dist_std);
    rng(opts.seed + 17 * r);

    x = x_ref(:, 1) + [opts.init_err * randn(3, 1); opts.init_err * randn(3, 1); 0.02 * randn(6, 1)];
    mem = [];
    for k = 1:M-1
        N  = N_set(randi(numel(N_set)));
        It = iter_set(randi(numel(iter_set)));
        e0 = x - x_ref(:, k);
        [fr, info, mem] = adaptive_mpc_solver(sprintf('dataset_%d_%d', N, It), x, k, trj, Q, R, P_term, K_lqr, T_bud, cpu_load(k), mem);
        i = i + 1;
        X_raw(i, :) = [N, It, cpu_load(k), norm(e0(1:3)), norm(e0(4:6))];
        Y_lat(i)    = info.t_calc_ms;
        G_roll(i)   = r;
        if i >= num_samples, break; end
        x = quad_nl_step(x, fr, Ts, pq, a_dist(:, k));
        if ~all(isfinite(x)) || norm(x(1:3) - x_ref(1:3, k+1)) > 5
            break;   % diverged rollout: keep the samples collected so far
        end
    end
end

latency_context(prev_ctx);
data.X_raw = X_raw; data.Y_lat = Y_lat; data.G_roll = G_roll; data.context = c; data.opts = opts;
if ~isempty(opts.out_file)
    save(opts.out_file, 'X_raw', 'Y_lat', 'G_roll');
end
if opts.verbose
    fprintf('Collected %d closed-loop samples from %d rollouts\n', num_samples, r);
end
end
