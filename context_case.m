function r = context_case(k, out_dir, quick)
% CONTEXT_CASE One latency context of the Tiny-ML study (see context_study.m):
% generate data with hidden latency formula k, retrain the fixed architecture
% from scratch on n = 25 ... 51200 samples (3 repeats), save
% results/context_case_<k>.mat. Independent of the other contexts (one CI job each).
% Needs arch_choice.mat.

if nargin < 2 || isempty(out_dir), out_dir = 'results'; end
if nargin < 3, quick = false; end   % quick = true: tiny smoke-test version
if ischar(k) || isstring(k), k = str2double(k); end
if ~isfolder(out_dir), mkdir(out_dir); end

SAT_TOL = 0.05;
R = 3;
sizes = [25, 50, 100, 200, 400, 800, 1600, 3200, 6400, 12800, 25600, 51200];
n_ctx_samples = 76000;          % 70 % pool >= 51200 training samples
if quick, sizes = sizes(1:5); n_ctx_samples = 1500; R = 2; end
ctx = context_list();
c = ctx{k};

ac = load('arch_choice.mat');
mo = @(seed) struct('hidden', ac.arch.hidden, 'log_target', ac.arch.log_target, 'seed', seed);

o = c.fl;
o.n_samples = n_ctx_samples; o.seed = 30000 + 100 * k; o.seed_workload = 40000 + 1000 * k;
o.out_file = ''; o.latency = c.lat; o.verbose = false;
D = generate_large_dataset(o);
[i_tr, i_va, i_te] = split_by_group(D.G_roll, 0.70, 0.15, 42 + k);
Xte = D.X_raw(i_te, :); Yte = D.Y_lat(i_te);
rel = @(yhat) 100 * mean(abs(yhat - Yte)) / mean(Yte);

rel_mlp = nan(numel(sizes), R); rel_lin = nan(numel(sizes), R);
r2_full = NaN; mae_full = NaN;
sz = sizes(sizes <= numel(i_tr));
for s = 1:numel(sz)
    for rr = 1:R
        rng(1000 * k + 10 * s + rr);
        sub = i_tr(randperm(numel(i_tr), sz(s)));
        mdl = train_mlp(D.X_raw(sub, :), D.Y_lat(sub), D.X_raw(i_va, :), D.Y_lat(i_va), mo(1000 * k + 10 * s + rr));
        rel_mlp(s, rr) = rel(predict_mlp(mdl, Xte));
        bl = latency_baselines(D.X_raw(sub, :), D.Y_lat(sub));
        rel_lin(s, rr) = rel(bl.linear.predict(Xte));
        if s == numel(sz) && rr == 1
            e = predict_mlp(mdl, Xte) - Yte;
            r2_full = 1 - sum(e.^2) / sum((Yte - mean(Yte)).^2);
            mae_full = mean(abs(e));
        end
    end
end
m = mean(rel_mlp(1:numel(sz), :), 2);
if numel(m) >= 2 && m(end-1) <= (1 + SAT_TOL) * m(end)
    n_sat = sz(find(m <= (1 + SAT_TOL) * m(end), 1, 'first'));
else
    n_sat = NaN;   % still improving at n_max: not saturated
end
fprintf('%-26s | mean t %6.2f ms | n_sat = %5d | rel. MAE at n_max %6.2f %% (linear %6.2f %%) | R^2 %.4f\n', ...
    c.label, mean(Yte), n_sat, m(end), mean(rel_lin(numel(sz), :)), r2_full);

r = struct('k', k, 'label', c.label, 'sizes', sizes, 'rel_mlp', rel_mlp, 'rel_lin', rel_lin, ...
    'n_sat', n_sat, 'r2', r2_full, 'mae', mae_full, 't_mean', mean(Yte), 'sat_tol', SAT_TOL);
save(fullfile(out_dir, sprintf('context_case_%d.mat', k)), 'r');
end
