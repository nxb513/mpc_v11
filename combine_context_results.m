function res = combine_context_results(in_dir)
% COMBINE_CONTEXT_RESULTS Gathers results/context_case_<k>.mat into one struct,
% saves context_study.mat and draws Fig_7.pdf (plot_context_study.m).
if nargin < 1, in_dir = 'results'; end
ctx = context_list();
K = numel(ctx);
for k = 1:K
    f = fullfile(in_dir, sprintf('context_case_%d.mat', k));
    s = load(f); r = s.r;
    if k == 1
        nS = numel(r.sizes); R = size(r.rel_mlp, 2);
        res.ctx = ctx; res.sizes = r.sizes; res.sat_tol = r.sat_tol;
        res.rel_mlp = nan(K, nS, R); res.rel_lin = nan(K, nS, R);
        res.n_sat = nan(K, 1); res.r2 = nan(K, 1); res.mae = nan(K, 1); res.t_mean = nan(K, 1);
    end
    res.rel_mlp(k, :, :) = r.rel_mlp; res.rel_lin(k, :, :) = r.rel_lin;
    res.n_sat(k) = r.n_sat; res.r2(k) = r.r2; res.mae(k) = r.mae; res.t_mean(k) = r.t_mean;
end
fprintf('\n%-26s | %9s | %7s | %16s | %14s | %7s\n', 'Context', 'mean t', 'n_sat', 'Tiny-ML rel. MAE', 'linear rel. MAE', 'R^2');
for k = 1:K
    mu = mean(res.rel_mlp(k, :, :), 3); ml = mean(res.rel_lin(k, :, :), 3);
    last = find(~isnan(mu), 1, 'last');
    fprintf('%-26s | %6.2f ms | %7d | %14.2f %% | %12.2f %% | %7.4f\n', ctx{k}.label, res.t_mean(k), ...
        res.n_sat(k), mu(last), ml(last), res.r2(k));
end
save('context_study.mat', 'res');
plot_context_study(res);
end
