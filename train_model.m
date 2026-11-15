function train_model()
% TRAIN_MODEL Trains the deployed Tiny-ML latency predictor (black-box) and
% compares it on a held-out TEST set (unseen rollouts) with a black-box linear
% regression. The latency formula is hidden from both.

if ~isfile('mpc_large_dataset.mat')
    generate_large_dataset();
end
d = load('mpc_large_dataset.mat');
X = d.X_raw; Y = d.Y_lat; G = d.G_roll;

[i_tr, i_va, i_te] = split_by_group(G, 0.70, 0.15, 42);
if ~isfile('arch_choice.mat')
    architecture_search();
end
a = load('arch_choice.mat');
mopts = struct('hidden', a.arch.hidden, 'log_target', a.arch.log_target, 'seed', 42);
[model, hist] = train_mlp(X(i_tr, :), Y(i_tr), X(i_va, :), Y(i_va), mopts);

base = latency_baselines(X(i_tr, :), Y(i_tr));
Yte = Y(i_te);
pred = {predict_mlp(model, X(i_te, :)), base.linear.predict(X(i_te, :))};
names = {'Tiny-ML MLP (black-box)', 'Linear regression (black-box)'};

fprintf('Latency predictor %s (log target %d, %d params): %d train / %d val / %d test samples (%d/%d/%d rollouts), best epoch %d\n', ...
    mat2str(model.hidden), model.log_target, model.n_params, numel(i_tr), numel(i_va), numel(i_te), numel(unique(G(i_tr))), numel(unique(G(i_va))), numel(unique(G(i_te))), hist.best_epoch);
fprintf('%-32s | %9s | %9s | %8s\n', 'Predictor (held-out test set)', 'MAE (ms)', 'RMSE (ms)', 'R^2');
for k = 1:2
    e = pred{k} - Yte;
    fprintf('%-32s | %9.4f | %9.4f | %8.4f\n', names{k}, mean(abs(e)), sqrt(mean(e.^2)), 1 - sum(e.^2) / sum((Yte - mean(Yte)).^2));
end

% Data-driven safety factor for deadline checks: 95th percentile of the ratio
% (true latency / predicted latency) on the VALIDATION rollouts (never the test
% set). The supervisor accepts a candidate only if margin * t_hat <= budget.
ratio = Y(i_va) ./ max(predict_mlp(model, X(i_va, :)), 1e-6);
model.margin = max(1, quantile_lin(ratio, 0.95));
fprintf('Deadline safety factor (95th pct. of true/predicted on validation): %.3f\n', model.margin);

e = pred{1} - Yte;
model.test_mae = mean(abs(e));
model.test_r2  = 1 - sum(e.^2) / sum((Yte - mean(Yte)).^2);
model.hist = hist;
save('trained_predictor.mat', 'model');
fprintf('Saved trained weights to trained_predictor.mat\n');
end

function q = quantile_lin(v, p)
% Linear-interpolation quantile (type 7, same as the MATLAB / R default).
v = sort(v(:)); n = numel(v);
h = (n - 1) * p + 1; lo = floor(h); hi = min(n, lo + 1);
q = v(lo) + (h - lo) * (v(hi) - v(lo));
end
