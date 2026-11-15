function lc = learning_curve()
% LEARNING_CURVE Learning curves of the black-box Tiny-ML latency predictor.
%
% (a) Sample-size curve: the MLP is trained on random subsets of the training
%     rollouts' samples (n = 25 ... all), R repeats per size (different subset
%     and initialisation), and evaluated on the fixed held-out test rollouts.
%     The same subsets are used to fit the black-box linear baseline.
% (b) Epoch curve: train / validation Huber loss of the full-data model vs epoch.
%
% Saturation: the smallest n whose mean test MAE is within SAT_TOL of the mean
% test MAE obtained with all training data.
% Exports Fig_4.pdf (sample-size curve) and Fig_5.pdf (epoch curve).

SAT_TOL = 0.05;
R = 5;

d = load('mpc_large_dataset.mat');
ac = load('arch_choice.mat');
mo = @(seed) struct('hidden', ac.arch.hidden, 'log_target', ac.arch.log_target, 'seed', seed);
X = d.X_raw; Y = d.Y_lat; G = d.G_roll;
[i_tr, i_va, i_te] = split_by_group(G, 0.70, 0.15, 42);
n_all = numel(i_tr);
sizes = unique([25, 50, 100, 200, 400, 800, 1600, 3200, 6400, 12800, n_all]);
sizes = sizes(sizes <= n_all);

mae = nan(numel(sizes), R, 2);   % (:, :, 1) MLP, 2 linear
Xte = X(i_te, :); Yte = Y(i_te);
fprintf('\n--- Tiny-ML learning curve (%d repeats per size, test = %d samples from unseen rollouts) ---\n', R, numel(i_te));
for s = 1:numel(sizes)
    for r = 1:R
        rng(1000 * s + r);
        sub = i_tr(randperm(n_all, sizes(s)));
        mdl = train_mlp(X(sub, :), Y(sub), X(i_va, :), Y(i_va), mo(1000 * s + r));
        mae(s, r, 1) = mean(abs(predict_mlp(mdl, Xte) - Yte));
        b = latency_baselines(X(sub, :), Y(sub));
        mae(s, r, 2) = mean(abs(b.linear.predict(Xte) - Yte));
    end
    fprintf('n = %5d | MLP MAE %.4f +/- %.4f ms | linear %.4f ms\n', sizes(s), ...
        mean(mae(s, :, 1)), std(mae(s, :, 1)), mean(mae(s, :, 2)));
end

m_mlp = mean(mae(:, :, 1), 2);
k_sat = find(m_mlp <= (1 + SAT_TOL) * m_mlp(end), 1, 'first');
lc.sizes = sizes; lc.mae = mae; lc.n_sat = sizes(k_sat); lc.sat_tol = SAT_TOL;
fprintf('Saturation: MAE within %.0f%% of the full-data value from n = %d training samples on.\n', 100 * SAT_TOL, lc.n_sat);

% (b) Epoch curve of the full-data model
[~, hist] = train_mlp(X(i_tr, :), Y(i_tr), X(i_va, :), Y(i_va), mo(42));
lc.hist = hist;
fprintf('Epoch curve: best validation MAE %.4f ms at epoch %d (stopped at %d).\n', ...
    min(hist.val_mae), hist.best_epoch, numel(hist.val_mae));

save('learning_curve.mat', 'lc');
plot_learning_curve(lc);
end
