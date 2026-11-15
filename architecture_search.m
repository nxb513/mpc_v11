function arch = architecture_search()
% ARCHITECTURE_SEARCH Selection of the Tiny-ML architecture by a fixed rule.
% Uses only the training and validation rollouts (never the test set).
%
% Candidate space (fully connected ReLU MLPs, widths halving with depth):
%   depth 1: [8], [16], [32], [64]
%   depth 2: [16 8], [32 16], [64 32]
%   depth 3: [32 16 8], [64 32 16]
%   target : latency y (ms)  or  log(y)
% Each candidate is trained with SEEDS different initialisations.
%
% Selection rule: among all candidates whose mean validation MAE is within TOL
% of the best mean validation MAE, choose the one with the fewest parameters
% (ties: fewer flops).

TOL = 0.05;
SEEDS = 3;
cands = {[8], [16], [32], [64], [16 8], [32 16], [64 32], [32 16 8], [64 32 16]};

d = load('mpc_large_dataset.mat');
X = d.X_raw; Y = d.Y_lat; G = d.G_roll;
[i_tr, i_va] = split_by_group(G, 0.70, 0.15, 42);

rows = struct('hidden', {}, 'log_target', {}, 'val_mae', {}, 'val_std', {}, 'n_params', {}, 'flops', {});
fprintf('\n--- Tiny-ML architecture search (%d candidates x 2 targets x %d seeds, validation rollouts only) ---\n', numel(cands), SEEDS);
fprintf('%-14s | %-6s | %8s | %16s | %6s\n', 'Hidden', 'Target', 'Params', 'Val MAE (ms)', 'Flops');
for lt = [false, true]
    for c = 1:numel(cands)
        v = zeros(SEEDS, 1);
        for s = 1:SEEDS
            [mdl, ~] = train_mlp(X(i_tr, :), Y(i_tr), X(i_va, :), Y(i_va), ...
                struct('hidden', cands{c}, 'log_target', lt, 'seed', 100 + s));
            v(s) = mean(abs(predict_mlp(mdl, X(i_va, :)) - Y(i_va)));
        end
        r = numel(rows) + 1;
        rows(r).hidden = cands{c}; rows(r).log_target = lt;
        rows(r).val_mae = mean(v); rows(r).val_std = std(v);
        rows(r).n_params = mdl.n_params; rows(r).flops = mdl.flops;
        tgt = 'y'; if lt, tgt = 'log y'; end
        fprintf('%-14s | %-6s | %8d | %7.4f +/- %6.4f | %6d\n', mat2str(cands{c}), tgt, mdl.n_params, mean(v), std(v), mdl.flops);
    end
end

vm = [rows.val_mae];
ok = find(vm <= (1 + TOL) * min(vm));
[~, o] = sortrows([[rows(ok).n_params]', [rows(ok).flops]']);
k = ok(o(1));
arch.hidden = rows(k).hidden;
arch.log_target = rows(k).log_target;
arch.table = rows;
arch.tol = TOL;
arch.best_val_mae = min(vm);
tgt = 'y'; if arch.log_target, tgt = 'log y'; end
fprintf('Selected: hidden %s, target %s, %d parameters, %d flops (val MAE %.4f ms; best %.4f ms; tolerance %.0f%%)\n', ...
    mat2str(arch.hidden), tgt, rows(k).n_params, rows(k).flops, rows(k).val_mae, min(vm), 100 * TOL);
save('arch_choice.mat', 'arch');
end
