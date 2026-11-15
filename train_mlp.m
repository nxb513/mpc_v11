function [model, hist] = train_mlp(X_tr, Y_tr, X_va, Y_va, opts)
% TRAIN_MLP Black-box regression of QP latency with a fully connected ReLU MLP.
% The learner sees ONLY input/target pairs (X, Y); it has no access to the
% latency generator or its formula.
%
% Architecture: in -> hidden(1) -> ... -> hidden(L) -> 1, ReLU on hidden layers,
% linear output. Optional log target: the network fits z = log(y) and predicts
% exp(z) (latency is positive and its factors combine multiplicatively).
% Training: Huber (Smooth-L1) loss on the fitted target, Adam, L2 weight decay,
% mini-batches, early stopping on the validation MAE in ms (best weights restored).
%
% QUANTILE MODE: opts.quantile = tau (e.g. 0.95) replaces the Huber loss by the
% pinball loss, so the network predicts the tau-quantile of the latency (an upper
% bound for deadline checks); early stopping then uses the validation pinball loss.
%
% opts (all optional): hidden (default [32 16]), log_target (default false), quantile,
%   seed, max_steps, batch_size, lr, lambda_l2, delta_huber, patience (epochs)
% hist: per-epoch train/val loss (on the fitted target), val MAE [ms], best_epoch, steps.

if nargin < 5, opts = struct(); end
o = local_defaults(opts);
rng(o.seed);

n_tr = size(X_tr, 1);
mu_X  = mean(X_tr, 1);
sig_X = std(X_tr, 0, 1);
sig_X(sig_X < 1e-8) = 1;
Xn_tr = (X_tr - mu_X) ./ sig_X;
Xn_va = (X_va - mu_X) ./ sig_X;

if o.log_target
    T_tr = log(max(Y_tr, 1e-6)); T_va = log(max(Y_va, 1e-6));
    delta = o.delta_huber_log;
else
    T_tr = Y_tr; T_va = Y_va;
    delta = o.delta_huber;
end

dims = [size(X_tr, 2), o.hidden(:)', 1];
L = numel(dims) - 1;
W = cell(L, 1); b = cell(L, 1);
for l = 1:L
    W{l} = randn(dims(l), dims(l+1)) * sqrt(2 / dims(l));   % He initialisation
    b{l} = zeros(1, dims(l+1));
end
b{L} = mean(T_tr);
mW = cellfun(@(p) zeros(size(p)), W, 'UniformOutput', false); vW = mW;
mb = cellfun(@(p) zeros(size(p)), b, 'UniformOutput', false); vb = mb;
b1 = 0.9; b2 = 0.999; eps_a = 1e-8;

bs = min(o.batch_size, n_tr);
max_epochs = max(1, ceil(o.max_steps / ceil(n_tr / bs)));
hist.train_loss = nan(max_epochs, 1); hist.val_loss = hist.train_loss; hist.val_mae = hist.train_loss;
best = inf; bestW = W; bestb = b; best_ep = 0; step = 0; since = 0;

for ep = 1:max_epochs
    perm = randperm(n_tr);
    for s0 = 1:bs:n_tr
        step = step + 1;
        idx = perm(s0:min(s0 + bs - 1, n_tr));
        A = cell(L + 1, 1); Z = cell(L, 1);
        A{1} = Xn_tr(idx, :);
        for l = 1:L
            Z{l} = A{l} * W{l} + b{l};
            if l < L, A{l+1} = max(0, Z{l}); else, A{l+1} = Z{l}; end
        end
        err = A{L+1} - T_tr(idx);
        if isempty(o.quantile)
            dZ = max(-delta, min(delta, err)) / numel(idx);            % Huber gradient
        else
            dZ = ((err > 0) * (1 - o.quantile) - (err < 0) * o.quantile) / numel(idx);   % pinball gradient
        end
        for l = L:-1:1
            gW = A{l}' * dZ + 2 * o.lambda_l2 * W{l};
            gb = sum(dZ, 1);
            if l > 1
                dZ_prev = (dZ * W{l}') .* (Z{l-1} > 0);
            end
            mW{l} = b1 * mW{l} + (1 - b1) * gW;  vW{l} = b2 * vW{l} + (1 - b2) * gW.^2;
            mb{l} = b1 * mb{l} + (1 - b1) * gb;  vb{l} = b2 * vb{l} + (1 - b2) * gb.^2;
            W{l} = W{l} - o.lr * (mW{l} / (1 - b1^step)) ./ (sqrt(vW{l} / (1 - b2^step)) + eps_a);
            b{l} = b{l} - o.lr * (mb{l} / (1 - b1^step)) ./ (sqrt(vb{l} / (1 - b2^step)) + eps_a);
            if l > 1, dZ = dZ_prev; end
        end
    end

    out_tr = forward(W, b, Xn_tr); out_va = forward(W, b, Xn_va);
    if isempty(o.quantile)
        hist.train_loss(ep) = huber(out_tr - T_tr, delta);
        hist.val_loss(ep)   = huber(out_va - T_va, delta);
    else
        hist.train_loss(ep) = pinball(T_tr - out_tr, o.quantile);
        hist.val_loss(ep)   = pinball(T_va - out_va, o.quantile);
    end
    if o.log_target, y_va = exp(out_va); else, y_va = out_va; end
    hist.val_mae(ep) = mean(abs(y_va - Y_va));
    if isempty(o.quantile), crit = hist.val_mae(ep); else, crit = hist.val_loss(ep); end

    if crit < best - 1e-12
        best = hist.val_mae(ep); bestW = W; bestb = b; best_ep = ep; since = 0;
    else
        since = since + 1;
        if since >= o.patience, break; end
    end
end

f = {'train_loss', 'val_loss', 'val_mae'};
for k = 1:3, hist.(f{k}) = hist.(f{k})(1:ep); end
hist.best_epoch = best_ep; hist.steps = step;

model.W = bestW; model.b = bestb;
model.mu_X = mu_X; model.sig_X = sig_X;
model.hidden = o.hidden; model.log_target = o.log_target; model.quantile = o.quantile;
model.n_params = sum(cellfun(@numel, bestW)) + sum(cellfun(@numel, bestb));
model.flops = 2 * sum(dims(1:end-1) .* dims(2:end));   % mult-adds of one forward pass
model.opts = o;
end

function y = forward(W, b, X)
a = X;
for l = 1:numel(W)
    a = a * W{l} + b{l};
    if l < numel(W), a = max(0, a); end
end
y = a;
end

function L = pinball(r, tau)
% pinball (quantile) loss of residual r = y - y_hat
L = mean(max(tau * r, (tau - 1) * r));
end

function Lh = huber(e, d)
a = abs(e);
Lh = mean((a <= d) .* 0.5 .* e.^2 + (a > d) .* (d * a - 0.5 * d^2));
end

function o = local_defaults(o)
def = struct('hidden', [32 16], 'log_target', false, 'seed', 42, 'max_steps', 20000, ...
    'batch_size', 128, 'lr', 0.005, 'lambda_l2', 1e-4, 'delta_huber', 1.5, ...
    'delta_huber_log', 0.5, 'patience', 40, 'quantile', []);
fn = fieldnames(def);
for k = 1:numel(fn)
    if ~isfield(o, fn{k}), o.(fn{k}) = def.(fn{k}); end
end
end
