function train_model()
% TRAIN_MODEL Trains a genuine 2-hidden-layer Multi-Layer Perceptron (MLP)
% with Huber (Smooth-L1) loss, Adam optimizer, and L2 regularization on raw features.

if ~isfile('mpc_large_dataset.mat')
    generate_large_dataset();
end

data = load('mpc_large_dataset.mat');
X = data.X_raw;
Y = data.Y_lat;

% Focus training on the real-time operational latency regime (<= 45 ms)
valid_mask = (Y <= 45.0);
X = X(valid_mask, :);
Y = Y(valid_mask, :);

rng(42);
num_samples = size(X, 1);
idx = randperm(num_samples);
n_train = round(0.8 * num_samples);

X_train = X(idx(1:n_train), :);
Y_train = Y(idx(1:n_train), :);
X_val   = X(idx(n_train+1:end), :);
Y_val   = Y(idx(n_train+1:end), :);

mu_X  = mean(X_train, 1);
sig_X = std(X_train, 0, 1) + 1e-8;

X_tr_norm  = (X_train - mu_X) ./ sig_X;
X_val_norm = (X_val   - mu_X) ./ sig_X;

% 2-Hidden-Layer MLP: 6 -> 32 (ReLU) -> 16 (ReLU) -> 1 (Linear)
in_dim = 6; h1_dim = 32; h2_dim = 16; out_dim = 1;

W1 = randn(in_dim, h1_dim) * sqrt(2 / in_dim);
b1 = zeros(1, h1_dim);
W2 = randn(h1_dim, h2_dim) * sqrt(2 / h1_dim);
b2 = zeros(1, h2_dim);
W3 = randn(h2_dim, out_dim) * sqrt(2 / h2_dim);
b3 = mean(Y_train);

lr = 0.005; beta1 = 0.9; beta2 = 0.999; eps_adam = 1e-8;
lambda_l2 = 1e-4; epochs = 500; batch_size = 128; delta_huber = 1.5;

params = {W1, b1, W2, b2, W3, b3};
m_adam = cellfun(@zeros, cellfun(@size, params, 'UniformOutput', false), 'UniformOutput', false);
v_adam = m_adam;
step = 0;

for ep = 1:epochs
    perm = randperm(n_train);
    for b_start = 1:batch_size:n_train
        step = step + 1;
        b_end = min(b_start + batch_size - 1, n_train);
        Xb = X_tr_norm(perm(b_start:b_end), :);
        Yb = Y_train(perm(b_start:b_end), :);
        B = size(Xb, 1);

        Z1 = Xb * params{1} + params{2};
        A1 = max(0, Z1);
        Z2 = A1 * params{3} + params{4};
        A2 = max(0, Z2);
        Y_hat = A2 * params{5} + params{6};

        % Huber (Smooth L1) Gradient
        err = Y_hat - Yb;
        dY = (1 / B) * max(-delta_huber, min(delta_huber, err));

        dW3 = A2' * dY + 2 * lambda_l2 * params{5};
        db3 = sum(dY, 1);

        dA2 = dY * params{5}';
        dZ2 = dA2 .* (Z2 > 0);
        dW2 = A1' * dZ2 + 2 * lambda_l2 * params{3};
        db2 = sum(dZ2, 1);

        dA1 = dZ2 * params{3}';
        dZ1 = dA1 .* (Z1 > 0);
        dW1 = Xb' * dZ1 + 2 * lambda_l2 * params{1};
        db1 = sum(dZ1, 1);

        grads = {dW1, db1, dW2, db2, dW3, db3};
        for p = 1:6
            m_adam{p} = beta1 * m_adam{p} + (1 - beta1) * grads{p};
            v_adam{p} = beta2 * v_adam{p} + (1 - beta2) * (grads{p}.^2);
            m_hat = m_adam{p} / (1 - beta1^step);
            v_hat = v_adam{p} / (1 - beta2^step);
            params{p} = params{p} - lr * m_hat ./ (sqrt(v_hat) + eps_adam);
        end
    end
end

W1 = params{1}; b1 = params{2};
W2 = params{3}; b2 = params{4};
W3 = params{5}; b3 = params{6};

A1_v = max(0, X_val_norm * W1 + b1);
A2_v = max(0, A1_v * W2 + b2);
Y_val_pred = A2_v * W3 + b3;

val_mae = mean(abs(Y_val - Y_val_pred));
val_r2  = 1 - sum((Y_val - Y_val_pred).^2) / sum((Y_val - mean(Y_val)).^2);

fprintf('2-Hidden-Layer Tiny-ML MLP trained! Val R^2 = %.4f, Val MAE = %.4f ms\n', val_r2, val_mae);

model.W1 = W1; model.b1 = b1;
model.W2 = W2; model.b2 = b2;
model.W3 = W3; model.b3 = b3;
model.mu_X = mu_X;
model.sig_X = sig_X;
model.val_r2 = val_r2;
model.val_mae = val_mae;

save('trained_predictor.mat', 'model');
fprintf('Saved trained weights to trained_predictor.mat\n');
end
