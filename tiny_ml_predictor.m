function t_pred_ms = tiny_ml_predictor(N, max_iters, cpu_load, pos_err_norm, vel_err_norm, hw_speed)
% TINY_ML_PREDICTOR Real-time forward inference of the trained 2-hidden-layer MLP.
% Automatically loads trained_predictor.mat on first call via persistent memory.

persistent net_model

if isempty(net_model)
    if ~isfile('trained_predictor.mat')
        train_model();
    end
    loaded = load('trained_predictor.mat');
    net_model = loaded.model;
end

x_in = [N, max_iters, cpu_load, pos_err_norm, vel_err_norm, hw_speed];
x_norm = (x_in - net_model.mu_X) ./ net_model.sig_X;

h1 = max(0, x_norm * net_model.W1 + net_model.b1);
h2 = max(0, h1 * net_model.W2 + net_model.b2);
t_pred_ms = max(0.10, h2 * net_model.W3 + net_model.b3);
end
