function [t_pred_ms, margin] = tiny_ml_predictor(N, max_iters, cpu_load, pos_err_norm, vel_err_norm)
% TINY_ML_PREDICTOR Real-time forward inference of the trained Tiny-ML latency
% predictor. Inputs: [N, I_max, CPU load, ||e_pos||, ||e_vel||] (single hardware
% profile). Loads trained_predictor.mat on first call (persistent). margin is the
% data-driven safety factor stored by train_model.m.

persistent net_model
if isempty(net_model)
    if ~isfile('trained_predictor.mat')
        train_model();
    end
    loaded = load('trained_predictor.mat');
    net_model = loaded.model;
end
t_pred_ms = max(0, predict_mlp(net_model, [N, max_iters, cpu_load, pos_err_norm, vel_err_norm]));
if isfield(net_model, 'margin'), margin = net_model.margin; else, margin = 1; end
end
