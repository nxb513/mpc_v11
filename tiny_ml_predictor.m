function [t_upper_ms, t_mean_ms] = tiny_ml_predictor(N, max_iters, cpu_load, pos_err_norm, vel_err_norm)
% TINY_ML_PREDICTOR Real-time inference of the Tiny-ML latency predictors.
% Inputs: [N, I_max, CPU load, ||e_pos||, ||e_vel||].
%   t_upper_ms : conformal-calibrated 0.95-quantile (upper bound) used for the
%                deadline check of the scheduler
%   t_mean_ms  : point prediction (accuracy studies)
% Loads trained_predictor.mat on first call (persistent).

persistent net_model net_q
if isempty(net_model)
    if ~isfile('trained_predictor.mat')
        train_model();
    end
    loaded = load('trained_predictor.mat');
    net_model = loaded.model;
    net_q = loaded.model_q;
end
x = [N, max_iters, cpu_load, pos_err_norm, vel_err_norm];
t_upper_ms = net_q.margin * max(0, predict_mlp(net_q, x));
if nargout > 1
    t_mean_ms = max(0, predict_mlp(net_model, x));
end
end
