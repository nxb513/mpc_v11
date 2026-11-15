function base = latency_baselines(X_tr, Y_tr)
% LATENCY_BASELINES Black-box reference predictor fitted by least squares on the
% SAME training data as the Tiny-ML model (the latency formula is hidden).
% Features: [N, I_max, cpu, |e_pos|, |e_vel|].
%  'linear' : linear regression on the raw features.
base.linear.beta = [ones(size(X_tr, 1), 1), X_tr] \ Y_tr;
base.linear.predict = @(X) [ones(size(X, 1), 1), X] * base.linear.beta;
end
