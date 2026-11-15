function y = predict_mlp(model, X)
% PREDICT_MLP Forward pass of a model returned by train_mlp.m (rows of X = samples).
a = (X - model.mu_X) ./ model.sig_X;
L = numel(model.W);
for l = 1:L
    a = a * model.W{l} + model.b{l};
    if l < L, a = max(0, a); end
end
if model.log_target
    y = exp(a);
else
    y = a;
end
end
