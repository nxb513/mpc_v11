function plot_learning_curve(lc)
% PLOT_LEARNING_CURVE Fig_4 (test MAE vs training-set size) and Fig_5 (training
% and validation loss vs epoch) from the output of learning_curve.m
% (learning_curve.mat). Single-column width (84 mm), no titles.
if nargin < 1
    s = load('learning_curve.mat'); lc = s.lc;
end
sizes = lc.sizes; mae = lc.mae; hist = lc.hist;

f4 = paper_figure(84, 62); hold on;
cols = [0.15 0.68 0.38; 0.45 0.45 0.45];
mk = {'-o', '--s'};
lab = {'Tiny-ML MLP', 'Linear regression'};
h = gobjects(2, 1);
for k = 1:2
    mu = mean(mae(:, :, k), 2); sd = std(mae(:, :, k), 0, 2);
    lo = max(mu - sd, 0.05 * mu);
    patch([sizes, fliplr(sizes)], [lo; flipud(mu + sd)]', cols(k, :), 'FaceAlpha', 0.15, 'EdgeColor', 'none');
    h(k) = plot(sizes, mu, mk{k}, 'Color', cols(k, :), 'MarkerSize', 4);
end
xline(lc.n_sat, 'k:', 'LineWidth', 0.8);
set(gca, 'XScale', 'log', 'YScale', 'log');
xlabel('Number of training samples'); ylabel('Test MAE (ms)');
legend(h, lab, 'Location', 'southwest'); grid on; box on;

f5 = paper_figure(84, 55);
semilogy(hist.train_loss, '-'); hold on;
semilogy(hist.val_loss, '--');
xline(hist.best_epoch, 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off');
xlabel('Epoch'); ylabel('Huber loss');
legend({'Training', 'Validation'}, 'Location', 'northeast'); grid on; box on;

export_figure(f4, 'Fig_4');
export_figure(f5, 'Fig_5');
end
