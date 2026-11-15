function plot_context_study(res)
% PLOT_CONTEXT_STUDY Fig 7 from the results of context_study.m (context_study.mat).
if nargin < 1
    s = load('context_study.mat'); res = s.res;
end
ctx = res.ctx; sizes = res.sizes; rel_mlp = res.rel_mlp; rel_lin = res.rel_lin; n_sat = res.n_sat;
K = numel(ctx);
labels = cellfun(@(c) c.label, ctx, 'UniformOutput', false);
st = paper_palette();
f7 = paper_figure(174, 100);
tl = tiledlayout(f7, 2, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
for k = 1:K
    ax = nexttile(tl); hold(ax, 'on');
    mu = mean(rel_mlp(k, :, :), 3); sd = std(rel_mlp(k, :, :), 0, 3);
    ok = ~isnan(mu);
    patch(ax, [sizes(ok), fliplr(sizes(ok))], [max(mu(ok) - sd(ok), 0.05 * mu(ok)), fliplr(mu(ok) + sd(ok))], ...
        st.color(4, :), 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    h1 = plot(ax, sizes(ok), mu(ok), '-o', 'Color', st.color(4, :), 'MarkerSize', 3.5);
    h2 = plot(ax, sizes(ok), mean(rel_lin(k, ok, :), 3), '--', 'Color', [0.5 0.5 0.5]);
    if ~isnan(n_sat(k))
        xline(ax, n_sat(k), 'k:', sprintf('n_{sat} = %d', n_sat(k)), 'LineWidth', 1.0, 'FontSize', 7, ...
            'LabelVerticalAlignment', 'middle', 'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
    end
    set(ax, 'XScale', 'log', 'YScale', 'log', 'FontSize', 8, 'XLim', [18, 7e4], ...
        'XTick', [1e2, 1e3, 1e4], 'XTickLabel', {'10^2', '10^3', '10^4'});
    title(ax, sprintf('(%c) %s', 'a' + k - 1, labels{k}), 'FontWeight', 'normal', 'FontSize', 7);
    grid(ax, 'on'); box(ax, 'on');
    if k == K
        legend(ax, [h1, h2], {'Tiny-ML (fixed arch.)', 'Linear regression'}, 'Location', 'southwest', 'FontSize', 7);
    end
end
xlabel(tl, 'Number of collected training samples');
ylabel(tl, 'Relative test MAE (%)');
export_figure(f7, 'Fig_7');
end
