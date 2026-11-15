function report_results(in_dir)
% REPORT_RESULTS Gathers results/bench_case_*.mat (bench_case.m), prints the
% tables of Parts 1-4 and exports Fig_1, Fig_2, Fig_3, Fig_8, Fig_9.
if nargin < 1, in_dir = 'results'; end
cfg = experiment_config();
nC = numel(cfg.controllers);
B = cell(1, cfg.n_cases);
for id = 1:cfg.n_cases
    s = load(fullfile(in_dir, sprintf('bench_case_%02d.mat', id)));
    B{id} = s.out;
end
parts = cellfun(@(o) o.case.part, B);

% ---- tables -----------------------------------------------------------------------
titles = {'PART 1: scenario ladder (50 Hz, CPU 50%, disturbance 0.5 m/s^2)', ...
          'PART 2: control-rate sweep at S3 (lemniscate)', ...
          'PART 3: mean CPU load x disturbance at S3 (lemniscate, 50 Hz)', ...
          'PART 4: loose Bryson weights at S3 (50 Hz)'};
for pt = 1:4
    fprintf('\n=== %s ===\n', titles{pt});
    for id = find(parts == pt)
        o = B{id}; cs = o.case;
        fprintf('\n%s, %s, %d Hz, CPU %.0f%%, dist %.1f, %s | ref beyond limits %.1f%%\n', ...
            cfg.trajectories{cs.traj}, cfg.levels{cs.level}, cs.freq, 100 * cs.cpu, cs.dist, cs.weights, o.ref_stats.ff_over);
        fprintf('  %-26s | %8s | %8s | %10s | %6s\n', 'Controller', 'RMSE (m)', 'Miss (%)', 'CPU (ms/s)', 'mean N');
        for c = 1:nC
            if o.res.crash(c), rs = '   CRASH'; else, rs = sprintf('%8.4f', o.res.rmse(c)); end
            fprintf('  %-26s | %s | %8.2f | %10.1f | %6.2f\n', cfg.labels{c}, rs, o.res.miss(c), o.res.cpu_ms_per_s(c), o.res.N_mean(c));
        end
    end
end

% ---- Part 1 arrays ----------------------------------------------------------------
nT = 3; nS = 4;
rmse1 = nan(nS, nT, nC); miss1 = nan(nS, nT, nC); cpu1 = nan(nS, nT, nC); crash1 = false(nS, nT, nC);
for id = find(parts == 1)
    cs = B{id}.case;
    rmse1(cs.level, cs.traj, :) = B{id}.res.rmse; miss1(cs.level, cs.traj, :) = B{id}.res.miss;
    cpu1(cs.level, cs.traj, :) = B{id}.res.cpu_ms_per_s; crash1(cs.level, cs.traj, :) = B{id}.res.crash;
end
fprintf('\nSummary Part 1 (mean over trajectories; crashes counted separately)\n');
fprintf('%-26s |', 'Controller'); fprintf(' %-13s |', cfg.levels{:}); fprintf(' crashes\n');
for c = 1:nC
    fprintf('%-26s |', cfg.labels{c});
    for s = 1:nS, fprintf(' %13.4f |', mean(rmse1(s, :, c), 'omitnan')); end
    fprintf(' %d\n', sum(crash1(:, :, c), 'all'));
end

colors = [0.25 0.55 0.85; 0.85 0.35 0.25; 0.90 0.65 0.15; 0.50 0.35 0.70; 0.15 0.68 0.38];
markers = {'o', 's', '^', 'd', 'p'};

% Fig 1: proposed controller trace, S3, lemniscate (Part 1 case traj 1 level 3)
id1 = find(parts == 1 & cellfun(@(o) o.case.traj == 1 && o.case.level == 3, B), 1);
o = B{id1}; tr = o.traces{nC}; n = tr.n_done; t = o.t(1:n);
f1 = figure('Name', 'Fig 1', 'Color', 'w', 'Position', [80, 80, 980, 460]);
tl = tiledlayout(f1, 2, 1, 'TileSpacing', 'compact');
ax = nexttile(tl); hold(ax, 'on');
ymax = 1.1 * max([tr.lat(1:n), o.budget]);
area(ax, t, o.ref_over(1:n) * ymax, 'FaceColor', [1 0.85 0.85], 'EdgeColor', 'none');
plot(ax, t, tr.lat(1:n), 'b-', 'LineWidth', 1.0);
yline(ax, o.budget, 'r--', sprintf('Budget %.0f ms', o.budget), 'LineWidth', 1.3);
ylabel(ax, 'Solve latency (ms)'); grid(ax, 'on'); ylim(ax, [0, ymax]);
legend(ax, {'Reference beyond rotor limits', 'Latency', 'Budget'}, 'Location', 'northeast');
ax = nexttile(tl); stairs(ax, t, tr.N(1:n), 'Color', colors(5, :), 'LineWidth', 1.2);
ylabel(ax, 'Selected N'); xlabel(ax, 'Time (s)'); grid(ax, 'on');
title(tl, 'Fig 1: Proposed controller (S3, lemniscate): horizon raised where the reference exceeds the rotor limits');

% Fig 2: RMSE vs scenario level per trajectory
f2 = figure('Name', 'Fig 2', 'Color', 'w', 'Position', [100, 100, 1150, 420]);
tl = tiledlayout(f2, 1, 3, 'TileSpacing', 'compact');
for trj = 1:nT
    ax = nexttile(tl); hold(ax, 'on');
    for c = 1:nC
        plot(ax, 1:nS, rmse1(:, trj, c), ['-' markers{c}], 'Color', colors(c, :), 'LineWidth', 1.5);
    end
    set(ax, 'XTick', 1:nS, 'XTickLabel', cfg.levels, 'XTickLabelRotation', 20); grid(ax, 'on');
    title(ax, cfg.trajectories{trj}); ylabel(ax, 'Position RMSE (m)');
end
legend(nexttile(tl, 1), cfg.labels, 'Location', 'northwest', 'FontSize', 8);
title(tl, 'Fig 2: Tracking error vs actuator demand (missing marker = crash)');

% Fig 3: misses and solver time vs level
f3 = figure('Name', 'Fig 3', 'Color', 'w', 'Position', [120, 120, 1000, 420]);
tl = tiledlayout(f3, 1, 2, 'TileSpacing', 'compact');
ax = nexttile(tl); hold(ax, 'on');
for c = 1:nC, plot(ax, 1:nS, mean(miss1(:, :, c), 2), ['-' markers{c}], 'Color', colors(c, :), 'LineWidth', 1.5); end
set(ax, 'XTick', 1:nS, 'XTickLabel', cfg.levels); ylabel(ax, 'Deadline misses (%)'); grid(ax, 'on');
ax = nexttile(tl); hold(ax, 'on');
for c = 1:nC, plot(ax, 1:nS, mean(cpu1(:, :, c), 2), ['-' markers{c}], 'Color', colors(c, :), 'LineWidth', 1.5); end
set(ax, 'XTick', 1:nS, 'XTickLabel', cfg.levels); ylabel(ax, 'Solver time per second of flight (ms/s)'); grid(ax, 'on');
legend(ax, cfg.labels, 'Location', 'northwest', 'FontSize', 8);
title(tl, 'Fig 3: Real-time cost vs actuator demand (mean over trajectories)');

% Fig 8: rate sweep
ids = find(parts == 2); fr = cellfun(@(o) o.case.freq, B(ids));
[fr, ix] = sort(fr); ids = ids(ix);
R2 = cell2mat(cellfun(@(o) o.res.rmse(:), B(ids), 'UniformOutput', false));
M2 = cell2mat(cellfun(@(o) o.res.miss(:), B(ids), 'UniformOutput', false));
f8 = figure('Name', 'Fig 8', 'Color', 'w', 'Position', [140, 140, 1000, 420]);
tl = tiledlayout(f8, 1, 2, 'TileSpacing', 'compact');
ax = nexttile(tl); hold(ax, 'on');
for c = 1:nC, plot(ax, fr, R2(c, :), ['-' markers{c}], 'Color', colors(c, :), 'LineWidth', 1.5); end
xlabel(ax, 'Control rate (Hz)'); ylabel(ax, 'Position RMSE (m)'); grid(ax, 'on');
ax = nexttile(tl); hold(ax, 'on');
for c = 1:nC, plot(ax, fr, M2(c, :), ['-' markers{c}], 'Color', colors(c, :), 'LineWidth', 1.5); end
xlabel(ax, 'Control rate (Hz)'); ylabel(ax, 'Deadline misses (%)'); grid(ax, 'on');
legend(ax, cfg.labels, 'Location', 'northwest', 'FontSize', 8);
title(tl, 'Fig 8: Control-rate sweep at S3 (lemniscate)');

% Fig 9: CPU load x disturbance
R3 = nan(nC, 3, 3);
for id = find(parts == 3)
    cs = B{id}.case;
    R3(:, cfg.cpu_set == cs.cpu, cfg.dist_set == cs.dist) = B{id}.res.rmse(:);
end
f9 = figure('Name', 'Fig 9', 'Color', 'w', 'Position', [160, 160, 1000, 420]);
tl = tiledlayout(f9, 1, 2, 'TileSpacing', 'compact');
ax = nexttile(tl); bar(ax, 100 * cfg.cpu_set, squeeze(R3(:, :, 2))'); grid(ax, 'on');
xlabel(ax, 'Mean CPU load (%)'); ylabel(ax, 'Position RMSE (m)'); title(ax, 'Disturbance 0.5 m/s^2');
ax = nexttile(tl); bar(ax, cfg.dist_set, squeeze(R3(:, 2, :))'); grid(ax, 'on');
xlabel(ax, 'Disturbance std (m/s^2)'); ylabel(ax, 'Position RMSE (m)'); title(ax, 'Mean CPU load 50%');
legend(ax, cfg.labels, 'Location', 'northwest', 'FontSize', 8);
title(tl, 'Fig 9: Sensitivity to CPU load and disturbance at S3 (lemniscate); missing bar = crash');

figs = {f1, 'Fig_1.pdf'; f2, 'Fig_2.pdf'; f3, 'Fig_3.pdf'; f8, 'Fig_8.pdf'; f9, 'Fig_9.pdf'};
for i = 1:size(figs, 1)
    try
        exportgraphics(figs{i, 1}, figs{i, 2}, 'ContentType', 'vector');
    catch
        print(figs{i, 1}, figs{i, 2}, '-dpdf', '-painters');
    end
end
save('benchmark_results.mat', 'B', 'cfg');
end
