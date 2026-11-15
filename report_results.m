function report_results(in_dir)
% REPORT_RESULTS Gathers results/bench_case_*.mat (bench_case.m, n seeds per
% case), prints statistics and exports Fig_1, Fig_2, Fig_3, Fig_8, Fig_9.
%
% Statistics (no toolbox needed):
%   * RMSE: mean over the NON-crashed seeds with a 95 % bootstrap percentile CI
%     (10 000 resamples, fixed seed). Crashes are reported as counts.
%   * Paired comparison Proposed vs each baseline on the SAME seeds where neither
%     crashed: mean RMSE difference (negative = Proposed better) with a 95 %
%     bootstrap CI, and the share of seeds won by Proposed.
if nargin < 1, in_dir = 'results'; end
cfg = experiment_config();
nC = numel(cfg.controllers); iP = nC;     % Proposed is the last controller
B = cell(1, cfg.n_cases);
for id = 1:cfg.n_cases
    s = load(fullfile(in_dir, sprintf('bench_case_%02d.mat', id)));
    B{id} = s.out;
end
parts = cellfun(@(o) o.case.part, B);
nSeed = size(B{1}.res.rmse, 1);

% ---- per-case tables ----------------------------------------------------------------
titles = {'PART 1: scenario ladder (50 Hz, CPU 50%, disturbance 0.5 m/s^2)', ...
          'PART 2: control-rate sweep at S3 (lemniscate)', ...
          'PART 3: mean CPU load x disturbance at S3 (lemniscate, 50 Hz)', ...
          'PART 4: loose Bryson weights at S3 (50 Hz)'};
fprintf('All statistics over %d seeds per case.\n', nSeed);
for pt = 1:4
    fprintf('\n=== %s ===\n', titles{pt});
    for id = find(parts == pt)
        o = B{id}; cs = o.case; r = o.res;
        fprintf('\n%s, %s, %d Hz, CPU %.0f%%, dist %.1f, %s | ref beyond limits %.1f%%\n', ...
            cfg.trajectories{cs.traj}, cfg.levels{cs.level}, cs.freq, 100 * cs.cpu, cs.dist, cs.weights, o.ref_stats.ff_over);
        fprintf('  %-30s | %-26s | %7s | %8s | %10s | %6s\n', 'Controller', 'RMSE (m) mean [95% CI]', 'crashes', 'miss (%)', 'CPU (ms/s)', 'mean N');
        for c = 1:nC
            [m, lo, hi] = boot_mean(r.rmse(~r.crash(:, c), c));
            fprintf('  %-30s | %8.4f [%7.4f, %7.4f] | %4d/%-2d | %8.2f | %10.1f | %6.2f\n', cfg.labels{c}, m, lo, hi, ...
                sum(r.crash(:, c)), nSeed, mean(r.miss(:, c)), mean(r.cpu_ms_per_s(:, c)), mean(r.N_mean(:, c)));
        end
        for c = 1:nC-1
            [d, lo, hi, win, n] = paired(r, iP, c);
            fprintf('  Proposed - %-19s : %+8.4f m [%+.4f, %+.4f], Proposed better on %3.0f%% of %d paired seeds\n', ...
                cfg.labels{c}, d, lo, hi, win, n);
        end
    end
end

% ---- Part 1 aggregates ------------------------------------------------------------------
nT = 3; nS = 4;
mean1 = nan(nS, nT, nC); lo1 = mean1; hi1 = mean1; crash1 = zeros(nS, nT, nC); miss1 = mean1; cpu1 = mean1;
for id = find(parts == 1)
    cs = B{id}.case; r = B{id}.res;
    for c = 1:nC
        [mean1(cs.level, cs.traj, c), lo1(cs.level, cs.traj, c), hi1(cs.level, cs.traj, c)] = boot_mean(r.rmse(~r.crash(:, c), c));
        crash1(cs.level, cs.traj, c) = sum(r.crash(:, c));
        miss1(cs.level, cs.traj, c) = mean(r.miss(:, c));
        cpu1(cs.level, cs.traj, c) = mean(r.cpu_ms_per_s(:, c));
    end
end
fprintf('\nSUMMARY Part 1: mean RMSE (m) over the 3 trajectories | crashes out of %d runs per level\n', 3 * nSeed);
fprintf('%-30s |', 'Controller'); fprintf(' %-22s |', cfg.levels{:}); fprintf('\n');
for c = 1:nC
    fprintf('%-30s |', cfg.labels{c});
    for s = 1:nS
        fprintf(' %8.4f  crash %3d   |', mean(mean1(s, :, c), 'omitnan'), sum(crash1(s, :, c)));
    end
    fprintf('\n');
end

% ---- Figures ------------------------------------------------------------------------------
colors = [0.25 0.55 0.85; 0.85 0.35 0.25; 0.50 0.35 0.70; 0.15 0.68 0.38];
markers = {'o', 's', 'd', 'p'};

% Fig 1: Proposed controller trace (Part 1, lemniscate, S3, first seed)
id1 = find(parts == 1 & cellfun(@(o) o.case.traj == 1 && o.case.level == 3, B), 1);
o = B{id1}; tr = o.traces{iP}; n = tr.n_done; t = o.t(1:n);
f1 = figure('Name', 'Fig 1', 'Color', 'w', 'Position', [80, 80, 980, 520]);
tl = tiledlayout(f1, 3, 1, 'TileSpacing', 'compact');
ax = nexttile(tl); plot(ax, t, 100 * tr.cpu(1:n), 'k-', 'LineWidth', 0.8); ylabel(ax, 'CPU load (%)'); grid(ax, 'on');
ax = nexttile(tl); hold(ax, 'on');
plot(ax, t, tr.lat(1:n), 'b-', 'LineWidth', 1.0);
yline(ax, o.budget, 'r--', sprintf('Budget %.0f ms', o.budget), 'LineWidth', 1.3);
ylabel(ax, 'Solve latency (ms)'); grid(ax, 'on');
ax = nexttile(tl); stairs(ax, t, tr.N(1:n), 'Color', colors(iP, :), 'LineWidth', 1.2);
ylabel(ax, 'Selected N'); xlabel(ax, 'Time (s)'); grid(ax, 'on');
title(tl, 'Fig 1: Proposed Tiny-ML scheduler (S3, lemniscate, 50 Hz): horizon follows the available computation');

% Fig 2: RMSE vs level with 95 % CI, per trajectory
f2 = figure('Name', 'Fig 2', 'Color', 'w', 'Position', [100, 100, 1200, 440]);
tl = tiledlayout(f2, 1, 3, 'TileSpacing', 'compact');
for trj = 1:nT
    ax = nexttile(tl); hold(ax, 'on');
    for c = 1:nC
        x = (1:nS) + (c - (nC + 1) / 2) * 0.08;
        m = mean1(:, trj, c);
        errorbar(ax, x, m, m - lo1(:, trj, c), hi1(:, trj, c) - m, ['-' markers{c}], 'Color', colors(c, :), ...
            'LineWidth', 1.4, 'MarkerFaceColor', colors(c, :), 'CapSize', 4);
    end
    set(ax, 'XTick', 1:nS, 'XTickLabel', cfg.levels, 'XTickLabelRotation', 20); grid(ax, 'on');
    title(ax, cfg.trajectories{trj}); ylabel(ax, 'Position RMSE (m)');
end
legend(nexttile(tl, 1), cfg.labels, 'Location', 'northwest', 'FontSize', 8);
title(tl, sprintf('Fig 2: Tracking error vs actuator demand (mean and 95%% CI over %d seeds, crashed runs excluded)', nSeed));

% Fig 3: crashes, misses and solver time vs level
f3 = figure('Name', 'Fig 3', 'Color', 'w', 'Position', [120, 120, 1200, 420]);
tl = tiledlayout(f3, 1, 3, 'TileSpacing', 'compact');
ax = nexttile(tl); bar(ax, squeeze(sum(crash1, 2)) / (3 * nSeed) * 100); grid(ax, 'on');
set(ax, 'XTickLabel', cfg.levels); ylabel(ax, 'Crashed runs (%)');
ax = nexttile(tl); hold(ax, 'on');
for c = 1:nC, plot(ax, 1:nS, mean(miss1(:, :, c), 2), ['-' markers{c}], 'Color', colors(c, :), 'LineWidth', 1.5); end
set(ax, 'XTick', 1:nS, 'XTickLabel', cfg.levels); ylabel(ax, 'Deadline misses (%)'); grid(ax, 'on');
ax = nexttile(tl); hold(ax, 'on');
for c = 1:nC, plot(ax, 1:nS, mean(cpu1(:, :, c), 2), ['-' markers{c}], 'Color', colors(c, :), 'LineWidth', 1.5); end
set(ax, 'XTick', 1:nS, 'XTickLabel', cfg.levels); ylabel(ax, 'Solver time per second of flight (ms/s)'); grid(ax, 'on');
legend(ax, cfg.labels, 'Location', 'northwest', 'FontSize', 8);
title(tl, 'Fig 3: Safety and real-time cost vs actuator demand (all trajectories and seeds)');

% Fig 8: rate sweep with CI
ids = find(parts == 2); fr = cellfun(@(o) o.case.freq, B(ids)); [fr, ix] = sort(fr); ids = ids(ix);
f8 = figure('Name', 'Fig 8', 'Color', 'w', 'Position', [140, 140, 1100, 420]);
tl = tiledlayout(f8, 1, 2, 'TileSpacing', 'compact');
ax = nexttile(tl); hold(ax, 'on'); ax2 = nexttile(tl); hold(ax2, 'on');
for c = 1:nC
    m = nan(size(fr)); lo = m; hi = m; cr = m;
    for k = 1:numel(ids)
        r = B{ids(k)}.res; [m(k), lo(k), hi(k)] = boot_mean(r.rmse(~r.crash(:, c), c)); cr(k) = 100 * mean(r.crash(:, c));
    end
    errorbar(ax, fr, m, m - lo, hi - m, ['-' markers{c}], 'Color', colors(c, :), 'LineWidth', 1.4, 'CapSize', 4);
    plot(ax2, fr, cr, ['-' markers{c}], 'Color', colors(c, :), 'LineWidth', 1.4);
end
xlabel(ax, 'Control rate (Hz)'); ylabel(ax, 'Position RMSE (m)'); grid(ax, 'on');
xlabel(ax2, 'Control rate (Hz)'); ylabel(ax2, 'Crashed runs (%)'); grid(ax2, 'on');
legend(ax2, cfg.labels, 'Location', 'northwest', 'FontSize', 8);
title(tl, 'Fig 8: Control-rate sweep at S3 (lemniscate), mean and 95% CI');

% Fig 9: CPU load x disturbance
M3 = nan(nC, 3, 3); C3 = M3;
for id = find(parts == 3)
    cs = B{id}.case; r = B{id}.res;
    i = find(cfg.cpu_set == cs.cpu); j = find(cfg.dist_set == cs.dist);
    for c = 1:nC
        M3(c, i, j) = mean(r.rmse(~r.crash(:, c), c)); C3(c, i, j) = 100 * mean(r.crash(:, c));
    end
end
f9 = figure('Name', 'Fig 9', 'Color', 'w', 'Position', [160, 160, 1100, 440]);
tl = tiledlayout(f9, 1, 2, 'TileSpacing', 'compact');
ax = nexttile(tl); bar(ax, 100 * cfg.cpu_set, squeeze(M3(:, :, 2))'); grid(ax, 'on');
xlabel(ax, 'Mean CPU load (%)'); ylabel(ax, 'Position RMSE (m)'); title(ax, 'Disturbance 0.5 m/s^2');
ax = nexttile(tl); bar(ax, 100 * cfg.cpu_set, squeeze(C3(:, :, 2))'); grid(ax, 'on');
xlabel(ax, 'Mean CPU load (%)'); ylabel(ax, 'Crashed runs (%)'); title(ax, 'Disturbance 0.5 m/s^2');
legend(ax, cfg.labels, 'Location', 'northwest', 'FontSize', 8);
title(tl, 'Fig 9: Sensitivity to the background CPU load at S3 (lemniscate)');

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

% =========================================================================================
function [m, lo, hi] = boot_mean(x)
x = x(:);
if isempty(x), m = NaN; lo = NaN; hi = NaN; return; end
m = mean(x);
if numel(x) < 2, lo = m; hi = m; return; end
rs = RandStream('mt19937ar', 'Seed', 12345);
bm = mean(x(randi(rs, numel(x), numel(x), 10000)), 1);
bm = sort(bm);
lo = bm(max(1, round(0.025 * 10000))); hi = bm(round(0.975 * 10000));
end

function [d, lo, hi, win, n] = paired(r, a, b)
ok = ~r.crash(:, a) & ~r.crash(:, b);
diffs = r.rmse(ok, a) - r.rmse(ok, b);
n = sum(ok);
[d, lo, hi] = boot_mean(diffs);
win = 100 * mean(diffs < 0);
if n == 0, win = NaN; end
end
