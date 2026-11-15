function report_results(in_dir)
% REPORT_RESULTS Gathers results/bench_case_*.mat (bench_case.m), prints the
% statistics of the full grid (experiment_config.m) and exports Fig_1, Fig_2,
% Fig_3, Fig_8, Fig_9.
%
% Statistics (no toolbox needed):
%   * RMSE: mean over the non-crashed seeds with a 95 % bootstrap percentile CI
%     (10 000 resamples, fixed seed). Crashes are reported as counts.
%   * Paired comparison of Proposed with each baseline on the same seeds where
%     neither crashed: mean relative RMSE difference (Proposed - baseline) /
%     baseline, with a 95 % bootstrap CI, and the share of seeds won by Proposed.
%     A case counts as "better" / "worse" when the CI excludes 0.
if nargin < 1, in_dir = 'results'; end
cfg = experiment_config();
nC = numel(cfg.controllers); iP = nC;
B = cell(1, cfg.n_cases);
for id = 1:cfg.n_cases
    s = load(fullfile(in_dir, sprintf('bench_case_%03d.mat', id)));
    B{id} = s.out;
end
nSeed = size(B{1}.res.rmse, 1);
parts = cellfun(@(o) o.case.part, B);
nL = numel(cfg.levels); nT = numel(cfg.trajectories);

% ---- per-case summary ------------------------------------------------------------
rel = nan(cfg.n_cases, nC - 1, 3);      % (case, baseline, [mean lo hi]) relative diff Proposed - baseline
win = nan(cfg.n_cases, nC - 1);
crash = zeros(cfg.n_cases, nC); rm = nan(cfg.n_cases, nC); miss = rm; cpu = rm;
fprintf('All statistics over %d seeds per case, %d cases.\n\n', nSeed, cfg.n_cases);
fprintf('%-4s %-4s %-10s %-14s %4s %4s %4s %-5s |', 'id', 'part', 'traj', 'level', 'Hz', 'CPU', 'dist', 'W');
fprintf(' %-17s |', 'Fixed5', 'Fixed10', 'Nominal', 'Proposed');
fprintf(' P vs Fixed5 [95%% CI] win  | P vs Nominal [95%% CI] win\n');
for id = 1:cfg.n_cases
    o = B{id}; cs = o.case; r = o.res;
    for c = 1:nC
        crash(id, c) = sum(r.crash(:, c));
        rm(id, c) = mean(r.rmse(~r.crash(:, c), c));
        miss(id, c) = mean(r.miss(:, c)); cpu(id, c) = mean(r.cpu_ms_per_s(:, c));
    end
    for c = 1:nC-1
        [rel(id, c, 1), rel(id, c, 2), rel(id, c, 3), win(id, c)] = paired_rel(r, iP, c);
    end
    fprintf('%-4d %-4s %-10s %-14s %4d %4.0f %4.1f %-5s |', id, cs.part, cfg.trajectories{cs.traj}, ...
        cfg.levels{cs.level}, cs.freq, 100 * cs.cpu, cs.dist, cs.weights);
    for c = 1:nC
        fprintf(' %7.4f crash%3d |', rm(id, c), crash(id, c));
    end
    fprintf(' %+6.1f%% [%+6.1f,%+6.1f] %3.0f%% | %+6.1f%% [%+6.1f,%+6.1f] %3.0f%%\n', ...
        100 * rel(id, 1, 1), 100 * rel(id, 1, 2), 100 * rel(id, 1, 3), win(id, 1), ...
        100 * rel(id, 3, 1), 100 * rel(id, 3, 2), 100 * rel(id, 3, 3), win(id, 3));
end

% ---- global verdicts ---------------------------------------------------------------
fprintf('\n=== OVERALL: Proposed vs each baseline (case counted when the 95%% CI excludes 0) ===\n');
for c = 1:nC-1
    hi = rel(:, c, 3); lo = rel(:, c, 2); valid = ~isnan(rel(:, c, 1));
    better = sum(hi < 0); worse = sum(lo > 0); ns = sum(valid) - better - worse;
    fprintf('vs %-26s : better in %3d, worse in %3d, no significant difference in %3d, not comparable (crashes) %3d of %d cases\n', ...
        cfg.labels{c}, better, worse, ns, sum(~valid), cfg.n_cases);
end
fprintf('\nCrashed runs per controller (out of %d runs):\n', cfg.n_cases * nSeed);
for c = 1:nC
    fprintf('  %-30s : %5d  (%.1f%%)\n', cfg.labels{c}, sum(crash(:, c)), 100 * sum(crash(:, c)) / (cfg.n_cases * nSeed));
end

% per level, Part A (all rates and loads pooled)
fprintf('\n=== Part A by scenario level (all trajectories, rates and CPU loads) ===\n');
fprintf('%-14s |', 'Level'); fprintf(' %-22s |', cfg.labels{:}); fprintf(' P better / worse vs Fixed5 | vs Nominal\n');
for s = 1:nL
    ids = find(parts == 'A' & cellfun(@(o) o.case.level == s, B));
    fprintf('%-14s |', cfg.levels{s});
    for c = 1:nC
        fprintf(' RMSE %6.4f crash %4.1f%% |', mean(rm(ids, c), 'omitnan'), 100 * sum(crash(ids, c)) / (numel(ids) * nSeed));
    end
    fprintf(' %3d / %3d of %3d | %3d / %3d\n', sum(rel(ids, 1, 3) < 0), sum(rel(ids, 1, 2) > 0), numel(ids), ...
        sum(rel(ids, 3, 3) < 0), sum(rel(ids, 3, 2) > 0));
end
for pt = 'BC'
    if pt == 'B', ttl = 'Part B (disturbance 0 and 1.0 m/s^2)'; else, ttl = 'Part C (loose Bryson weights)'; end
    ids = find(parts == pt);
    v1 = ~isnan(rel(ids, 1, 1)); v3 = ~isnan(rel(ids, 3, 1));
    fprintf('\n%s: Proposed better / worse / n.s. / not comparable vs Fixed5: %d / %d / %d / %d; vs Nominal: %d / %d / %d / %d (of %d)\n', ttl, ...
        sum(rel(ids, 1, 3) < 0), sum(rel(ids, 1, 2) > 0), sum(v1) - sum(rel(ids, 1, 3) < 0) - sum(rel(ids, 1, 2) > 0), sum(~v1), ...
        sum(rel(ids, 3, 3) < 0), sum(rel(ids, 3, 2) > 0), sum(v3) - sum(rel(ids, 3, 3) < 0) - sum(rel(ids, 3, 2) > 0), sum(~v3), numel(ids));
end

% ---- Figures ----------------------------------------------------------------------------
colors = [0.25 0.55 0.85; 0.85 0.35 0.25; 0.50 0.35 0.70; 0.15 0.68 0.38];
markers = {'o', 's', 'd', 'p'};
isA0 = @(o) o.case.part == 'A' && o.case.freq == cfg.freq0 && o.case.cpu == cfg.cpu0;

% Fig 1: scheduler trace, lemniscate, S4, 50 Hz, CPU 50 %, first seed
id1 = find(cellfun(@(o) isA0(o) && o.case.traj == 1 && o.case.level == 4, B), 1);
o = B{id1}; tr = o.traces{iP}; n = tr.n_done; t = o.t(1:n);
f1 = paper_figure(174, 110);
tl = tiledlayout(f1, 3, 1, 'TileSpacing', 'compact');
ax = nexttile(tl); plot(ax, t, 100 * tr.cpu(1:n), 'k-', 'LineWidth', 0.8); ylabel(ax, 'CPU load (%)'); grid(ax, 'on');
ax = nexttile(tl); hold(ax, 'on'); plot(ax, t, tr.lat(1:n), 'b-', 'LineWidth', 1.0);
yline(ax, o.budget, 'r--', sprintf('Budget %.0f ms', o.budget), 'LineWidth', 1.3); ylabel(ax, 'Solve latency (ms)'); grid(ax, 'on');
ax = nexttile(tl); stairs(ax, t, tr.N(1:n), 'Color', colors(iP, :), 'LineWidth', 1.2);
ylabel(ax, 'Selected N'); xlabel(ax, 'Time (s)'); grid(ax, 'on');

% Fig 2: RMSE vs level at the nominal rate / load, with 95 % CI
f2 = paper_figure(174, 70);
tl = tiledlayout(f2, 1, nT, 'TileSpacing', 'compact');
for trj = 1:nT
    ax = nexttile(tl); hold(ax, 'on');
    for c = 1:nC
        m = nan(1, nL); lo = m; hi = m;
        for s = 1:nL
            id = find(cellfun(@(o) isA0(o) && o.case.traj == trj && o.case.level == s, B), 1);
            r = B{id}.res; [m(s), lo(s), hi(s)] = boot_mean(r.rmse(~r.crash(:, c), c));
        end
        x = (1:nL) + (c - (nC + 1) / 2) * 0.08;
        errorbar(ax, x, m, m - lo, hi - m, ['-' markers{c}], 'Color', colors(c, :), 'LineWidth', 1.3, ...
            'MarkerFaceColor', colors(c, :), 'CapSize', 3);
    end
    set(ax, 'XTick', 1:nL, 'XTickLabel', cfg.levels, 'XTickLabelRotation', 30); grid(ax, 'on');
    title(ax, sprintf('(%c) %s', 'a' + trj - 1, cfg.trajectories{trj}), 'FontWeight', 'normal'); ylabel(ax, 'Position RMSE (m)');
end
lg = legend(nexttile(tl, 1), cfg.labels, 'Orientation', 'horizontal'); lg.Layout.Tile = 'north';

% Fig 3: crash rate, deadline misses, solver time vs level (Part A pooled)
f3 = paper_figure(174, 70);
tl = tiledlayout(f3, 1, 3, 'TileSpacing', 'compact');
CR = nan(nL, nC); MS = CR; CP = CR;
for s = 1:nL
    ids = find(parts == 'A' & cellfun(@(o) o.case.level == s, B));
    CR(s, :) = 100 * sum(crash(ids, :), 1) / (numel(ids) * nSeed);
    MS(s, :) = mean(miss(ids, :), 1); CP(s, :) = mean(cpu(ids, :), 1);
end
ax = nexttile(tl); b = bar(ax, CR); for c = 1:nC, b(c).FaceColor = colors(c, :); end
set(ax, 'XTickLabel', cfg.levels, 'XTickLabelRotation', 30); ylabel(ax, 'Crashed runs (%)'); grid(ax, 'on');
ax = nexttile(tl); hold(ax, 'on');
for c = 1:nC, plot(ax, 1:nL, MS(:, c), ['-' markers{c}], 'Color', colors(c, :), 'LineWidth', 1.4); end
set(ax, 'XTick', 1:nL, 'XTickLabel', cfg.levels, 'XTickLabelRotation', 30); ylabel(ax, 'Deadline misses (%)'); grid(ax, 'on');
ax = nexttile(tl); hold(ax, 'on');
for c = 1:nC, plot(ax, 1:nL, CP(:, c), ['-' markers{c}], 'Color', colors(c, :), 'LineWidth', 1.4); end
set(ax, 'XTick', 1:nL, 'XTickLabel', cfg.levels, 'XTickLabelRotation', 30); ylabel(ax, 'Solver time per second of flight (ms/s)'); grid(ax, 'on');
lg = legend(ax, cfg.labels, 'Orientation', 'horizontal'); lg.Layout.Tile = 'north';

% Fig 8: relative RMSE change of Proposed vs Fixed N=5 over rate x CPU load, per level
f8 = paper_figure(174, 105);
tl = tiledlayout(f8, 2, 3, 'TileSpacing', 'compact');
for s = 1:nL
    Hm = nan(numel(cfg.cpu_set), numel(cfg.freqs));
    for i = 1:numel(cfg.cpu_set)
        for j = 1:numel(cfg.freqs)
            ids = find(cellfun(@(o) o.case.part == 'A' && o.case.level == s && o.case.cpu == cfg.cpu_set(i) ...
                && o.case.freq == cfg.freqs(j), B));
            Hm(i, j) = 100 * mean(rel(ids, 1, 1), 'omitnan');
        end
    end
    ax = nexttile(tl);
    imagesc(ax, Hm, 'AlphaData', ~isnan(Hm)); set(ax, 'Color', [0.85 0.85 0.85]);
    caxis(ax, [-40 40]); colormap(ax, local_diverging());
    set(ax, 'XTick', 1:numel(cfg.freqs), 'XTickLabel', cfg.freqs, 'YTick', 1:numel(cfg.cpu_set), ...
        'YTickLabel', 100 * cfg.cpu_set, 'YDir', 'normal');
    for i = 1:numel(cfg.cpu_set)
        for j = 1:numel(cfg.freqs)
            if ~isnan(Hm(i, j)), text(ax, j, i, sprintf('%+.0f%%', Hm(i, j)), 'HorizontalAlignment', 'center', 'FontSize', 7); end
        end
    end
    title(ax, sprintf('(%c) %s', 'a' + s - 1, cfg.levels{s}), 'FontWeight', 'normal'); xlabel(ax, 'Control rate (Hz)'); ylabel(ax, 'Mean CPU load (%)');
end
cb = colorbar(ax); cb.Layout.Tile = 'east'; cb.Label.String = 'RMSE change vs Fixed N = 5 (%)';

% Fig 9: Parts B and C, relative change vs Fixed N=5 and vs Nominal per level
f9 = paper_figure(174, 70);
tl = tiledlayout(f9, 1, 2, 'TileSpacing', 'compact');
conds = {@(o) o.case.part == 'B' && o.case.dist == 0, 'disturbance 0'; ...
         @(o) isA0(o), 'disturbance 0.5 (Part A)'; ...
         @(o) o.case.part == 'B' && o.case.dist == 1.0, 'disturbance 1.0'; ...
         @(o) o.case.part == 'C', 'loose weights'};
for bb = [1, 3]
    ax = nexttile(tl); V = nan(nL, size(conds, 1));
    for k = 1:size(conds, 1)
        for s = 1:nL
            ids = find(cellfun(@(o) conds{k, 1}(o) && o.case.level == s, B));
            V(s, k) = 100 * mean(rel(ids, bb, 1), 'omitnan');
        end
    end
    bar(ax, V); grid(ax, 'on'); set(ax, 'XTickLabel', cfg.levels, 'XTickLabelRotation', 30);
    ylabel(ax, sprintf('RMSE change vs %s (%%)', cfg.labels{bb}));
end
lg = legend(ax, conds(:, 2), 'Orientation', 'horizontal'); lg.Layout.Tile = 'north';

figs = {f1, 'Fig_1'; f2, 'Fig_2'; f3, 'Fig_3'; f8, 'Fig_8'; f9, 'Fig_9'};
for i = 1:size(figs, 1)
    export_figure(figs{i, 1}, figs{i, 2});
end
save('benchmark_results.mat', 'B', 'cfg', 'rel', 'win', 'crash', 'rm', 'miss', 'cpu');
end

% =========================================================================================
function [m, lo, hi] = boot_mean(x)
x = x(:);
if isempty(x), m = NaN; lo = NaN; hi = NaN; return; end
m = mean(x);
if numel(x) < 2, lo = m; hi = m; return; end
rs = RandStream('mt19937ar', 'Seed', 12345);
bm = sort(mean(x(randi(rs, numel(x), numel(x), 10000)), 1));
lo = bm(250); hi = bm(9750);
end

function [d, lo, hi, w] = paired_rel(r, a, b)
ok = ~r.crash(:, a) & ~r.crash(:, b);
if sum(ok) < 3, d = NaN; lo = NaN; hi = NaN; w = NaN; return; end
x = (r.rmse(ok, a) - r.rmse(ok, b)) ./ r.rmse(ok, b);
[d, lo, hi] = boot_mean(x);
w = 100 * mean(x < 0);
end

function cm = local_diverging()
n = 128;
up = [linspace(0.15, 1, n)', linspace(0.55, 1, n)', linspace(0.25, 1, n)'];
dn = [linspace(1, 0.80, n)', linspace(1, 0.25, n)', linspace(1, 0.20, n)'];
cm = [up; dn];
end
