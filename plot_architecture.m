function plot_architecture()
% PLOT_ARCHITECTURE Diagram of the deployed Tiny-ML latency predictor (read from
% trained_predictor.mat): representative neurons per layer, a vertical ellipsis
% and unit count for wide layers, layer captions, and the pre-processing (z-score)
% and post-processing (exp) blocks. Exports Fig_6 (174 mm wide).

s = load('trained_predictor.mat');
mdl = s.model;
dims = [5, mdl.hidden(:)', 1];
L = numel(dims);
in_sym  = {'$N$', '$I_{\max}$', '$u_{\mathrm{cpu}}$', '$\|e_{p}\|$', '$\|e_{v}\|$'};
in_desc = {'horizon', 'iter. cap', 'CPU load', 'pos. error', 'vel. error'};

fh = paper_figure(174, 72);
ax = axes(fh, 'Position', [0 0 1 1]); hold(ax, 'on'); axis(ax, 'off');
set(ax, 'XLim', [0, 10.9], 'YLim', [-0.05, 3.95]);

FS = 8;                              % base font size (pt)
x_sym  = 2.20;                       % right edge of feature symbols
x_norm = 3.10;                       % z-score block centre
x_lay  = linspace(4.40, 7.95, L);    % neuron columns
x_exp  = 9.20;                       % exp block centre
x_out  = 10.20;                       % prediction label
y_c    = 2.45;                       % vertical centre
MAXSHOW = 6;  gap = 0.36;  r = 0.12;

c_in  = [0.16 0.44 0.71];
c_hid = [0.20 0.60 0.36];
c_out = [0.85 0.48 0.10];
grey  = [0.30 0.30 0.30];
layer_col = @(l) (l == 1) * c_in + (l == L) * c_out + (l > 1 && l < L) * c_hid;

% neuron positions
Y = cell(L, 1); ell = false(L, 1);
for l = 1:L
    n = dims(l);
    if n <= MAXSHOW
        yy = ((1:n) - (n + 1) / 2) * gap;
    else
        ell(l) = true; h = MAXSHOW / 2;
        yy = [(-h:-1) * gap - 0.5 * gap, (1:h) * gap + 0.5 * gap];
    end
    Y{l} = y_c - yy;                 % first neuron on top
end

% tinted layer panels
pad = 0.26;
for l = 1:L
    c = layer_col(l);
    yl = min(Y{l}) - pad; yh = max(Y{l}) + pad; w = 0.30;
    local_rbox(ax, x_lay(l) - w, yl, 2 * w, yh - yl, 0.10, c, 0.12, c);
end

% edges between drawn neurons
for l = 1:L-1
    for i = 1:numel(Y{l})
        for j = 1:numel(Y{l+1})
            plot(ax, [x_lay(l) + r, x_lay(l+1) - r], [Y{l}(i), Y{l+1}(j)], '-', ...
                'Color', [0.45 0.45 0.45 0.45], 'LineWidth', 0.4);
        end
    end
end

% neurons
th = linspace(0, 2 * pi, 40);
for l = 1:L
    c = layer_col(l);
    for i = 1:numel(Y{l})
        patch(ax, x_lay(l) + r * cos(th), Y{l}(i) + r * sin(th), c, 'EdgeColor', 'k', 'LineWidth', 0.6);
    end
    if ell(l)
        text(ax, x_lay(l), y_c, '$\vdots$', 'Interpreter', 'latex', 'FontSize', FS + 5, ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
        text(ax, x_lay(l) + 0.36, y_c, sprintf('$\\times\\,%d$', dims(l)), 'Interpreter', 'latex', ...
            'FontSize', FS, 'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
    end
end

% inputs: symbol + description, arrow, z-score block, arrow
yb = [min(Y{1}) - 0.22, max(Y{1}) + 0.22];
local_rbox(ax, x_norm - 0.40, yb(1), 0.80, yb(2) - yb(1), 0.08, [0.93 0.93 0.93], 1, grey);
text(ax, x_norm, y_c + 0.18, '$z$-score', 'Interpreter', 'latex', 'FontSize', FS, 'HorizontalAlignment', 'center');
text(ax, x_norm, y_c - 0.22, '$\frac{x-\mu}{\sigma}$', 'Interpreter', 'latex', 'FontSize', FS + 2, 'HorizontalAlignment', 'center');
for i = 1:dims(1)
    yi = Y{1}(i);
    text(ax, x_sym, yi, in_sym{i}, 'Interpreter', 'latex', 'FontSize', FS + 1, ...
        'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle');
    text(ax, x_sym - 0.62, yi, in_desc{i}, 'FontSize', FS - 1.5, 'Color', [0.35 0.35 0.35], ...
        'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle');
    local_arrow(ax, [x_sym + 0.06, x_norm - 0.42], yi);
    local_arrow(ax, [x_norm + 0.42, x_lay(1) - r - 0.03], yi);
end

% output: neuron -> exp -> prediction
yo = Y{L}(1);
if mdl.log_target
    local_rbox(ax, x_exp - 0.34, yo - 0.25, 0.68, 0.50, 0.15, [0.93 0.93 0.93], 1, grey);
    text(ax, x_exp, yo, '$\exp(\cdot)$', 'Interpreter', 'latex', 'FontSize', FS + 1, 'HorizontalAlignment', 'center');
    local_arrow(ax, [x_lay(L) + r + 0.03, x_exp - 0.36], yo);
    local_arrow(ax, [x_exp + 0.36, x_out - 0.30], yo);
else
    local_arrow(ax, [x_lay(L) + r + 0.03, x_out - 0.30], yo);
end
text(ax, x_out, yo + 0.10, '$\hat{t}_{\mathrm{QP}}$', 'Interpreter', 'latex', 'FontSize', FS + 2, ...
    'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
text(ax, x_out, yo - 0.22, '[ms]', 'FontSize', FS - 1, 'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');

% captions under each block (plain text lines, no LaTeX environments)
y_cap = min(cellfun(@min, Y)) - 0.50;
local_caption(ax, x_norm, y_cap, {'Pre-processing', 'train-set \mu, \sigma'}, FS);
local_caption(ax, x_lay(1), y_cap, {'Input layer', sprintf('%d units', dims(1))}, FS);
for l = 2:L-1
    np = dims(l-1) * dims(l) + dims(l);
    local_caption(ax, x_lay(l), y_cap, {sprintf('Hidden layer %d', l - 1), sprintf('%d units, ReLU', dims(l)), ...
        sprintf('%d params', np)}, FS);
end
if mdl.log_target, otxt = 'linear (log t)'; else, otxt = 'linear'; end
local_caption(ax, x_lay(L), y_cap, {'Output layer', sprintf('1 unit, %s', otxt), sprintf('%d params', dims(L-1) + 1)}, FS);
if mdl.log_target
    local_caption(ax, x_exp, yo + 0.55, {'Post-processing'}, FS);
end

export_figure(fh, 'Fig_6');
end

function local_rbox(ax, x, y, w, h, rad, face, alpha, edge)
% rounded rectangle as a patch (supports transparency in vector output)
t = linspace(0, pi / 2, 8);
cx = [x + w - rad, x + rad, x + rad, x + w - rad];
cy = [y + h - rad, y + h - rad, y + rad, y + rad];
px = [cx(1) + rad * cos(t), cx(2) + rad * cos(t + pi / 2), cx(3) + rad * cos(t + pi), cx(4) + rad * cos(t + 3 * pi / 2)];
py = [cy(1) + rad * sin(t), cy(2) + rad * sin(t + pi / 2), cy(3) + rad * sin(t + pi), cy(4) + rad * sin(t + 3 * pi / 2)];
patch(ax, px, py, face, 'FaceAlpha', alpha, 'EdgeColor', edge, 'LineWidth', 0.8);
end

function local_arrow(ax, xx, y)
c = [0.25 0.25 0.25];
plot(ax, xx, [y, y], '-', 'Color', c, 'LineWidth', 0.7);
d = 0.08;
patch(ax, [xx(2) - d, xx(2), xx(2) - d], [y + 0.045, y, y - 0.045], c, 'EdgeColor', c);
end

function local_caption(ax, x, y, lines, FS)
text(ax, x, y, lines{1}, 'FontSize', FS, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
for k = 2:numel(lines)
    text(ax, x, y - 0.24 * (k - 1), lines{k}, 'FontSize', FS - 1, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
end
end
