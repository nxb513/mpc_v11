function ctx = latency_context(new_ctx)
% LATENCY_CONTEXT Get / set the hidden parameters of the latency generator.
% The generator (true_latency.m) represents the real on-board timing, whose
% parameters are unknown to the predictor and change between flight contexts
% (other SoC, OS overhead, other contention behaviour, timing jitter).
%
%   ctx = latency_context()          returns the active context
%   latency_context(ctx)             activates ctx
%   latency_context('nominal')       resets to the calibrated nominal context
%
% Fields:
%   name        label
%   R_scale     multiplier on the calibrated flop rate R_eff (>1 = faster CPU)
%   c_iter      flops per iteration coefficient (nominal 4, latency_calibration.m)
%   overhead_ms fixed per-solve overhead (OS / scheduling / memory), ms
%   contention  'fluid'  : 1/(1-u)            (nominal, Joseph & Pandya approx.)
%               'power'  : 1 + a*u^b          (different, unknown law)
%   cont_a, cont_b  parameters of the 'power' law
%   jitter_cv   coefficient of variation of multiplicative log-normal jitter

persistent active
if isempty(active)
    active = nominal();
end
if nargin == 1
    if ischar(new_ctx) && strcmpi(new_ctx, 'nominal')
        active = nominal();
    else
        active = fill_defaults(new_ctx);
    end
end
ctx = active;
end

function c = nominal()
c = struct('name', 'nominal', 'R_scale', 1.0, 'c_iter', 4, 'overhead_ms', 0, ...
    'contention', 'fluid', 'cont_a', 0, 'cont_b', 1, 'jitter_cv', 0);
end

function c = fill_defaults(c)
d = nominal();
f = fieldnames(d);
for k = 1:numel(f)
    if ~isfield(c, f{k}), c.(f{k}) = d.(f{k}); end
end
end
