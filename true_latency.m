function t_ms = true_latency(N, nx, nu, m_ineq, iters, cpu_load, hw_factor)
% TRUE_LATENCY "Real" QP latency used by the simulator (the hidden ground truth
% that the Tiny-ML predictor must learn from data). It equals the calibrated
% nominal model (qp_latency_model.m) when latency_context() is 'nominal', and
% deviates from it in the other contexts (see latency_context.m).

ctx = latency_context();
cal = latency_calibration_cached();

nU = N * nu;
n  = nU + 2;                                  % decision variables (inputs + 2 slacks)
F_iter  = ctx.c_iter * n^2 + 2 * n * m_ineq;
% Online LTV condensing (matrices change every step), flop counts:
%   free response 2 N nx^2, Su blocks 2 N^2 nx^2 nu, Q*Su and Su'*Q*Su
%   nu^2 nx N^3, gradient 2 nu nx N^2, Cholesky of H (nu N)^3 / 3
F_setup = 2 * N * nx^2 + 2 * N^2 * nx^2 * nu + nu^2 * nx * N^3 + 2 * nu * nx * N^2 + nU^3 / 3;
t_core  = hw_factor * (F_setup + iters * F_iter) / (cal.R_eff_flops_per_ms * ctx.R_scale) + ctx.overhead_ms;

switch lower(ctx.contention)
    case 'fluid'
        slowdown = 1 / max(0.05, 1 - cpu_load);
    case 'power'
        slowdown = 1 + ctx.cont_a * cpu_load^ctx.cont_b;
    otherwise
        error('true_latency:contention', 'Unknown contention law %s', ctx.contention);
end

t_ms = slowdown * t_core;
if ctx.jitter_cv > 0
    s = sqrt(log(1 + ctx.jitter_cv^2));
    t_ms = t_ms * exp(s * randn() - 0.5 * s^2);   % mean-preserving log-normal jitter
end
end

function cal = latency_calibration_cached()
persistent c
if isempty(c), c = latency_calibration(); end
cal = c;
end
