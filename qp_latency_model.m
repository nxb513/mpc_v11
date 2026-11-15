function [t_ms, F_setup, F_iter] = qp_latency_model(N, nx, nu, m_ineq, iters, cpu_load, hw_factor)
% QP_LATENCY_MODEL Nominal FLOP-based latency of one LTV-MPC step (online
% condensing + active-set QP with m_ineq inequality rows) on the CPU calibrated
% in latency_calibration.m. It is the nominal model used by the nominal-model
% scheduler; the simulator's ground truth is true_latency.m.
%
%   t = slowdown(cpu_load) * hw_factor * (F_setup + iters * F_iter) / R_eff
%
% Per-iteration form as in the calibration: F_iter = c_iter*n^2 + 2*n*m_ineq.
%
% CPU contention (assumption A5): the control task receives only the idle
% fraction (1 - cpu_load) of its core, i.e. background tasks have precedence.
% This is the fluid approximation of response-time analysis under higher-priority
% interference, R = C / (1 - U_hp) (cf. Joseph & Pandya, The Computer Journal,
% 1986). It is an approximation, not a measurement.

persistent cal
if isempty(cal)
    cal = latency_calibration();
end

nU = N * nu;
n  = nU + 1;                                  % decision variables (inputs + 1 slack)
F_iter  = cal.c_iter * n^2 + 2 * n * m_ineq;
% Online LTV condensing (matrices change every step), flop counts:
%   free response 2 N nx^2, Su blocks 2 N^2 nx^2 nu, Q*Su and Su'*Q*Su
%   nu^2 nx N^3, gradient 2 nu nx N^2, Cholesky of H (nu N)^3 / 3
F_setup = 2 * N * nx^2 + 2 * N^2 * nx^2 * nu + nu^2 * nx * N^3 + 2 * nu * nx * N^2 + nU^3 / 3;

t_nominal = (F_setup + iters * F_iter) / cal.R_eff_flops_per_ms;   % ms, idle core
slowdown  = 1 / max(0.05, 1 - cpu_load);

t_ms = hw_factor * slowdown * t_nominal;
end
