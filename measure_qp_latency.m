function [z_sol, t_calc_ms, actual_iters, is_converged] = measure_qp_latency(qp, max_iters, cpu_load)
% MEASURE_QP_LATENCY Solves the condensed LTV-MPC QP (build_condensed_qp_ltv.m)
% with MATLAB's active-set quadprog (iteration cap = max_iters), starting from the
% feasible point qp.z0, and returns the latency of the simulated platform.
%
% NOTE (state explicitly in the manuscript): the latency is produced by the
% hidden timing model true_latency.m (calibrated against published qpOASES
% timings on an ARM Cortex-A7, latency_calibration.m); it is not a hardware
% measurement. Only the iteration count comes from the actual solve.

options = optimoptions('quadprog', ...
    'Algorithm', 'active-set', ...
    'MaxIterations', max_iters, ...
    'Display', 'off', ...
    'OptimalityTolerance', 1e-5, ...
    'ConstraintTolerance', 1e-5);

[z_sol, ~, exitflag, output] = quadprog(qp.H, qp.f, qp.A, qp.b, [], [], [], [], qp.z0, options);
if isempty(z_sol)
    z_sol = qp.z0;
end

nx = 12; nu = 4;
N = qp.nU / nu;
actual_iters = max(1, output.iterations);
is_converged = (exitflag == 1);

% Single hardware profile: the calibrated Cortex-A7 (hardware factor 1).
t_calc_ms = true_latency(N, nx, nu, qp.m, actual_iters, cpu_load, 1.0);
end
