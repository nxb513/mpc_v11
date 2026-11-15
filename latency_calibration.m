function cal = latency_calibration()
% LATENCY_CALIBRATION Calibrates the FLOP-based QP latency model against a
% published embedded benchmark. Each number below is (a) taken from the cited
% source, (b) derived from it, or (c) a stated modelling assumption.
%
% Source [S1]: J. Frey, S. Di Cairano, R. Quirynen, "Active-Set based Inexact
%   Interior Point QP Solver for Model Predictive Control", IFAC World Congress
%   2020 (MERL TR2020-105), Table 2: qpOASES on a Raspberry Pi 2
%   (BCM2836, 900 MHz ARM Cortex-A7, 256 KB L2), inverted pendulum MPC,
%   Ts = 50 ms, N = 50, nx = 4, nu = 2 (force + slack), 4 soft state
%   constraints per stage (eq. 13a-b).
%
% Per-iteration cost of a dense active-set solver (flops):
%   F_iter(n, m) = c_iter * n^2 + 2 * n * m
% with n decision variables and m general inequality rows. The per-step setup
% cost of the LTV-MPC is counted in qp_latency_model.m / true_latency.m.
%
% R_eff (effective flop rate of the embedded CPU running an active-set solver)
% is fitted from the slope of time vs. iterations in [S1, Table 2].

% ---- (a) Published data [S1, Table 2], qpOASES rows (mean values) -----------
cal.src.iters_mean = [0.2, 2.8, 13.3];      % mean # iterations, theta0 = 0.16/0.20/0.24
cal.src.time_mean  = [8.90, 19.30, 59.40];  % mean time [ms]
cal.src.iters_max  = [5, 10, 40];           % worst-case # iterations
cal.src.time_max   = [37.72, 57.51, 162.66];% worst-case time [ms]
cal.src.N  = 50;  cal.src.nx = 4;  cal.src.nu = 2;
cal.src.m_per_stage = 4;                    % soft state constraints, eq. (13a-b)

% ---- (b) Least-squares fit t = a + b * iters on the mean values -------------
Xf = [ones(3, 1), cal.src.iters_mean(:)];
ab = Xf \ cal.src.time_mean(:);
cal.fit.a_ms        = ab(1);   % intercept: per-solve overhead of [S1] (not transferred, see below)
cal.fit.b_ms_per_it = ab(2);   % slope: time per active-set iteration

% ---- (c) assumptions --------------------------------------------------------
% A1: [S1] used qpOASES on the condensed (dense) QP: n = N*nu variables,
%     m = 4*N general inequality rows (input/slack bounds are simple bounds).
% A2: c_iter = 4: per iteration, ~2 n^2 for the step (two triangular solves with
%     the Cholesky factor of the reduced Hessian) + ~2 n^2 for the factor
%     up/down-date when one bound/constraint enters or leaves the working set.
%     The ratio-test term 2*n*m covers products with the constraint matrix.
% A3: Iteration counts of MATLAB quadprog('active-set') are used as a proxy for
%     the iteration counts of an embedded active-set solver.
% A4: The intercept a of [S1] (condensing / initialisation of a larger problem)
%     is not transferred; the per-step overhead is F_setup / R_eff only.
cal.c_iter = 4;

n_src = cal.src.N * cal.src.nu;
m_src = cal.src.m_per_stage * cal.src.N;
cal.src.F_iter = cal.c_iter * n_src^2 + 2 * n_src * m_src;
cal.R_eff_flops_per_ms = cal.src.F_iter / cal.fit.b_ms_per_it;  % Cortex-A7 @ 900 MHz

end
