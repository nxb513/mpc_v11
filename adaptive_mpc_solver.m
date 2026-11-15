function [f_rotor, info, mem] = adaptive_mpc_solver(mode, x_curr, k, trj, Q, R, P_term, K_lqr, T_budget_ms, cpu_load, mem)
% ADAPTIVE_MPC_SOLVER One control step of the LTV-MPC family on the calibrated
% Cortex-A7 platform. trj = ltv_model(...) of the reference; k = time index.
%
% Controllers (all share Q, R, P_term, constraints and fallback):
%   'fixed_5', 'fixed_10' : fixed horizon, iteration cap 40
%   'nominal_max'         : largest (N, I_max) whose latency from the nominal
%                           formula (qp_latency_model.m, the calibration believed
%                           before flight, with I_max iterations as worst case)
%                           fits the budget, every step
%   'proposed'            : largest (N, I_max) whose latency upper bound predicted
%                           by the Tiny-ML quantile model (learned from data,
%                           formula unknown, conformal-calibrated 95 % coverage)
%                           fits the budget, every step
%   'dataset_<N>_<I>'     : explicit (N, I_max) for data collection
%
% Deadline miss: the late QP solution is not applied. The controller applies the
% next input of its last successful plan (shifted plan); if that plan is
% exhausted, the hover-LQR law u_ref - K e is used.
% The applied rotor thrusts are always clipped to [T_min, T_max].
%
% mem (in/out): .plan (4 x Np absolute inputs of the last successful plan),
%               .age (steps since that plan was computed), .last_N.

p = trj.p;
nx = 12; nu = 4;
CAND_N = [4, 6, 8, 10, 12, 16];
CAND_I = [10, 20, 40];
e0 = x_curr - trj.x_ref(:, k);
pos_err_norm = norm(e0(1:3));
vel_err_norm = norm(e0(4:6));
if nargin < 11 || isempty(mem), mem = struct('plan', [], 'age', 0, 'last_N', CAND_N(1)); end

switch lower(mode)
    case 'fixed_5'
        N_sel = 5; iter_sel = 40;
    case 'fixed_10'
        N_sel = 10; iter_sel = 40;
    case {'nominal_max', 'proposed'}
        N_sel = CAND_N(1); iter_sel = CAND_I(1);
        found = false;
        for iN = numel(CAND_N):-1:1
            for iI = numel(CAND_I):-1:1
                if strcmpi(mode, 'proposed')
                    t_hat = tiny_ml_predictor(CAND_N(iN), CAND_I(iI), cpu_load, pos_err_norm, vel_err_norm);   % calibrated upper bound
                else
                    t_hat = qp_latency_model(CAND_N(iN), nx, nu, 14 * CAND_N(iN) + 2, CAND_I(iI), cpu_load, 1.0);
                end
                if t_hat <= T_budget_ms
                    N_sel = CAND_N(iN); iter_sel = CAND_I(iI); found = true; break;
                end
            end
            if found, break; end
        end
    otherwise
        tok = regexp(lower(mode), '^dataset_(\d+)_(\d+)$', 'tokens', 'once');
        if isempty(tok)
            error('adaptive_mpc_solver:mode', 'Unknown controller mode: %s', mode);
        end
        N_sel = str2double(tok{1}); iter_sel = str2double(tok{2});
end

qp = build_condensed_qp_ltv(trj, k, N_sel, e0, Q, R, P_term, p.w_max);
[z, t_calc_ms, actual_iters, is_converged] = measure_qp_latency(qp, iter_sel, cpu_load);
deadline_missed = (t_calc_ms > T_budget_ms);

u_ref0 = trj.u_ref(:, k);
if ~deadline_missed
    dU = reshape(z(1:qp.nU), nu, N_sel);
    u_cmd = u_ref0 + dU(:, 1);
    mem.plan = trj.u_ref(:, k:k+N_sel-1) + dU;   % absolute inputs of this plan
    mem.age = 0;
    fallback = 'none';
else
    mem.age = mem.age + 1;
    if ~isempty(mem.plan) && mem.age < size(mem.plan, 2)
        u_cmd = mem.plan(:, mem.age + 1);         % shifted previous plan
        fallback = 'shift';
    else
        u_cmd = u_ref0 - K_lqr * e0;              % hover LQR
        fallback = 'lqr';
    end
end
f_request = p.Mix_inv * (u_cmd + p.u_hover);
[~, f_rotor] = saturate_motors(u_cmd, p);
mem.last_N = N_sel;

info.t_calc_ms = t_calc_ms;
info.deadline_missed = deadline_missed;
info.N_sel = N_sel;
info.iter_sel = iter_sel;
info.actual_iters = actual_iters;
info.is_converged = is_converged;
info.f_rotor = f_rotor;
info.f_request = f_request;
info.fallback = fallback;
info.m_ineq = qp.m;
end
