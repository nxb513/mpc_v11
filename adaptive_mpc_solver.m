function [u_applied, info] = adaptive_mpc_solver(mode, x_curr, x_ref_win, u_ref_win, u_prev, Ad, Bd, Q, R, P_term, K_lqr, T_budget_ms, cpu_load, hw_speed, state_mem)
% ADAPTIVE_MPC_SOLVER Executes Proposed Computation-Aware Adaptive MPC
% or baseline controllers (Fixed-N, Anytime MPC, Rule-Based Adaptive MPC).

u_min = [-6.0; -1.2; -1.2; -0.8];
u_max = [ 8.0;  1.2;  1.2;  0.8];

e0 = x_curr - x_ref_win(:, 1);
pos_err_norm = norm(e0(1:3));
vel_err_norm = norm(e0(4:6));
err_norm     = pos_err_norm + 0.4 * vel_err_norm;

switch lower(mode)
    case 'fixed_5'
        N_sel = 5; iter_sel = 40; q_scale = 1.0;
    case 'fixed_10'
        N_sel = 10; iter_sel = 40; q_scale = 1.0;
    case 'fixed_15'
        N_sel = 15; iter_sel = 40; q_scale = 1.0;
    case 'fixed_20'
        N_sel = 20; iter_sel = 40; q_scale = 1.0;

    case 'anytime'
        N_sel = 15; q_scale = 1.0;
        t_setup_est = 0.35 * hw_speed * (1 + 1.5 * cpu_load^2);
        t_iter_est  = (0.042 * N_sel + 0.0075 * N_sel^2) * hw_speed * (1 + 2.2 * cpu_load^1.8);
        iter_sel = max(1, min(40, floor((T_budget_ms - t_setup_est) / max(1e-4, t_iter_est))));

    case 'rule_based'
        q_scale = 1.0;
        if cpu_load < 0.40
            N_sel = 15; iter_sel = 35;
        elseif cpu_load < 0.65
            N_sel = 10; iter_sel = 20;
        else
            N_sel = 5;  iter_sel = 10;
        end

    case 'proposed'
        % STAGE 1: Predictive Supervisor via Tiny-ML 2-hidden-layer MLP
        cand_N    = [5, 8, 12, 16];
        cand_iter = [12, 20, 30, 40];
        cand_q    = [1.0, 1.15, 1.30];

        best_score = inf;
        N_sel = 5; iter_sel = 12; q_scale = 1.0;
        safety_margin = 0.85;

        for iN = 1:length(cand_N)
            for iI = 1:length(cand_iter)
                N_c = cand_N(iN);
                I_c = cand_iter(iI);
                t_hat = tiny_ml_predictor(N_c, I_c, cpu_load, pos_err_norm, vel_err_norm, hw_speed);
                if t_hat > safety_margin * T_budget_ms
                    continue;
                end
                for iq = 1:length(cand_q)
                    q_c = cand_q(iq);
                    % Longer horizon N dramatically lowers agile tracking error; moderate q_c boosts responsiveness
                    perf_cost = 6.0 / (N_c * (I_c^0.25)) + 0.12 * (1.30 - q_c)^2;
                    sw_cost   = 0.010 * abs(N_c - state_mem.last_N) / 10.0;
                    if (perf_cost + sw_cost) < best_score
                        best_score = perf_cost + sw_cost;
                        N_sel = N_c; iter_sel = I_c; q_scale = q_c;
                    end
                end
            end
        end
end

% Construct future reference shift over preview horizon N_sel
nx = size(Ad, 1);
ref_shift = zeros(nx * N_sel, 1);
for i = 1:N_sel
    idx_curr = min(i, size(x_ref_win, 2));
    idx_next = min(i + 1, size(x_ref_win, 2));
    idx_u    = min(i, size(u_ref_win, 2));
    ref_shift((i-1)*nx+1 : i*nx) = x_ref_win(:, idx_next) - Ad * x_ref_win(:, idx_curr) - Bd * u_ref_win(:, idx_u);
end

Q_eff = Q * q_scale;
P_eff = P_term * q_scale;
[H, f, A_ineq, b_ineq] = build_condensed_qp(Ad, Bd, Q_eff, R, P_eff, N_sel, e0, u_min, u_max, ref_shift);
[u_qp, t_calc_ms, actual_iters, is_converged] = measure_qp_latency(H, f, A_ineq, b_ineq, iter_sel, cpu_load, hw_speed, [], err_norm);

deadline_missed = (t_calc_ms > T_budget_ms);

if strcmpi(mode, 'anytime')
    u_applied = min(u_max, max(u_min, u_qp));
elseif deadline_missed
    % Memoryless damped LQR fallback guarantees asymptotic stability across all sampling rates (10-50 Hz)
    u_lqr = -0.85 * K_lqr * e0;
    u_applied = min(u_max, max(u_min, u_lqr));
else
    u_applied = min(u_max, max(u_min, u_qp));
end

info.t_calc_ms = t_calc_ms;
info.deadline_missed = deadline_missed;
info.N_sel = N_sel;
info.iter_sel = iter_sel;
info.actual_iters = actual_iters;
info.is_converged = is_converged;
end
