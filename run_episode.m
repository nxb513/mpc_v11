function out = run_episode(mode, trj, M, cpu_load, a_dist, Q, R, P_term, K_lqr, T_budget_ms)
% RUN_EPISODE Closed-loop simulation of one controller on the nonlinear plant.
%   trj    : ltv_model of the (padded) reference
%   M      : number of evaluated samples (control steps k = 1..M-1)
%   a_dist : 3 x M external acceleration disturbance (simulate_workload.m)
% Tracking error is evaluated at t_{k+1} against x_ref(t_{k+1}). A run is
% declared crashed if the position error exceeds 5 m (half the arena) or the
% state is not finite; the simulation then stops and the RMSE is NaN.

p = trj.p;
x = trj.x_ref(:, 1);
mem = [];
err2 = nan(1, M-1); miss = zeros(1, M-1); lat = zeros(1, M-1); Nsel = zeros(1, M-1);
f_min = inf; f_max = -inf; sat = 0; w_peak = 0; crashed = false; n_shift = 0; n_lqr = 0;
for k = 1:M-1
    [f, info, mem] = adaptive_mpc_solver(mode, x, k, trj, Q, R, P_term, K_lqr, T_budget_ms, cpu_load(k), mem);
    if any(f < p.T_min - 1e-9) || any(f > p.T_max + 1e-9)
        error('run_episode:actuator', 'Rotor thrust %s N outside [%g, %g] N', mat2str(f', 4), p.T_min, p.T_max);
    end
    x = quad_nl_step(x, f, trj.Ts, p, a_dist(:, k));
    e = norm(x(1:3) - trj.x_ref(1:3, k+1));
    if ~all(isfinite(x)) || e > 5
        crashed = true; break;
    end
    err2(k) = e^2; miss(k) = info.deadline_missed; lat(k) = info.t_calc_ms; Nsel(k) = info.N_sel;
    f_min = min(f_min, min(f)); f_max = max(f_max, max(f));
    sat = sat + any(abs(info.f_request - f) > 1e-9);
    w_peak = max(w_peak, max(abs(x(10:12))));
    n_shift = n_shift + strcmp(info.fallback, 'shift');
    n_lqr = n_lqr + strcmp(info.fallback, 'lqr');
end
n_done = k;
out.crashed = crashed;
out.rmse = sqrt(mean(err2(1:n_done), 'omitnan'));
if crashed, out.rmse = NaN; end
out.miss = 100 * mean(miss(1:n_done));
out.lat_mean = mean(lat(1:n_done));
out.cpu_ms_per_s = sum(lat(1:n_done)) / (n_done * trj.Ts);   % solver time per second of flight
out.sat = 100 * sat / n_done;
out.f_range = [f_min, f_max];
out.w_peak = w_peak;
out.fallback_shift = 100 * n_shift / n_done;
out.fallback_lqr = 100 * n_lqr / n_done;
out.N_mean = mean(Nsel(1:n_done));
out.lat = lat; out.Nsel = Nsel; out.n_done = n_done;
end
