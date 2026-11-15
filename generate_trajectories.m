function [x_ref, u_ref, stats] = generate_trajectories(traj_type, Ts, T_sim, aggr)
% GENERATE_TRAJECTORIES Smooth 3D reference trajectories and the matching state /
% input references of the nonlinear quadrotor (quad_nl_dynamics.m), obtained by
% differential flatness (yaw fixed to zero).
%
%   aggr : aggressiveness level (default 1). It scales the angular frequency of
%          the trajectory, so speed scales with aggr and acceleration with aggr^2.
%
% Flatness map (drag d, Z-Y-X Euler angles, psi = 0):
%   t   = m (a + d v + g e3),  T = |t|,  z_b = t / T
%   phi = -asin(z_b,y),  theta = atan2(z_b,x, z_b,z)
%   omega from the derivative of z_b (uses jerk), tau = J omega_dot + omega x J omega
%   (omega_dot by central differences of the analytic omega(t), h = 1e-4 s).
% u_ref = [T - m g; tau] (thrust deviation from hover, body torques).
% The reference may demand more than the rotors can give; stats.ff_over is the
% share of samples (%) whose feed-forward rotor thrusts leave [T_min, T_max].

if nargin < 4 || isempty(aggr), aggr = 1.0; end
p = quad_params();
t = 0:Ts:T_sim;

[x_ref, u_ref, aux] = flat_reference(traj_type, t, aggr, p);

tilt = acos(min(1, aux.zb(3, :)));
f_ff = p.Mix_inv * (u_ref + p.u_hover);
speeds = sqrt(sum(x_ref(4:6, :).^2, 1));
stats.min = min(speeds); stats.max = max(speeds); stats.mean = mean(speeds);
stats.tilt_max_deg = rad2deg(max(tilt));
stats.f_ff_range = [min(f_ff(:)), max(f_ff(:))];
stats.rate_max = max(max(abs(x_ref(10:12, :))));
stats.ff_over = 100 * mean(any(f_ff > p.T_max | f_ff < p.T_min, 1));
end

% =============================================================================
function [x_ref, u_ref, aux] = flat_reference(traj_type, t, aggr, p)
h = 1e-4;
[eta, w, T, zb, P] = flat_state(traj_type, t, aggr, p);
[~, w_p] = flat_state(traj_type, t + h, aggr, p);
[~, w_m] = flat_state(traj_type, t - h, aggr, p);
w_dot = (w_p - w_m) / (2 * h);
J = diag(p.J);
tau = J * w_dot + cross(w, J * w, 1);

x_ref = [P(:, :, 1); P(:, :, 2); eta; w];
u_ref = [T - p.m * p.g; tau];
aux.zb = zb;
end

function [eta, w, T, zb, P] = flat_state(traj_type, t, aggr, p)
P = position_derivatives(traj_type, t, aggr);
v = P(:, :, 2); a = P(:, :, 3); j = P(:, :, 4);
e3 = [0; 0; 1];
tv  = p.m * (a + p.d_drag * v + p.g * e3);
tdv = p.m * (j + p.d_drag * a);
T  = sqrt(sum(tv.^2, 1));
zb = tv ./ T;
zd = (tdv - zb .* sum(zb .* tdv, 1)) ./ T;

phi = -asin(zb(2, :));
th  = atan2(zb(1, :), zb(3, :));
phid = -zd(2, :) ./ cos(phi);
thd  = (zb(3, :) .* zd(1, :) - zb(1, :) .* zd(3, :)) ./ (zb(1, :).^2 + zb(3, :).^2);
eta = [phi; th; zeros(size(phi))];
% omega = W^{-1} * eta_dot, with psi_dot = 0
w = [phid - sin(th) .* 0;
     cos(phi) .* thd;
    -sin(phi) .* thd];
end

function P = position_derivatives(traj_type, t, aggr)
% P(:, :, n+1) = n-th time derivative of the position, n = 0..4 (analytic).
% Shapes are sized to an indoor flight arena of about 10 m x 10 m x 5 m
% (|x|, |y| <= 4.5 m, 0.5 m <= z <= 4.5 m). Base angular frequencies give
% about 2 m/s peak speed at aggr = 1; difficulty is set by aggr (scenario_levels.m).
M = numel(t);
sdn = @(w, n) w^n * sin(w * t + n * pi / 2);
cdn = @(w, n) w^n * cos(w * t + n * pi / 2);
P = zeros(3, M, 5);
switch lower(traj_type)
    case 'lemniscate'      % figure-8 (Lemniscate of Gerono), 8 m x 6 m
        A = 4.0; B = 6.0; C = 1.0; w = 0.45 * aggr;
        for n = 0:4
            P(1, :, n+1) = A * sdn(w, n);
            P(2, :, n+1) = 0.5 * B * sdn(2*w, n);
            P(3, :, n+1) = C * sdn(w, n);
        end
        P(3, :, 1) = P(3, :, 1) + 2.5;
    case 'helix'           % circle of radius 3.5 m with a +/-1.5 m vertical wave
        R0 = 3.5; Hz = 1.5; w = 0.55 * aggr;
        for n = 0:4
            P(1, :, n+1) = R0 * cdn(w, n);
            P(2, :, n+1) = R0 * sdn(w, n);
            P(3, :, n+1) = Hz * sdn(0.5 * w, n);
        end
        P(3, :, 1) = P(3, :, 1) + 2.5;
    case 'racing'          % multi-harmonic circuit, about 9 m x 8 m
        w = 0.40 * aggr;
        for n = 0:4
            P(1, :, n+1) = 3.5 * sdn(w, n) + 1.0 * sdn(3*w, n);
            P(2, :, n+1) = 3.2 * cdn(w, n) - 0.8 * cdn(2*w, n);
            P(3, :, n+1) = 0.8 * sdn(2*w, n);
        end
        P(3, :, 1) = P(3, :, 1) + 2.5;
    otherwise
        error('Unknown trajectory type: %s', traj_type);
end
end
