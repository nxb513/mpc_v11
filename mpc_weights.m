function [Q, R, tol] = mpc_weights(level, p)
% MPC_WEIGHTS Stage-cost weights by Bryson's rule (Bryson & Ho, Applied Optimal
% Control, 1975; Franklin, Powell & Emami-Naeini, Feedback Control of Dynamic
% Systems): Q_ii = 1 / (largest acceptable deviation of state i)^2,
%            R_jj = 1 / (largest input j)^2.
%
% Largest inputs follow from the actuator limits of quad_params.m ([S2] Table I):
%   thrust deviation : max(4 T_max - m g, m g - 4 T_min)
%   roll/pitch torque: (l/sqrt(2)) * 2 (T_max - T_min)   (two rotors up, two down)
%   yaw torque       : c_tau * 2 (T_max - T_min)
% Acceptable deviations are a design choice; two levels are provided and both
% are reported:
%   'tight' : position 0.1 m, velocity 0.5 m/s, angles 0.2 rad, rates 1 rad/s
%   'loose' : position 0.3 m, velocity 1.0 m/s, angles 0.5 rad, rates 2 rad/s

if nargin < 1 || isempty(level), level = 'tight'; end
if nargin < 2, p = quad_params(); end
switch lower(level)
    case 'tight', tol = struct('pos', 0.1, 'vel', 0.5, 'ang', 0.2, 'rate', 1.0);
    case 'loose', tol = struct('pos', 0.3, 'vel', 1.0, 'ang', 0.5, 'rate', 2.0);
    otherwise, error('mpc_weights:level', 'Unknown level %s', level);
end
dev = [tol.pos * [1 1 1], tol.vel * [1 1 1], tol.ang * [1 1 1], tol.rate * [1 1 1]];
Q = diag(1 ./ dev.^2);

s = p.l / sqrt(2);
dT_max   = max(4 * p.T_max - p.m * p.g, p.m * p.g - 4 * p.T_min);
tauxy_max = s * 2 * (p.T_max - p.T_min);
tauz_max  = p.c_tau * 2 * (p.T_max - p.T_min);
R = diag(1 ./ [dT_max, tauxy_max, tauxy_max, tauz_max].^2);
end
