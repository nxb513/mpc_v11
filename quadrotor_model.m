function [Ad, Bd, Ac, Bc] = quadrotor_model(Ts, mass_scale)
% QUADROTOR_MODEL 6-DOF linearized hover state-space model with ZOH discretization
% States x = [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]' (12 states)
% Inputs u = [delta_T, tau_x, tau_y, tau_z]' (4 inputs)

if nargin < 2
    mass_scale = 1.0;
end

g = 9.81;
m = 1.25 * mass_scale;
Ixx = 0.0142 * mass_scale;
Iyy = 0.0142 * mass_scale;
Izz = 0.0284 * mass_scale;

Ac = zeros(12, 12);
Ac(1:3, 4:6) = eye(3);
Ac(4, 8) = g;     % ax = g * theta
Ac(5, 7) = -g;    % ay = -g * phi
Ac(7:9, 10:12) = eye(3);

Bc = zeros(12, 4);
Bc(6, 1)  = 1.0 / m;
Bc(10, 2) = 1.0 / Ixx;
Bc(11, 3) = 1.0 / Iyy;
Bc(12, 4) = 1.0 / Izz;

% Exact Zero-Order Hold (ZOH) discretization via matrix exponential
M = [Ac, Bc; zeros(4, 16)];
eM = expm(M * Ts);
Ad = eM(1:12, 1:12);
Bd = eM(1:12, 13:16);
end
