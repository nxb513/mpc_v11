function [Ad, Bd] = quadrotor_model(Ts)
% QUADROTOR_MODEL 12-state hover linearisation of quad_nl_dynamics.m with exact
% zero-order-hold discretisation. Used for the terminal cost (DARE) and the
% hover-LQR fallback.
% States x = [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]'
% Inputs u = [dT, tau_x, tau_y, tau_z]' (dT = total thrust minus hover thrust)

p = quad_params();
g = p.g;

Ac = zeros(12, 12);
Ac(1:3, 4:6) = eye(3);
Ac(4:6, 4:6) = -p.d_drag * eye(3);
Ac(4, 8) = g;      % ax =  g * theta
Ac(5, 7) = -g;     % ay = -g * phi
Ac(7:9, 10:12) = eye(3);

Bc = zeros(12, 4);
Bc(6, 1)  = 1.0 / p.m;
Bc(10, 2) = 1.0 / p.J(1);
Bc(11, 3) = 1.0 / p.J(2);
Bc(12, 4) = 1.0 / p.J(3);

eM = expm([Ac, Bc; zeros(4, 16)] * Ts);
Ad = eM(1:12, 1:12);
Bd = eM(1:12, 13:16);
end
