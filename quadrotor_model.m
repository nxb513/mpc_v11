function [Ad, Bd, Ac, Bc, gd] = quadrotor_model(Ts, mass_scale, inertia_scale)
% QUADROTOR_MODEL 12-state linearised hover model with exact ZOH discretisation.
% States x = [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]'
% Inputs u = [dT, tau_x, tau_y, tau_z]'  (dT = total thrust minus NOMINAL hover thrust)
% Parameters: quad_params('race') [S2, Table I], incl. linear drag d_drag.
%
% mass_scale / inertia_scale perturb the TRUE plant (Monte Carlo). Because the
% controller compensates the nominal hover thrust m_nom*g, a mass error leaves a
% residual vertical acceleration a_off = g*(m_nom/m_true - 1), returned as the
% discrete affine term gd (zero for the nominal model).

if nargin < 2 || isempty(mass_scale),    mass_scale = 1.0;    end
if nargin < 3 || isempty(inertia_scale), inertia_scale = 1.0; end

p = quad_params();
g = p.g;
m = p.m * mass_scale;
J = p.J * inertia_scale;

Ac = zeros(12, 12);
Ac(1:3, 4:6) = eye(3);
Ac(4:6, 4:6) = -p.d_drag * eye(3);   % linear drag [S2]
Ac(4, 8) = g;      % ax =  g * theta
Ac(5, 7) = -g;     % ay = -g * phi
Ac(7:9, 10:12) = eye(3);

Bc = zeros(12, 4);
Bc(6, 1)  = 1.0 / m;
Bc(10, 2) = 1.0 / J(1);
Bc(11, 3) = 1.0 / J(2);
Bc(12, 4) = 1.0 / J(3);

% Constant disturbance input: residual vertical acceleration from mass mismatch
Ec = zeros(12, 1);
Ec(6) = 1;
a_off = g * (1 / mass_scale - 1);

% Exact ZOH discretisation via the augmented matrix exponential
nx = 12; nu = 5;
Mx = [Ac, [Bc, Ec]; zeros(nu, nx + nu)];
eM = expm(Mx * Ts);
Ad = eM(1:nx, 1:nx);
Bd = eM(1:nx, nx+1:nx+4);
gd = eM(1:nx, nx+5) * a_off;
end
