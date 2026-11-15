function p = quad_params(name)
% QUAD_PARAMS Physical parameters of the simulated quadrotor.
%
% Source [S2]: P. Foehn, A. Romero, D. Scaramuzza, "Time-Optimal Planning for
%   Quadrotor Waypoint Flight", Science Robotics 6(56), eabh1221, 2021,
%   Table I "Quadrotor Configurations", and actuator model eqs. (16)-(17).
%
%   'standard' : Standard Quad  (default; thrust-to-weight ~2, so aggressive
%                references can demand more than the rotors can give)
%   'race'     : Race Quad      (thrust-to-weight ~4)
%
% Control allocation [S2, eq. 16] (X-configuration, arm length l, torque const c_tau):
%   [T; tau_x; tau_y; tau_z] = Mix * [T1; T2; T3; T4]
% Actuator limits [S2, eq. 17]:  T_min <= T_i <= T_max  for every rotor.

if nargin < 1 || isempty(name)
    name = 'standard';
end

switch lower(name)
    case 'race'
        p.m      = 0.8;               % [kg]
        p.l      = 0.15;              % [m]
        p.J      = [1, 1, 1.7] * 1e-3;% diag(J) [kg m^2]  (Table I lists g m^2)
        p.T_min  = 0.0;               % [N] per rotor
        p.T_max  = 8.0;               % [N] per rotor
        p.c_tau  = 0.01;              % [-]
        p.w_max  = 15;                % [rad/s] max body rate
        p.d_drag = 0.4;               % [1/s] linear drag
    case 'standard'
        p.m      = 1.0;
        p.l      = 0.15;
        p.J      = [5, 5, 10] * 1e-3;
        p.T_min  = 0.25;
        p.T_max  = 5.0;
        p.c_tau  = 0.01;
        p.w_max  = 10;
        p.d_drag = 0.0;               % not given in Table I for this platform
    otherwise
        error('quad_params:name', 'Unknown platform: %s', name);
end
p.name = lower(name);
p.g = 9.81;

% No tilt envelope is imposed (Inf): the only constraints are the physical
% actuator limits of Table I and the body-rate limit w_max.
p.tilt_max = Inf;

s = p.l / sqrt(2);
p.Mix = [ 1,      1,      1,      1;
          s,      s,     -s,     -s;
         -s,      s,      s,     -s;
          p.c_tau, -p.c_tau, p.c_tau, -p.c_tau];
p.Mix_inv = inv(p.Mix);

% Inputs of the linear model are deviations from hover: u = [dT; tau_x; tau_y; tau_z]
% with total thrust T = m*g + dT. Rotor thrusts: f = Mix_inv * (u + u_hover).
p.u_hover = [p.m * p.g; 0; 0; 0];
p.f_hover = p.Mix_inv * p.u_hover;

% Per-stage polytope on u:  G*u <= h  <=>  T_min <= f_i <= T_max for all rotors
p.G = [ p.Mix_inv; -p.Mix_inv];
p.h = [ p.T_max - p.f_hover; -(p.T_min - p.f_hover)];
end
