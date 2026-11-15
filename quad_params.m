function p = quad_params()
% QUAD_PARAMS Physical parameters of the simulated quadrotor: the "Standard
% Quad" of [S2] Table I (thrust-to-weight ratio about 2).
%
% [S2] P. Foehn, A. Romero, D. Scaramuzza, "Time-Optimal Planning for Quadrotor
%      Waypoint Flight", Science Robotics 6(56), eabh1221, 2021, Table I and
%      actuator model eqs. (16)-(17).
%
% Control allocation [S2, eq. 16] (X-configuration, arm length l, torque const c_tau):
%   [T; tau_x; tau_y; tau_z] = Mix * [T1; T2; T3; T4]
% Actuator limits [S2, eq. 17]:  T_min <= T_i <= T_max for every rotor.

p.name   = 'standard';
p.m      = 1.0;                 % [kg]
p.l      = 0.15;                % [m]
p.J      = [5, 5, 10] * 1e-3;   % diag(J) [kg m^2] (Table I lists g m^2)
p.T_min  = 0.25;                % [N] per rotor
p.T_max  = 5.0;                 % [N] per rotor
p.c_tau  = 0.01;                % [-]
p.w_max  = 10;                  % [rad/s] maximum body rate
p.d_drag = 0.0;                 % no drag coefficient is given in Table I
p.g      = 9.81;

s = p.l / sqrt(2);
p.Mix = [ 1,      1,      1,      1;
          s,      s,     -s,     -s;
         -s,      s,      s,     -s;
          p.c_tau, -p.c_tau, p.c_tau, -p.c_tau];
p.Mix_inv = inv(p.Mix);

% Inputs are deviations from hover: u = [dT; tau_x; tau_y; tau_z], total thrust
% T = m*g + dT. Rotor thrusts: f = Mix_inv * (u + u_hover).
p.u_hover = [p.m * p.g; 0; 0; 0];
p.f_hover = p.Mix_inv * p.u_hover;

% Per-stage polytope on u:  G*u <= h  <=>  T_min <= f_i <= T_max for all rotors
p.G = [ p.Mix_inv; -p.Mix_inv];
p.h = [ p.T_max - p.f_hover; -(p.T_min - p.f_hover)];
end
