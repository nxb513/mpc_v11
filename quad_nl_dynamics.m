function xdot = quad_nl_dynamics(x, f_rotor, p, a_ext)
% QUAD_NL_DYNAMICS Nonlinear 6-DOF rigid-body quadrotor dynamics [S2, eqs. 1-4, 16]
% with Z-Y-X Euler angles.
%   x       = [pos(3); vel(3); phi; theta; psi; omega_body(3)]
%   f_rotor = rotor thrusts [N] (already saturated to [T_min, T_max])
%   p       = quad_params() (optionally perturbed m, J)
%   a_ext   = external acceleration in the world frame (wind gust), 3x1
%
%   v_dot     = R(eta) * [0; 0; T] / m - [0; 0; g] - d_drag * v + a_ext
%   eta_dot   = W(phi, theta) * omega
%   omega_dot = J^{-1} (tau - omega x J omega)
%   [T; tau]  = Mix * f_rotor

if nargin < 4, a_ext = zeros(3, 1); end
v   = x(4:6);
phi = x(7); th = x(8); psi = x(9);
w   = x(10:12);

wrench = p.Mix * f_rotor;
T = wrench(1); tau = wrench(2:4);

cph = cos(phi); sph = sin(phi); cth = cos(th); sth = sin(th); cps = cos(psi); sps = sin(psi);
% body z-axis expressed in the world frame (third column of R = Rz*Ry*Rx)
zb = [cps * sth * cph + sps * sph;
      sps * sth * cph - cps * sph;
      cth * cph];

J = diag(p.J);
xdot = zeros(12, 1);
xdot(1:3)   = v;
xdot(4:6)   = zb * T / p.m - [0; 0; p.g] - p.d_drag * v + a_ext;
W = [1, sph * sth / cth, cph * sth / cth;
     0, cph,            -sph;
     0, sph / cth,       cph / cth];
xdot(7:9)   = W * w;
xdot(10:12) = J \ (tau - cross(w, J * w));
end
