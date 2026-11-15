function x = quad_nl_step(x, f_rotor, Ts, p, a_ext)
% QUAD_NL_STEP Integrates the nonlinear quadrotor over one sampling period with
% zero-order-hold rotor thrusts and constant external acceleration, using
% classical RK4 with sub-steps of at most 2.5 ms.
if nargin < 5, a_ext = zeros(3, 1); end
n_sub = max(1, ceil(Ts / 0.0025));
h = Ts / n_sub;
for s = 1:n_sub
    k1 = quad_nl_dynamics(x, f_rotor, p, a_ext);
    k2 = quad_nl_dynamics(x + 0.5 * h * k1, f_rotor, p, a_ext);
    k3 = quad_nl_dynamics(x + 0.5 * h * k2, f_rotor, p, a_ext);
    k4 = quad_nl_dynamics(x + h * k3, f_rotor, p, a_ext);
    x = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
end
end
