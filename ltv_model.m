function trj = ltv_model(x_ref, u_ref, Ts, p)
% LTV_MODEL Linear time-varying prediction model of the nonlinear quadrotor
% along the reference (x_ref, u_ref), for every reference sample k:
%   e_{k+1} = A_k e_k + B_k du_k + c_k,
%   A_k, B_k : ZOH discretisation of the Jacobians of quad_nl_dynamics at
%              (x_ref_k, u_ref_k) (central differences, then matrix exponential)
%   c_k      : defect F(x_ref_k, u_ref_k) - x_ref_{k+1}, F = nonlinear RK4 step.
% The reference is fixed, so the model is computed once per trajectory and reused
% by all controllers; the ONLINE cost of condensing it into a QP is accounted
% for in the latency model (qp_latency_model.m / true_latency.m).

if nargin < 4, p = quad_params(); end
[nx, M] = size(x_ref); nu = size(u_ref, 1);
A = zeros(nx, nx, M); B = zeros(nx, nu, M); c = zeros(nx, M);
dx = 1e-6; du = 1e-6;
for k = 1:M
    xr = x_ref(:, k); ur = u_ref(:, k);
    fr = @(x, u) quad_nl_dynamics(x, p.Mix_inv * (u + p.u_hover), p);
    Ac = zeros(nx); Bc = zeros(nx, nu);
    for i = 1:nx
        e = zeros(nx, 1); e(i) = dx;
        Ac(:, i) = (fr(xr + e, ur) - fr(xr - e, ur)) / (2 * dx);
    end
    for i = 1:nu
        e = zeros(nu, 1); e(i) = du;
        Bc(:, i) = (fr(xr, ur + e) - fr(xr, ur - e)) / (2 * du);
    end
    eM = expm([Ac, Bc; zeros(nu, nx + nu)] * Ts);
    A(:, :, k) = eM(1:nx, 1:nx);
    B(:, :, k) = eM(1:nx, nx+1:end);
    if k < M
        c(:, k) = quad_nl_step(xr, p.Mix_inv * (ur + p.u_hover), Ts, p) - x_ref(:, k+1);
    end
end
trj.x_ref = x_ref; trj.u_ref = u_ref; trj.A = A; trj.B = B; trj.c = c; trj.Ts = Ts; trj.p = p;
end
