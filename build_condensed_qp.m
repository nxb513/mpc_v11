function [H, f, A_ineq, b_ineq] = build_condensed_qp(Ad, Bd, Q, R, P_term, N, e0, u_min, u_max, ref_shift)
% BUILD_CONDENSED_QP Constructs dense QP matrices for horizon N with preview tracking

if nargin < 10
    ref_shift = zeros(size(Ad, 1) * N, 1);
end

nx = size(Ad, 1);
nu = size(Bd, 2);

Sx = zeros(nx * N, nx);
Su = zeros(nx * N, nu * N);

A_pow = eye(nx);
for i = 1:N
    A_pow = A_pow * Ad;
    Sx((i-1)*nx+1 : i*nx, :) = A_pow;
    for j = 1:i
        A_ij = Ad^(i - j);
        Su((i-1)*nx+1 : i*nx, (j-1)*nu+1 : j*nu) = A_ij * Bd;
    end
end

Q_bar = blkdiag(kron(eye(N-1), Q), P_term);
R_bar = kron(eye(N), R);

H = Su' * Q_bar * Su + R_bar;
H = 0.5 * (H + H') + 1e-6 * eye(nu * N);
f = Su' * Q_bar * (Sx * e0 - ref_shift);

% Input box constraints: u_min <= u_k <= u_max
A_ineq = [eye(nu * N); -eye(nu * N)];
b_ineq = [repmat(u_max, N, 1); repmat(-u_min, N, 1)];
end
