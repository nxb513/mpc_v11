function qp = build_condensed_qp_ltv(trj, k, N, e0, Q, R, P_term, w_max)
% BUILD_CONDENSED_QP_LTV Condensed LTV-MPC tracking QP over horizon N starting
% at reference index k (trj from ltv_model.m).
%
% Decision vector z = [dU; s_1; s_2]: input deviations du_0..du_{N-1} from u_ref
% and two non-negative slack variables. s_1 softens the body-rate limit; s_2 is
% not attached to any constraint in this study (s_2 = 0 at the optimum) and only
% enters the problem dimension n = 4N + 2 used by the timing model.
%   min  sum_{i=1}^{N-1} e_i' Q e_i + e_N' P e_N + sum du_i' R du_i
%        + rho2 (s_1^2 + 10 s_2^2) + rho1 (s_1 + 10 s_2)
%   s.t. T_min <= f_rotor(u_ref_i + du_i) <= T_max          (hard, 8 rows / stage)
%        |omega_ref_i + e_omega_i| <= w_max + s_1            (soft, 6 rows / stage)
%        s_1 >= 0, s_2 >= 0
% Prediction: E = E_free + Su dU, with E_free the propagation of e0 and the
% defects c_i, and Su(i,j) = A_{i-1}...A_j B_{j-1} built recursively.
%
% Returns quadprog data (H, f, A, b), a feasible start z0 (hover input and a
% sufficiently large slack) and the problem sizes.

p = trj.p;
nx = size(trj.A, 1); nu = size(trj.B, 2);
nU = nu * N;
rho2 = 1e3; rho1 = 1e3;

% Free response and input-to-error map (recursive, time-varying)
E_free = zeros(nx * N, 1);
Su = zeros(nx * N, nU);
e = e0;
for i = 1:N
    kk = k + i - 1;
    Ai = trj.A(:, :, kk); Bi = trj.B(:, :, kk);
    e = Ai * e + trj.c(:, kk);
    r = (i-1)*nx+1 : i*nx;
    E_free(r) = e;
    if i > 1
        Su(r, 1:(i-1)*nu) = Ai * Su(r - nx, 1:(i-1)*nu);
    end
    Su(r, (i-1)*nu+1 : i*nu) = Bi;
end

Q_bar = blkdiag(kron(eye(N-1), Q), P_term);
QSu = Q_bar * Su;
H_u = Su' * QSu + kron(eye(N), R);
H_u = 0.5 * (H_u + H_u') + 1e-6 * eye(nU);
f_u = QSu' * E_free;

H = blkdiag(H_u, rho2, 10 * rho2);
f = [f_u; rho1; 10 * rho1];

% Rotor-thrust constraints on the absolute input
u_ref_stack = reshape(trj.u_ref(:, k:k+N-1), [], 1);
A_rot = [kron(eye(N), p.G), zeros(8 * N, 2)];
b_rot = repmat(p.h, N, 1) - kron(eye(N), p.G) * u_ref_stack;

% Soft body-rate constraints on the predicted states
sel = zeros(3 * N, nx * N);
for i = 1:N
    sel((i-1)*3+1 : i*3, (i-1)*nx+10 : (i-1)*nx+12) = eye(3);
end
w_ref = reshape(trj.x_ref(10:12, k+1:k+N), [], 1);
Sw = sel * Su; wf = sel * E_free;
A_w = [ Sw, -ones(3 * N, 1), zeros(3 * N, 1);
       -Sw, -ones(3 * N, 1), zeros(3 * N, 1)];
b_w = [ w_max - w_ref - wf;
        w_max + w_ref + wf];

A_s = [zeros(2, nU), -eye(2)];   % s_1, s_2 >= 0
b_s = [0; 0];

qp.H = H; qp.f = f;
qp.A = [A_rot; A_w; A_s];
qp.b = [b_rot; b_w; b_s];

% Feasible start: hover input, s_1 covering any rate violation
dU0 = -u_ref_stack;
s1 = max(0, max(A_w(:, 1:nU) * dU0 - b_w)) + 1e-6;
qp.z0 = [dU0; s1; 1e-6];
qp.nU = nU; qp.n = nU + 2; qp.m = size(qp.A, 1);
end
