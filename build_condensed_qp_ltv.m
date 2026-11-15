function qp = build_condensed_qp_ltv(trj, k, N, e0, Q, R, P_term, w_max, tilt_max)
% BUILD_CONDENSED_QP_LTV Condensed LTV-MPC tracking QP over horizon N starting
% at reference index k (trj from ltv_model.m).
%
% Decision vector z = [dU; s_w; s_t]: input deviations du_0..du_{N-1} from u_ref
% and two slacks >= 0 that soften the body-rate and tilt limits.
%   min  sum_{i=1}^{N-1} e_i' Q e_i + e_N' P e_N + sum du_i' R du_i
%        + rho2 (s_w^2 + 10 s_t^2) + rho1 (s_w + 10 s_t)
%   s.t. T_min <= f_rotor(u_ref_i + du_i) <= T_max          (hard, 8 rows / stage)
%        |omega_ref_i + e_omega_i| <= w_max + s_w            (soft, 6 rows / stage)
%        |eta_ref_i + e_eta_i| <= tilt_max + s_t, eta = phi, theta (soft, 4 rows / stage)
% Prediction: E = E_free + Su dU, with E_free the propagation of e0 and the
% defects c_i, and Su(i,j) = A_{i-1}...A_j B_{j-1} built recursively.
%
% Returns quadprog data (H, f, A_ineq, b_ineq), a FEASIBLE start z0 (hover
% input and a large enough slack) and sizes.

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

% Soft body-rate constraints on predicted states
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

% Soft tilt limits on predicted Euler angles phi, theta
selt = zeros(2 * N, nx * N);
for i = 1:N
    selt((i-1)*2+1 : i*2, (i-1)*nx+7 : (i-1)*nx+8) = eye(2);
end
eta_ref = reshape(trj.x_ref(7:8, k+1:k+N), [], 1);
St = selt * Su; tf = selt * E_free;
A_t = [ St, zeros(2 * N, 1), -ones(2 * N, 1);
       -St, zeros(2 * N, 1), -ones(2 * N, 1)];
b_t = [ tilt_max - eta_ref - tf;
        tilt_max + eta_ref + tf];
if isinf(tilt_max)          % no tilt envelope: drop the rows
    A_t = zeros(0, nU + 2); b_t = zeros(0, 1);
end

A_s = [zeros(2, nU), -eye(2)];   % s_w, s_t >= 0
b_s = [0; 0];

qp.H = H; qp.f = f;
qp.A = [A_rot; A_w; A_t; A_s];
qp.b = [b_rot; b_w; b_t; b_s];

% Feasible start: hover input, slacks covering any rate / tilt violation
dU0 = -u_ref_stack;
s_w0 = max(0, max(A_w(:, 1:nU) * dU0 - b_w)) + 1e-6;
if isempty(A_t), s_t0 = 1e-6; else, s_t0 = max(0, max(A_t(:, 1:nU) * dU0 - b_t)) + 1e-6; end
qp.z0 = [dU0; s_w0; s_t0];
qp.nU = nU; qp.n = nU + 2; qp.m = size(qp.A, 1);
end
