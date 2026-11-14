function [x_ref, u_ref, vel_stats] = generate_trajectories(traj_type, Ts, T_sim)
% GENERATE_TRAJECTORIES Generates smooth 3D trajectories with agile speed (2.3 - 16.8 m/s)

t = 0:Ts:T_sim;
M = length(t);
x_ref = zeros(12, M);
u_ref = zeros(4, M);

switch lower(traj_type)
    case 'lemniscate'
        % High-speed 3D Figure-8 (Lemniscate of Gerono)
        A = 14.0; B = 9.0; C = 2.5;
        w = 0.72;
        x  = A * sin(w * t);
        y  = B * sin(2 * w * t) * 0.5;
        z  = 4.0 + C * sin(w * t);
        vx = A * w * cos(w * t);
        vy = B * w * cos(2 * w * t);
        vz = C * w * cos(w * t);
        ax = -A * w^2 * sin(w * t);
        ay = -2 * B * w^2 * sin(2 * w * t);
        az = -C * w^2 * sin(w * t);

    case 'helix'
        % Expanding 3D Climbing Helix
        R0 = 6.5; alpha = 0.35;
        w = 1.05; vz_const = 1.2;
        R_t = R0 + alpha * t;
        x  = R_t .* cos(w * t);
        y  = R_t .* sin(w * t);
        z  = 2.0 + vz_const * t;
        vx = alpha * cos(w * t) - R_t .* w .* sin(w * t);
        vy = alpha * sin(w * t) + R_t .* w .* cos(w * t);
        vz = vz_const * ones(size(t));
        ax = -2 * alpha * w * sin(w * t) - R_t .* w^2 .* cos(w * t);
        ay =  2 * alpha * w * cos(w * t) - R_t .* w^2 .* sin(w * t);
        az = zeros(size(t));

    case 'racing'
        % Multi-harmonic Agile Racing Circuit
        w = 0.65;
        x  = 15.0 * sin(w * t) + 3.5 * sin(3 * w * t);
        y  = 11.0 * cos(w * t) - 2.5 * cos(2 * w * t);
        z  = 3.5 + 2.0 * sin(2 * w * t);
        vx = 15.0 * w * cos(w * t) + 10.5 * w * cos(3 * w * t);
        vy = -11.0 * w * sin(w * t) + 5.0 * w * sin(2 * w * t);
        vz = 4.0 * w * cos(2 * w * t);
        ax = -15.0 * w^2 * sin(w * t) - 31.5 * w^2 * sin(3 * w * t);
        ay = -11.0 * w^2 * cos(w * t) + 10.0 * w^2 * cos(2 * w * t);
        az = -8.0 * w^2 * sin(2 * w * t);

    otherwise
        error('Unknown trajectory type: %s', traj_type);
end

g = 9.81;
m = 1.25;
phi_ref   = -ay / g;
theta_ref =  ax / g;
psi_ref   = zeros(size(t));

x_ref(1, :) = x;
x_ref(2, :) = y;
x_ref(3, :) = z;
x_ref(4, :) = vx;
x_ref(5, :) = vy;
x_ref(6, :) = vz;
x_ref(7, :) = phi_ref;
x_ref(8, :) = theta_ref;
x_ref(9, :) = psi_ref;

u_ref(1, :) = m * az;

speeds = sqrt(vx.^2 + vy.^2 + vz.^2);
vel_stats.min  = min(speeds);
vel_stats.max  = max(speeds);
vel_stats.mean = mean(speeds);
end
