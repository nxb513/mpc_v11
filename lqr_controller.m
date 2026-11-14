function [K_lqr, P_term] = lqr_controller(Ad, Bd, Q, R)
% LQR_CONTROLLER Solves the Discrete Algebraic Riccati Equation (DARE)
% Returns optimal feedback gain K_lqr and terminal cost matrix P_term.

P_term = idare(Ad, Bd, Q, R, [], []);
K_lqr = (R + Bd' * P_term * Bd) \ (Bd' * P_term * Ad);
end
