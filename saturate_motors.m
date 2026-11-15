function [u_sat, f_sat] = saturate_motors(u, p)
% SATURATE_MOTORS Actuator-level saturation. Maps the commanded
% u = [dT; tau_x; tau_y; tau_z] (deviation from hover) to rotor thrusts,
% clips every rotor to [T_min, T_max] [S2, eq. 17], and maps back.
% The applied command therefore always lies within the actuator limits.
f = p.Mix_inv * (u + p.u_hover);
f_sat = min(p.T_max, max(p.T_min, f));
u_sat = p.Mix * f_sat - p.u_hover;
end
