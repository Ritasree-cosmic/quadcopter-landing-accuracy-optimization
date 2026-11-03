
function dx = quadcopter_dynamics_disturbed(t, x, u, m, g, L, Ixx, Iyy, Izz, km)
% QUADCOPTER_DYNAMICS_DISTURBED
% Original nonlinear model with deterministic wind-like
% translational disturbance accelerations.
%
% State: [X Y Z Xdot Ydot Zdot phi theta psi p q r]'
% Input: [T1 T2 T3 T4]'
%
% The disturbance is an illustrative simulation input, not a
% measured wind profile or a validated aerodynamic model.

dx = quadcopter_dynamics(t, x, u, m, g, L, Ixx, Iyy, Izz, km);

% Reproducible time-varying disturbance accelerations (m/s^2).
axWind = 0.20*sin(0.7*t) + 0.08*sin(2.3*t);
ayWind = 0.15*sin(0.9*t + 0.4) + 0.05*sin(2.7*t);
azWind = 0.03*sin(1.1*t + 0.2);

% Add disturbances to inertial translational accelerations.
dx(4) = dx(4) + axWind;
dx(5) = dx(5) + ayWind;
dx(6) = dx(6) + azWind;

end
