function dx = quadcopter_dynamics(t, x, u, m, g, L, Ixx, Iyy, Izz, km)
% QUADCOPTER_DYNAMICS
% Nonlinear 12-state quadcopter model
%
% State vector:
% x = [X Y Z Xdot Ydot Zdot phi theta psi p q r]
%
% Inputs:
% u = [T1 T2 T3 T4]
%
% T1, T2, T3, T4 = thrust produced by the four motors

%% States

Xdot  = x(4);
Ydot  = x(5);
Zdot  = x(6);

phi   = x(7);
theta = x(8);
psi   = x(9);

p     = x(10);
q     = x(11);
r     = x(12);

%% Motor thrusts

T1 = u(1);
T2 = u(2);
T3 = u(3);
T4 = u(4);

%% Total thrust

T = T1 + T2 + T3 + T4;

%% Translational dynamics

% Body-to-inertial force transformation

Xddot = (T/m) * (cos(phi)*sin(theta)*cos(psi) + ...
                 sin(phi)*sin(psi));

Yddot = (T/m) * (cos(phi)*sin(theta)*sin(psi) - ...
                 sin(phi)*cos(psi));

Zddot = (T/m) * (cos(phi)*cos(theta)) - g;

%% Rotational dynamics

% Control torques

tau_phi = L * (T2 - T4);

tau_theta = L * (T3 - T1);

tau_psi = km * (T1 - T2 + T3 - T4);

%% Angular acceleration

pdot = ((Iyy - Izz)/Ixx) * q * r + tau_phi/Ixx;

qdot = ((Izz - Ixx)/Iyy) * p * r + tau_theta/Iyy;

rdot = ((Ixx - Iyy)/Izz) * p * q + tau_psi/Izz;

%% Euler angle kinematics

phidot = p + sin(phi)*tan(theta)*q + ...
         cos(phi)*tan(theta)*r;

thetadot = cos(phi)*q - sin(phi)*r;

psidot = (sin(phi)/cos(theta))*q + ...
         (cos(phi)/cos(theta))*r;

%% State derivative

dx = [Xdot;
      Ydot;
      Zdot;
      Xddot;
      Yddot;
      Zddot;
      phidot;
      thetadot;
      psidot;
      pdot;
      qdot;
      rdot];

end