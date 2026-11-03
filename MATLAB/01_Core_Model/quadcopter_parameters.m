% 01_quadcopter_parameters.m
% Quadcopter Parameters
% Purpose: Define all physical and simulation parameters

clear;
clc;
close all;

%% Physical parameters

m = 0.50;          % Mass of quadcopter (kg)
g = 9.81;          % Gravitational acceleration (m/s^2)

L = 0.20;          % Arm length (m)

% Moments of inertia
Ixx = 0.005;       % kg.m^2
Iyy = 0.005;       % kg.m^2
Izz = 0.009;       % kg.m^2

%% Propeller parameters

kf = 1e-5;         % Thrust coefficient
km = 1e-6;         % Drag/moment coefficient

%% Hover condition

% Total thrust required for hover
T_hover = m*g;

% Approximate thrust produced by each motor during hover
T_motor_hover = T_hover/4;

%% Initial state

% State vector:
%
% x = [X Y Z X_dot Y_dot Z_dot phi theta psi ...
%      p q r]
%
% Position:
X0 = 0;
Y0 = 0;
Z0 = 0;

% Linear velocity:
Xdot0 = 0;
Ydot0 = 0;
Zdot0 = 0;

% Attitude:
phi0   = 0;       % Roll (rad)
theta0 = 0;       % Pitch (rad)
psi0   = 0;       % Yaw (rad)

% Angular velocity:
p0 = 0;
q0 = 0;
r0 = 0;

%% Initial state vector

x0 = [X0;
      Y0;
      Z0;
      Xdot0;
      Ydot0;
      Zdot0;
      phi0;
      theta0;
      psi0;
      p0;
      q0;
      r0];

%% Display parameters

fprintf('========================================\n');
fprintf('       QUADCOPTER PARAMETERS\n');
fprintf('========================================\n');

fprintf('Mass              = %.3f kg\n', m);
fprintf('Gravity           = %.3f m/s^2\n', g);
fprintf('Arm length        = %.3f m\n', L);

fprintf('Ixx               = %.5f kg.m^2\n', Ixx);
fprintf('Iyy               = %.5f kg.m^2\n', Iyy);
fprintf('Izz               = %.5f kg.m^2\n', Izz);

fprintf('Thrust coefficient = %.2e\n', kf);
fprintf('Moment coefficient = %.2e\n', km);

fprintf('\nHover condition:\n');
fprintf('Total thrust      = %.3f N\n', T_hover);
fprintf('Thrust per motor  = %.3f N\n', T_motor_hover);

fprintf('\nInitial state:\n');
disp(x0);

fprintf('========================================\n');