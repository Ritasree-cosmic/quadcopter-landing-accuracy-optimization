%% linearization.m
% Linearization of nonlinear quadcopter model around hover

clear;
clc;
close all;

%% Load parameters

quadcopter_parameters;

%% Symbolic variables

syms X Y Z Xdot Ydot Zdot real
syms phi theta psi p q r real

syms T1 T2 T3 T4 real

%% State vector

x_sym = [X;
         Y;
         Z;
         Xdot;
         Ydot;
         Zdot;
         phi;
         theta;
         psi;
         p;
         q;
         r];

%% Input vector

u_sym = [T1;
         T2;
         T3;
         T4];

%% Total thrust

T = T1 + T2 + T3 + T4;

%% Nonlinear equations

Xddot = (T/m) * (cos(phi)*sin(theta)*cos(psi) + ...
                 sin(phi)*sin(psi));

Yddot = (T/m) * (cos(phi)*sin(theta)*sin(psi) - ...
                 sin(phi)*cos(psi));

Zddot = (T/m) * cos(phi)*cos(theta) - g;

%% Control torques

tau_phi = L * (T2 - T4);

tau_theta = L * (T3 - T1);

tau_psi = km * (T1 - T2 + T3 - T4);

%% Angular accelerations

pdot = ((Iyy - Izz)/Ixx)*q*r + tau_phi/Ixx;

qdot = ((Izz - Ixx)/Iyy)*p*r + tau_theta/Iyy;

rdot = ((Ixx - Iyy)/Izz)*p*q + tau_psi/Izz;

%% Euler angle kinematics

phidot = p + sin(phi)*tan(theta)*q + ...
         cos(phi)*tan(theta)*r;

thetadot = cos(phi)*q - sin(phi)*r;

psidot = (sin(phi)/cos(theta))*q + ...
         (cos(phi)/cos(theta))*r;

%% Complete nonlinear state equation

f = [Xdot;
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

%% Jacobian matrices

A = jacobian(f, x_sym);

B = jacobian(f, u_sym);

%% Hover equilibrium

x_eq = zeros(12,1);

T_eq = m*g/4;

u_eq = [T_eq;
        T_eq;
        T_eq;
        T_eq];

%% Evaluate Jacobians at hover

A_hover = double(subs(A, ...
    [x_sym; u_sym], ...
    [x_eq; u_eq]));

B_hover = double(subs(B, ...
    [x_sym; u_sym], ...
    [x_eq; u_eq]));

%% Display results

fprintf('\n========================================\n');
fprintf('     QUADCOPTER HOVER LINEARIZATION\n');
fprintf('========================================\n');

fprintf('\nHover state:\n');
disp(x_eq);

fprintf('Hover motor thrusts:\n');
disp(u_eq);

fprintf('\nLinearized A matrix:\n');
disp(A_hover);

fprintf('\nLinearized B matrix:\n');
disp(B_hover);

%% Eigenvalues of A matrix

eigenvalues_A = eig(A_hover);

fprintf('\nEigenvalues of A matrix:\n');
disp(eigenvalues_A);

%% Save matrices

save('linearized_quadcopter.mat', ...
     'A_hover', 'B_hover', ...
     'x_eq', 'u_eq', ...
     'm', 'g', 'L', ...
     'Ixx', 'Iyy', 'Izz', 'km');

fprintf('\nLinearized model saved as:\n');
fprintf('linearized_quadcopter.mat\n');

fprintf('========================================\n');