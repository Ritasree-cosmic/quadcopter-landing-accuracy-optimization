%% nonlinear_lqr.m
% LQR Controller Applied to Nonlinear Quadcopter
% Prediction OFF - Baseline Controller

clear;
clc;
close all;

%% Load parameters and LQR controller

quadcopter_parameters;
load('lqr_controller.mat');

%% Simulation time

tspan = [0 10];

%% Hover equilibrium

x_eq = zeros(12,1);

u_eq = [T_motor_hover;
        T_motor_hover;
        T_motor_hover;
        T_motor_hover];

%% Initial disturbance

x_initial = x_eq;

x_initial(1) = 0.5;              % X displacement (m)
x_initial(2) = 0.3;              % Y displacement (m)
x_initial(3) = 0.2;              % Z displacement (m)

x_initial(7) = deg2rad(5);       % Roll disturbance
x_initial(8) = deg2rad(4);       % Pitch disturbance
x_initial(9) = deg2rad(3);       % Yaw disturbance

%% LQR control law

% u = u_eq - K(x - x_eq)

control_law = @(x) ...
    u_eq - K*(x - x_eq);

%% Nonlinear closed-loop dynamics

nonlinear_closed_loop = @(t,x) ...
    quadcopter_dynamics( ...
    t, ...
    x, ...
    control_law(x), ...
    m, g, L, Ixx, Iyy, Izz, km);

%% Simulate nonlinear system

[t,x] = ode45( ...
    nonlinear_closed_loop, ...
    tspan, ...
    x_initial);

%% Extract states

X = x(:,1);
Y = x(:,2);
Z = x(:,3);

Xdot = x(:,4);
Ydot = x(:,5);
Zdot = x(:,6);

phi   = x(:,7);
theta = x(:,8);
psi   = x(:,9);

p = x(:,10);
q = x(:,11);
r = x(:,12);

%% ========================================================
% Position response
% =========================================================

figure;

plot(t,X,'LineWidth',1.5);
hold on;
plot(t,Y,'LineWidth',1.5);
plot(t,Z,'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('Position (m)');

title('Nonlinear Quadcopter: LQR Position Response');

legend('X','Y','Z');

%% ========================================================
% Attitude response
% =========================================================

figure;

plot(t,rad2deg(phi),'LineWidth',1.5);
hold on;
plot(t,rad2deg(theta),'LineWidth',1.5);
plot(t,rad2deg(psi),'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('Angle (degrees)');

title('Nonlinear Quadcopter: LQR Attitude Response');

legend('Roll','Pitch','Yaw');

%% ========================================================
% Velocity response
% =========================================================

figure;

plot(t,Xdot,'LineWidth',1.5);
hold on;
plot(t,Ydot,'LineWidth',1.5);
plot(t,Zdot,'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('Velocity (m/s)');

title('Nonlinear Quadcopter: LQR Velocity Response');

legend('X velocity','Y velocity','Z velocity');

%% ========================================================
% Angular velocity response
% =========================================================

figure;

plot(t,p,'LineWidth',1.5);
hold on;
plot(t,q,'LineWidth',1.5);
plot(t,r,'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('Angular velocity (rad/s)');

title('Nonlinear Quadcopter: LQR Angular Velocity');

legend('p','q','r');

%% ========================================================
% Final results
% =========================================================

fprintf('\n========================================\n');
fprintf('      NONLINEAR LQR RESULTS\n');
fprintf('========================================\n');

fprintf('\nInitial position:\n');
fprintf('X = %.3f m\n',X(1));
fprintf('Y = %.3f m\n',Y(1));
fprintf('Z = %.3f m\n',Z(1));

fprintf('\nFinal position:\n');
fprintf('X = %.6f m\n',X(end));
fprintf('Y = %.6f m\n',Y(end));
fprintf('Z = %.6f m\n',Z(end));

fprintf('\nFinal attitude:\n');
fprintf('Roll  = %.6f deg\n',rad2deg(phi(end)));
fprintf('Pitch = %.6f deg\n',rad2deg(theta(end)));
fprintf('Yaw   = %.6f deg\n',rad2deg(psi(end)));

fprintf('\n========================================\n');