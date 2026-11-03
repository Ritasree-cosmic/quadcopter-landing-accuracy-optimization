%% lqr_closed_loop.m
% LQR Closed-Loop Quadcopter Response
% Prediction OFF - Baseline Controller

clear;
clc;
close all;

%% Load models

load('linearized_quadcopter.mat');
load('lqr_controller.mat');

%% Simulation time

tspan = [0 10];

%% Hover equilibrium

x_eq = zeros(12,1);

T_hover_total = m*g;

u_eq = [T_hover_total/4;
        T_hover_total/4;
        T_hover_total/4;
        T_hover_total/4];

%% Initial disturbance

% Small displacement and attitude disturbance

x_initial = x_eq;

x_initial(1) = 0.5;              % X position disturbance (m)
x_initial(2) = 0.3;              % Y position disturbance (m)
x_initial(3) = 0.2;              % Z position disturbance (m)

x_initial(7) = deg2rad(5);       % Roll disturbance
x_initial(8) = deg2rad(4);       % Pitch disturbance
x_initial(9) = deg2rad(3);       % Yaw disturbance

%% Closed-loop dynamics

closed_loop_dynamics = @(t,x) ...
    Acl*(x-x_eq);

%% Simulate

[t,x] = ode45(closed_loop_dynamics, ...
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

title('LQR Closed-Loop Position Response');

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

title('LQR Closed-Loop Attitude Response');

legend('Roll \phi','Pitch \theta','Yaw \psi');

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

title('LQR Closed-Loop Velocity Response');

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

title('LQR Closed-Loop Angular Velocity');

legend('p','q','r');

%% ========================================================
% Final state
% =========================================================

fprintf('\n========================================\n');
fprintf('       LQR CLOSED-LOOP RESULTS\n');
fprintf('========================================\n');

fprintf('\nInitial position:\n');
fprintf('X = %.3f m\n',x_initial(1));
fprintf('Y = %.3f m\n',x_initial(2));
fprintf('Z = %.3f m\n',x_initial(3));

fprintf('\nFinal position:\n');
fprintf('X = %.6f m\n',X(end));
fprintf('Y = %.6f m\n',Y(end));
fprintf('Z = %.6f m\n',Z(end));

fprintf('\nFinal attitude:\n');
fprintf('Roll  = %.6f deg\n',rad2deg(phi(end)));
fprintf('Pitch = %.6f deg\n',rad2deg(theta(end)));
fprintf('Yaw   = %.6f deg\n',rad2deg(psi(end)));

fprintf('\n========================================\n');