clc;
clear;
close all;

%% PARAMETERS
% Stronger state-feedback gains
k1 = 3;
k2 = 3;

% Simulation time
tspan = [0 20];

% Smaller initial condition (near equilibrium)
x0 = [0.5; 0.2];

%% UNCONTROLLED NONLINEAR SYSTEM
% x1_dot = x2
% x2_dot = -x1 + x1^3
uncontrolled = @(t,x) [
    x(2);
    -x(1) + x(1)^3
];

[t1,x1] = ode45(uncontrolled, tspan, x0);

%% CONTROLLED NONLINEAR SYSTEM
% Control law:
% u = -k1*x1 - k2*x2
%
% Controlled:
% x2_dot = -x1 + x1^3 - k1*x1 - k2*x2
controlled = @(t,x) [
    x(2);
    -x(1) + x(1)^3 - k1*x(1) - k2*x(2)
];

[t2,x2] = ode45(controlled, tspan, x0);

%% FIGURE 1: STATE RESPONSE COMPARISON
figure;

% x1 response
subplot(2,1,1);
plot(t1, x1(:,1), 'r', 'LineWidth', 2);
hold on;
plot(t2, x2(:,1), 'b--', 'LineWidth', 2);
grid on;
xlabel('Time');
ylabel('x1');
title('State x1: Controlled vs Uncontrolled');
legend('Uncontrolled','Controlled');

% x2 response
subplot(2,1,2);
plot(t1, x1(:,2), 'r', 'LineWidth', 2);
hold on;
plot(t2, x2(:,2), 'b--', 'LineWidth', 2);
grid on;
xlabel('Time');
ylabel('x2');
title('State x2: Controlled vs Uncontrolled');
legend('Uncontrolled','Controlled');

%% FIGURE 2: PHASE PORTRAIT COMPARISON
figure;
plot(x1(:,1), x1(:,2), 'r', 'LineWidth', 2);
hold on;
plot(x2(:,1), x2(:,2), 'b--', 'LineWidth', 2);

% Mark initial point
plot(x0(1), x0(2), 'ko', 'MarkerSize', 8, 'LineWidth', 2);

% Mark equilibrium point
plot(0,0,'go','MarkerSize',10,'LineWidth',2);

grid on;
xlabel('x1');
ylabel('x2');
title('Phase Portrait: Controlled vs Uncontrolled');
legend('Uncontrolled','Controlled','Initial Condition','Equilibrium');

% Axis limits for clearer visualization
xlim([-2 2]);
ylim([-2 2]);

%% COMMAND WINDOW OUTPUT
disp(' ');
disp('===== NONLINEAR CONTROL ANALYSIS =====');
disp('System: x1_dot = x2');
disp('        x2_dot = -x1 + x1^3');
disp(' ');
disp('Control Law: u = -k1*x1 - k2*x2');
fprintf('k1 = %.2f, k2 = %.2f\n', k1, k2);
disp(' ');
disp('Initial Condition:');
disp(x0);
disp(' ');
disp('OBSERVATION GUIDE:');
disp('Red Solid  = Uncontrolled');
disp('Blue Dashed = Controlled');
disp('Green Point = Equilibrium');
disp('Black Point = Initial Condition');
disp(' ');
disp('Check whether controlled system converges faster toward equilibrium.');