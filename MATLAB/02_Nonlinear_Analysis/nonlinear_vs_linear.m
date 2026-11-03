clc;
clear;
close all;

tspan = [0 5];
x0 = 1.1;   % Initial condition near x = 1

% Nonlinear system
[t1,x1] = ode45(@(t,x) x - x^3, tspan, x0);

% Linearized system around x = 1
[t2,x2] = ode45(@(t,x) -2*(x-1), tspan, x0);

% Plot comparison
figure;
plot(t1, x1, 'b', 'LineWidth', 2);
hold on;
plot(t2, x2, 'r--', 'LineWidth', 2);

grid on;
xlabel('Time');
ylabel('x(t)');
title('Nonlinear vs Linearized System near x = 1');
legend('Nonlinear','Linearized');