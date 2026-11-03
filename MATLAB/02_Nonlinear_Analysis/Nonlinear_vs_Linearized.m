clc;
clear;
close all;

%% NONLINEAR SYSTEM

f = @(t,x)[
x(2);
-x(1)+x(1)^3
];

%% LINEARIZED SYSTEM

A = [0 1;
    -1 0];

g = @(t,x) A*x;

%% INITIAL CONDITION

x0 = [0.2;0.1];

%% SIMULATION

[t1,x1] = ode45(f,[0 20],x0);

[t2,x2] = ode45(g,[0 20],x0);

%% COMPARISON

figure

subplot(2,1,1)

plot(t1,x1(:,1),'r','LineWidth',2)
hold on
plot(t2,x2(:,1),'b--','LineWidth',2)

grid on

title('State x_1')
legend('Nonlinear','Linearized')

subplot(2,1,2)

plot(t1,x1(:,2),'r','LineWidth',2)
hold on
plot(t2,x2(:,2),'b--','LineWidth',2)

grid on

title('State x_2')
legend('Nonlinear','Linearized')