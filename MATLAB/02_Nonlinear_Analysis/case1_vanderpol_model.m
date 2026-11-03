%% Case Study 1
%% Van der Pol Oscillator

clc;
clear;
close all;

mu = 1;

[t,x] = ode45(@(t,x) ...
    [x(2);
     mu*(1-x(1)^2)*x(2)-x(1)],...
     [0 20],[2;0]);

figure

plot(t,x(:,1),'LineWidth',2)

xlabel('Time (s)')
ylabel('x(t)')

title('Van der Pol Oscillator time Response')

grid on