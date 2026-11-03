%% CASE STUDY 2
%% Quadcopter Roll Dynamics

clc;
clear;
close all;

a = 2;
b = 0.5;

tspan = [0 20];

x0 = [1;
      0];

[t,x] = ode45(@(t,x) quad_model(t,x,a,b),...
              tspan,x0);

%% Roll Angle

figure

plot(t,x(:,1),'LineWidth',2)

xlabel('Time (s)')
ylabel('\phi (rad)')

title('Quadcopter Roll Angle Response')

grid on

%% Roll Rate

figure

plot(t,x(:,2),'LineWidth',2)

xlabel('Time (s)')
ylabel('p (rad/s)')

title('Quadcopter Roll Rate Response')

grid on

%% Local Function

function dx = quad_model(~,x,a,b)

phi = x(1);
p   = x(2);

dx = zeros(2,1);

dx(1) = p;

dx(2) = -a*sin(phi) - b*p;

end