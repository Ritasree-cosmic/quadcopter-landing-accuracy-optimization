%% CASE STUDY 2
%% Quadcopter Controller Design

clc;
clear;
close all;

a = 2;
b = 0.5;

%% Linearized Model

A = [0 1;
    -a -b];

B = [0;
      1];

%% Performance Specifications

OS = 5;
Ts = 1.5;

%% Damping Ratio

zeta = -log(OS/100)/...
sqrt(pi^2 + (log(OS/100))^2);

%% Natural Frequency

wn = 4/(zeta*Ts);

%% Desired Poles

sigma = zeta*wn;

wd = wn*sqrt(1-zeta^2);

desired_poles = ...
[-sigma+1i*wd ...
 -sigma-1i*wd];

disp('Desired Poles')
disp(desired_poles)

%% Pole Placement

K = place(A,B,desired_poles);

disp('Feedback Gain K')
disp(K)

%% Closed Loop

Acl = A - B*K;

disp('Closed Loop Eigenvalues')
disp(eig(Acl))

%% Initial Condition

sys_cl = ss(Acl,B,eye(2),0);

x0 = [0.5;
      0];

t = 0:0.01:10;

[~,t,x] = initial(sys_cl,x0,t);

%% Roll Angle

figure

plot(t,x(:,1),...
    'LineWidth',2)

xlabel('Time (s)')
ylabel('\phi (rad)')

title('Controlled Roll Angle Response')

grid on

%% Roll Rate

figure

plot(t,x(:,2),...
    'LineWidth',2)

xlabel('Time (s)')
ylabel('p (rad/s)')

title('Controlled Roll Rate Response')

grid on

%% State Trajectory

figure

plot(x(:,1),x(:,2),...
    'LineWidth',2)

hold on

plot(0,0,'ro',...
    'MarkerSize',10,...
    'LineWidth',2)

xlabel('\phi')
ylabel('p')

title('Controlled State Trajectory')

grid on