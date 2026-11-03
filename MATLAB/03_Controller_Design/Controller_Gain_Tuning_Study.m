%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CODE 1: GAIN TUNING STUDY
% Compare multiple state-feedback gains under same disturbance
%
% System:
% x1_dot = x2
% x2_dot = -x1 + x1^3 - k1*x1 - k2*x2 + 0.2*sin(2t)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear;
close all;

%% PARAMETERS
tspan = [0 20];
x0 = [0.5; 0.2];

% Gain sets to compare
gain_sets = [
    1 1;
    3 3;
    5 5
];

labels = {'k1=1, k2=1','k1=3, k2=3','k1=5, k2=5'};

%% FIGURE FOR x1 RESPONSE
figure;
subplot(2,1,1);
hold on;
grid on;
title('Gain Tuning: x1 Response');
xlabel('Time');
ylabel('x1');

%% FIGURE FOR x2 RESPONSE
subplot(2,1,2);
hold on;
grid on;
title('Gain Tuning: x2 Response');
xlabel('Time');
ylabel('x2');

%% PHASE PORTRAIT
figure;
hold on;
grid on;
title('Gain Tuning: Phase Portrait');
xlabel('x1');
ylabel('x2');

%% LOOP THROUGH GAIN SETS
for i = 1:size(gain_sets,1)

    k1 = gain_sets(i,1);
    k2 = gain_sets(i,2);

    system = @(t,x) [
        x(2);
        -x(1) + x(1)^3 - k1*x(1) - k2*x(2) + 0.2*sin(2*t)
    ];

    [t,x] = ode45(system, tspan, x0);

    % x1 plot
    figure(1);
    subplot(2,1,1);
    plot(t,x(:,1),'LineWidth',2);

    % x2 plot
    subplot(2,1,2);
    plot(t,x(:,2),'LineWidth',2);

    % phase portrait
    figure(2);
    plot(x(:,1),x(:,2),'LineWidth',2);

end

%% FINAL LABELS
figure(1);
subplot(2,1,1);
legend(labels);

subplot(2,1,2);
legend(labels);

figure(2);
plot(0,0,'ko','MarkerSize',8,'LineWidth',2);
legend([labels,'Equilibrium']);

disp('===== GAIN TUNING COMPLETE =====');
disp('Compare settling speed, oscillation, and robustness.');