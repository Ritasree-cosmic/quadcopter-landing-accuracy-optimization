
%% COMPARE LANDING CONTROLLERS UNDER SENSOR NOISE AND WIND
% Controller A: Standard discrete-time LQR
% Controller B: One-step predictive LQR, without wind compensation
% Controller C: One-step predictive LQR, with known simulated wind
%
% Same reference, initial state, sensor noise, and wind realization.
%
% IMPORTANT:
% This is a one-step predictive feedback comparison, not full MPC.
% Controller C uses known simulated wind as an idealized diagnostic.
% Validate results before claiming prediction improves landing accuracy.

clear;
clc;
close all;

fprintf('\n============================================\n');
fprintf(' CLOSED-LOOP LANDING CONTROLLER COMPARISON\n');
fprintf('============================================\n');

%% 1. CHECK REQUIRED FILES

requiredFiles = {
    'landing_reference_data_rebuilt.mat'
    'quadcopter_dynamics.m'
    'quadcopter_linear_model.m'
};

for i = 1:numel(requiredFiles)
    if ~isfile(requiredFiles{i})
        error('Required file not found: %s', requiredFiles{i});
    end
end

% Load the rebuilt reference file, not the corrupted original.
D = load('landing_reference_data_rebuilt.mat');

if ~isfield(D,'t') || ~isfield(D,'x_ref') || ~isfield(D,'dt')
    error('Reference MAT file must contain t, x_ref and dt.');
end

t = D.t(:)';
x_ref = D.x_ref;
Ts = D.dt;
N = numel(t);

if size(x_ref,1) ~= 12 || size(x_ref,2) ~= N
    error('x_ref must be a 12-by-length(t) matrix.');
end

if N < 2 || Ts <= 0
    error('Reference time vector or sample time is invalid.');
end

if any(abs(diff(t)-Ts) > max(1e-8,Ts*1e-4))
    error('Time vector spacing does not match D.dt.');
end

fprintf('Reference data loaded successfully.\n');
fprintf('Samples: %d\n', N);
fprintf('Duration: %.2f seconds\n', t(end));
fprintf('Sample time: %.4f seconds\n', Ts);

%% 2. QUADCOPTER PARAMETERS

m = 0.50;
g = 9.81;
L = 0.20;

Ixx = 0.005;
Iyy = 0.005;
Izz = 0.009;
km = 1e-6;

x_eq = zeros(12,1);
u_eq = (m*g/4)*ones(4,1);

%% 3. LINEAR MODEL AND DISCRETE-TIME LQR

fprintf('\nBuilding linear and discrete-time models...\n');

[A,B] = quadcopter_linear_model( ...
    m,g,L,Ixx,Iyy,Izz,km);

sys = ss(A,B,eye(12),zeros(12,4));
sysd = c2d(sys,Ts);

Ad = sysd.A;
Bd = sysd.B;

% LQR weights
Q_lqr = diag([20,20,30,5,5,10,50,50,20,2,2,2]);
R_lqr = eye(4);

% Design gain for the discrete-time model.
K = dlqr(Ad,Bd,Q_lqr,R_lqr);

closedLoopPoles = eig(Ad-Bd*K);
maxPoleMagnitude = max(abs(closedLoopPoles));

fprintf('Sample time: %.5f s\n',Ts);
fprintf('Maximum discrete closed-loop pole magnitude: %.8f\n', ...
    maxPoleMagnitude);

if maxPoleMagnitude >= 1
    warning(['The discrete-time LQR closed loop is not stable ', ...
        'according to the pole test.']);
else
    fprintf('Discrete-time LQR pole test: stable.\n');
end

%% 4. AUGMENTED MODEL FOR WIND-AWARE PREDICTION

% Wind input vector: [Fx; Fy; Fz], in newtons.
% Translational acceleration receives force divided by mass.

E = zeros(12,3);
E(4,1) = 1/m;
E(5,2) = 1/m;
E(6,3) = 1/m;

sysAug = ss(A,[B E],eye(12),zeros(12,7));
sysAugD = c2d(sysAug,Ts);

AdW = sysAugD.A;
BdControlW = sysAugD.B(:,1:4);
BdWindW = sysAugD.B(:,5:7);

%% 5. SENSOR NOISE AND KALMAN FILTER SETTINGS

sigma_pos = 0.03;
sigma_vel = 0.05;
sigma_angle = deg2rad(0.5);
sigma_rate = deg2rad(1.0);

sensorStd = [
    sigma_pos; sigma_pos; sigma_pos
    sigma_vel; sigma_vel; sigma_vel
    sigma_angle; sigma_angle; sigma_angle
    sigma_rate; sigma_rate; sigma_rate
];

R_meas = diag(sensorStd.^2);

Q_kf = diag([
    1e-6 1e-6 1e-6 ...
    1e-5 1e-5 1e-5 ...
    1e-6 1e-6 1e-6 ...
    1e-5 1e-5 1e-5
]);

H = eye(12);
I12 = eye(12);

% Generate one shared measurement-noise sequence.
rng(1,'twister');
measurementNoise = sensorStd .* randn(12,N);

%% 6. WIND MODEL

windFx = 0.08*sin(0.8*t);
windFy = 0.06*cos(0.6*t);
windFz = zeros(1,N);

%% 7. EXPERIMENT SETTINGS

modeNames = {
    'Standard LQR'
    'Prediction without wind compensation'
    'Prediction with known wind'
};

nModes = numel(modeNames);

targetPosition = [0;0;0];

landingAltitude = 0.15;
maxVerticalSpeed = 0.30;

% Arrays for simulation results.
Xall = zeros(12,N,nModes);
XhatAll = zeros(12,N,nModes);
Uall = zeros(4,N,nModes);

%% 8. RUN THE THREE SIMULATIONS

for mode = 1:nModes

    fprintf('\nRunning controller: %s\n',modeNames{mode});

    X = zeros(12,N);
    Xhat = zeros(12,N);
    U = zeros(4,N);

    % Identical initial condition and initial measurement noise.
    x = x_ref(:,1);
    xhat = x + measurementNoise(:,1);

    P = 0.01*eye(12);

    X(:,1) = x;
    Xhat(:,1) = xhat;

    for k = 1:N-1

        %% A. COMPUTE THE CURRENT PROVISIONAL COMMAND

        uProvisional = u_eq - K*(xhat-x_ref(:,k));
        uProvisional = max(uProvisional,0);

        %% B. CHOOSE THE CONTROL STATE

        if mode == 1

            % Standard LQR uses the current estimated state.
            xControl = xhat;
            xReference = x_ref(:,k);

            u = u_eq - K*(xControl-xReference);
            u = max(u,0);

        else

            % Predict one sample ahead using the provisional command.
            % The plant state is represented as a deviation from hover.
            deltaX = xhat-x_eq;
            deltaUProvisional = uProvisional-u_eq;

            if mode == 2

                % Prediction without explicit wind compensation.
                deltaXPred = ...
                    Ad*deltaX + Bd*deltaUProvisional;

            else

                % Idealized diagnostic using known simulated wind.
                windInput = [
                    windFx(k)
                    windFy(k)
                    windFz(k)
                ];

                deltaXPred = ...
                    AdW*deltaX ...
                    + BdControlW*deltaUProvisional ...
                    + BdWindW*windInput;
            end

            xPred = x_eq+deltaXPred;

            % Use the next reference sample.
            xReference = x_ref(:,k+1);

            % Final command for this plant interval.
            u = u_eq-K*(xPred-xReference);
            u = max(u,0);

        end

        %% C. APPLY CONTROL TO THE NONLINEAR PLANT

        U(:,k) = u;

        Fx = windFx(k);
        Fy = windFy(k);
        Fz = windFz(k);

        [~,segment] = ode45( ...
            @(tt,xx) plantWithWind( ...
                tt,xx,u,m,g,L,Ixx,Iyy,Izz,km,Fx,Fy,Fz), ...
            [t(k),t(k+1)],x);

        x = segment(end,:)';
        X(:,k+1) = x;

        %% D. SIMULATED SENSOR MEASUREMENT

        y = x+measurementNoise(:,k+1);

        %% E. KALMAN FILTER UPDATE

        % The filter uses the same nominal model for all controllers.
        % Wind is not directly estimated by this filter.
        xhatMinus = Ad*xhat+Bd*(u-u_eq);
        Pminus = Ad*P*Ad'+Q_kf;

        S = H*Pminus*H'+R_meas;
        Kgain = (Pminus*H')/S;

        xhat = xhatMinus+Kgain*(y-H*xhatMinus);

        % Joseph-form covariance update.
        P = (I12-Kgain*H)*Pminus*(I12-Kgain*H)' ...
            + Kgain*R_meas*Kgain';

        P = (P+P')/2;

        Xhat(:,k+1) = xhat;

    end

    U(:,N) = U(:,N-1);

    Xall(:,:,mode) = X;
    XhatAll(:,:,mode) = Xhat;
    Uall(:,:,mode) = U;

    fprintf('Simulation complete.\n');

end

%% 9. CALCULATE PERFORMANCE METRICS

PositionRMSE = zeros(nModes,1);
HorizontalLandingError_cm = zeros(nModes,1);
VerticalLandingError_cm = zeros(nModes,1);
Final3DError_cm = zeros(nModes,1);
FinalVerticalSpeed_mps = zeros(nModes,1);
LandingZoneEntryTime_s = nan(nModes,1);
ControlEffort = zeros(nModes,1);

for mode = 1:nModes

    X = Xall(:,:,mode);

    % Position RMSE against the complete reference trajectory.
    positionError = X(1:3,:)-x_ref(1:3,:);

    PositionRMSE(mode) = sqrt(mean(positionError(:).^2));

    % Final position error relative to the origin.
    finalPosition = X(1:3,end);

    HorizontalLandingError_cm(mode) = ...
        100*norm(finalPosition(1:2)-targetPosition(1:2));

    VerticalLandingError_cm(mode) = ...
        100*abs(finalPosition(3)-targetPosition(3));

    Final3DError_cm(mode) = ...
        100*norm(finalPosition-targetPosition);

    FinalVerticalSpeed_mps(mode) = abs(X(6,end));

    % First time the simulation meets the landing-zone criteria.
    idx = find( ...
        X(3,:) <= landingAltitude & ...
        abs(X(6,:)) <= maxVerticalSpeed,1,'first');

    if ~isempty(idx)
        LandingZoneEntryTime_s(mode) = t(idx);
    end

    % Integrated squared control deviation from hover.
    deltaU = Uall(:,:,mode)-u_eq;
    ControlEffort(mode) = sum(deltaU(:).^2)*Ts;

end

%% 10. DISPLAY AND SAVE RESULTS

Controller = modeNames(:);

Results = table( ...
    Controller, ...
    PositionRMSE, ...
    HorizontalLandingError_cm, ...
    VerticalLandingError_cm, ...
    Final3DError_cm, ...
    FinalVerticalSpeed_mps, ...
    LandingZoneEntryTime_s, ...
    ControlEffort);

fprintf('\n============================================\n');
fprintf(' CLOSED-LOOP PERFORMANCE RESULTS\n');
fprintf('============================================\n');

disp(Results);

% Save outputs to dedicated comparison files.
% These filenames may replace results from an earlier run.
writetable(Results,'landing_controller_comparison.csv');

save('landing_controller_comparison.mat', ...
    'Results','Xall','XhatAll','Uall','t','x_ref', ...
    'modeNames','windFx','windFy','windFz','K','Ad','Bd','Ts');

%% 11. PLOT POSITION TRAJECTORIES

figure('Name','Landing Trajectories','Color','w');

for axisIndex = 1:3

    subplot(3,1,axisIndex);

    plot(t,x_ref(axisIndex,:),'k--','LineWidth',1.5);
    hold on;

    for mode = 1:nModes
        plot(t,Xall(axisIndex,:,mode),'LineWidth',1.2);
    end

    if axisIndex == 1
        ylabel('X (m)');
    elseif axisIndex == 2
        ylabel('Y (m)');
    else
        ylabel('Z (m)');
        xlabel('Time (s)');
    end

    grid on;
    hold off;

end

% Explicit cell array for the legend labels.
legend([{'Reference'}; modeNames], 'Location','best');
sgtitle('Reference vs Actual Landing Trajectories');

%% 12. PLOT HORIZONTAL POSITION ERROR

figure('Name','Horizontal Landing Error','Color','w');

for mode = 1:nModes

    X = Xall(:,:,mode);

    horizontalError = sqrt( ...
        (X(1,:)-targetPosition(1)).^2 + ...
        (X(2,:)-targetPosition(2)).^2);

    plot(t,100*horizontalError,'LineWidth',1.4);
    hold on;

end

xlabel('Time (s)');
ylabel('Horizontal error from target (cm)');
title('Horizontal Position Error During Landing');
legend(modeNames,'Location','best');
grid on;
hold off;

%% 13. PLOT VERTICAL SPEED

figure('Name','Vertical Speed','Color','w');

for mode = 1:nModes
    plot(t,Xall(6,:,mode),'LineWidth',1.4);
    hold on;
end

yline(maxVerticalSpeed,'k--','Landing speed limit');
yline(-maxVerticalSpeed,'k--');

xlabel('Time (s)');
ylabel('Vertical speed (m/s)');
title('Vertical Speed During Landing');
legend([modeNames; {'Speed limit'; 'Negative speed limit'}], ...
    'Location','best');
grid on;
hold off;

%% 14. REPORT SAVED FILES

fprintf('\nSaved results:\n');
fprintf('  landing_controller_comparison.csv\n');
fprintf('  landing_controller_comparison.mat\n');

fprintf('\nController comparison complete.\n');

%% LOCAL FUNCTION: NONLINEAR PLANT WITH WIND

function dx = plantWithWind(tt,x,u,m,g,L,Ixx,Iyy,Izz,km,Fx,Fy,Fz)

    dx = quadcopter_dynamics( ...
        tt,x,u,m,g,L,Ixx,Iyy,Izz,km);

    dx(4) = dx(4)+Fx/m;
    dx(5) = dx(5)+Fy/m;
    dx(6) = dx(6)+Fz/m;

end
