
clear; clc;

% Quadcopter parameters
m = 0.50;
g = 9.81;
L = 0.20;
Ixx = 0.005;
Iyy = 0.005;
Izz = 0.009;
km = 1e-6;
Ts = 0.01;

% Continuous-time linear model
[A, B] = quadcopter_linear_model( ...
    m, g, L, Ixx, Iyy, Izz, km);

% Discretize the model
sys_c = ss(A, B, eye(12), zeros(12,4));
sys_d = c2d(sys_c, Ts);

Ad = sys_d.A;
Bd = sys_d.B;

% Same LQR weights
Q = diag([20,20,30,5,5,10,50,50,20,2,2,2]);
R = eye(4);

% Discrete-time LQR
[Kd, ~, poles] = dlqr(Ad, Bd, Q, R);

% Closed-loop eigenvalues
Acl_d = Ad - Bd*Kd;
lambda_d = eig(Acl_d);

fprintf('\nDiscrete-time closed-loop eigenvalues:\n');
disp(lambda_d);

fprintf('Maximum eigenvalue magnitude: %.8f\n', ...
    max(abs(lambda_d)));

if all(abs(lambda_d) < 1)
    fprintf('RESULT: Discrete-time closed loop is stable.\n');
else
    fprintf('RESULT: Discrete-time closed loop is NOT stable.\n');
end
