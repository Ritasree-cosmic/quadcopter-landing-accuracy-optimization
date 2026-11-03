
clear; clc;

% Quadcopter parameters
m = 0.50;
g = 9.81;
L = 0.20;
Ixx = 0.005;
Iyy = 0.005;
Izz = 0.009;
km = 1e-6;

% Linear model
[A, B] = quadcopter_linear_model( ...
    m, g, L, Ixx, Iyy, Izz, km);

% LQR weights — same as your controller
Q = diag([20,20,30,5,5,10,50,50,20,2,2,2]);
R = eye(4);

% Design LQR
K = lqr(A, B, Q, R);

% Closed-loop eigenvalues
Acl = A - B*K;
lambda = eig(Acl);

fprintf('\nClosed-loop eigenvalues of A-BK:\n');
disp(lambda);

fprintf('Maximum real part: %.8f\n', max(real(lambda)));

if all(real(lambda) < 0)
    fprintf('RESULT: Continuous-time closed loop is stable.\n');
else
    fprintf('RESULT: Continuous-time closed loop is NOT stable.\n');
end
