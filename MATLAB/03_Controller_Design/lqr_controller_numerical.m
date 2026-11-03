function [K, A, B] = lqr_controller_numerical( ...
    m,g,L,Ixx,Iyy,Izz,km)

% ============================================================
% NUMERICAL LQR CONTROLLER
% ============================================================

% Obtain linearized model directly from nonlinear dynamics
[A,B] = quadcopter_linear_model( ...
    m,g,L,Ixx,Iyy,Izz,km);

% State weighting matrix
%
% State vector:
% [X Y Z Xdot Ydot Zdot phi theta psi p q r]

Q = diag([ ...
    20, ...   % X
    20, ...   % Y
    30, ...   % Z
    5,  ...   % Xdot
    5,  ...   % Ydot
    10, ...   % Zdot
    50, ...   % phi
    50, ...   % theta
    20, ...   % psi
    2,  ...   % p
    2,  ...   % q
    2]);      % r

% Control weighting
R = diag([1 1 1 1]);

% LQR gain
K = lqr(A,B,Q,R);

fprintf('\n');
fprintf('=============================================\n');
fprintf(' NUMERICAL LQR CONTROLLER\n');
fprintf('=============================================\n');

fprintf('\nLQR gain matrix K:\n');
disp(K);

fprintf('=============================================\n');

end