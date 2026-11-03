function x_measured = noisy_measurement_model(x_true)

% ============================================================
% NOISY MEASUREMENT MODEL
% Simulates noisy sensor measurements of the quadcopter state
%
% Input:
%   x_true      - actual 12-state quadcopter state
%
% Output:
%   x_measured  - simulated noisy measurement
%
% State vector:
% [X Y Z Xdot Ydot Zdot phi theta psi p q r]
% ============================================================

% ---------------- Sensor noise levels ----------------

% Position measurement noise (m)
sigma_position = 0.03;

% Velocity measurement noise (m/s)
sigma_velocity = 0.05;

% Attitude measurement noise (rad)
sigma_angle = deg2rad(0.5);

% Angular velocity measurement noise (rad/s)
sigma_angular_velocity = deg2rad(1.0);


% ---------------- Generate noisy measurement ----------------

x_measured = x_true;

% Position: X, Y, Z
x_measured(1:3) = x_true(1:3) + ...
    sigma_position * randn(3,1);

% Linear velocity: Xdot, Ydot, Zdot
x_measured(4:6) = x_true(4:6) + ...
    sigma_velocity * randn(3,1);

% Attitude: phi, theta, psi
x_measured(7:9) = x_true(7:9) + ...
    sigma_angle * randn(3,1);

% Angular velocity: p, q, r
x_measured(10:12) = x_true(10:12) + ...
    sigma_angular_velocity * randn(3,1);

end