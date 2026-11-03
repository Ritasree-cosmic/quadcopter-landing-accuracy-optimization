function [A,B] = quadcopter_linear_model( ...
    m,g,L,Ixx,Iyy,Izz,km)

%% ============================================================
% NUMERICAL LINEARIZATION OF THE ACTUAL QUADCOPTER MODEL
%
% State:
% x = [X Y Z Xdot Ydot Zdot phi theta psi p q r]'
%
% Input:
% u = [u1 u2 u3 u4]'   motor thrusts
%
% Linearization is performed directly around the hover
% equilibrium using the nonlinear quadcopter_dynamics function.
% ============================================================

%% Hover equilibrium

x_eq = zeros(12,1);

u_eq = (m*g/4) * ones(4,1);

%% Perturbation sizes

dx_step = 1e-6;
du_step = 1e-6;

%% Time used for linearization

t0 = 0;

%% Evaluate nonlinear model at equilibrium

f0 = quadcopter_dynamics( ...
    t0, ...
    x_eq, ...
    u_eq, ...
    m,g,L,Ixx,Iyy,Izz,km);

%% ============================================================
% Numerical A matrix
% ============================================================

A = zeros(12,12);

for i = 1:12

    x_plus = x_eq;
    x_minus = x_eq;

    x_plus(i) = x_plus(i) + dx_step;
    x_minus(i) = x_minus(i) - dx_step;

    f_plus = quadcopter_dynamics( ...
        t0, ...
        x_plus, ...
        u_eq, ...
        m,g,L,Ixx,Iyy,Izz,km);

    f_minus = quadcopter_dynamics( ...
        t0, ...
        x_minus, ...
        u_eq, ...
        m,g,L,Ixx,Iyy,Izz,km);

    A(:,i) = ...
        (f_plus - f_minus) / ...
        (2*dx_step);

end

%% ============================================================
% Numerical B matrix
% ============================================================

B = zeros(12,4);

for i = 1:4

    u_plus = u_eq;
    u_minus = u_eq;

    u_plus(i) = u_plus(i) + du_step;
    u_minus(i) = u_minus(i) - du_step;

    f_plus = quadcopter_dynamics( ...
        t0, ...
        x_eq, ...
        u_plus, ...
        m,g,L,Ixx,Iyy,Izz,km);

    f_minus = quadcopter_dynamics( ...
        t0, ...
        x_eq, ...
        u_minus, ...
        m,g,L,Ixx,Iyy,Izz,km);

    B(:,i) = ...
        (f_plus - f_minus) / ...
        (2*du_step);

end

%% ============================================================
% Display verification
% ============================================================

fprintf('\n');
fprintf('=============================================\n');
fprintf(' NUMERICAL QUADCOPTER LINEARIZATION\n');
fprintf('=============================================\n');

fprintf('\nHover equilibrium check:\n');

fprintf('Maximum |f(x_eq,u_eq)| = %.6e\n', ...
    max(abs(f0)));

fprintf('\nEigenvalues of A:\n');

disp(eig(A));

fprintf('=============================================\n');

end