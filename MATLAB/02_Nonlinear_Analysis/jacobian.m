clc;
clear;
close all;

disp('Jacobian Analysis of Nonlinear System');
disp('System: x1_dot = x2, x2_dot = -x1 + x1^3');
disp(' ');

% Equilibrium points
equilibria = [-1 0;
               0 0;
               1 0];

for i = 1:size(equilibria,1)

    x1 = equilibria(i,1);
    x2 = equilibria(i,2);

    % Jacobian matrix
    J = [0 1;
        -1 + 3*x1^2  0];

    % Eigenvalues
    eigenvalues = eig(J);

    fprintf('Equilibrium Point: (%d, %d)\n', x1, x2);
    disp('Jacobian Matrix:');
    disp(J);

    disp('Eigenvalues:');
    disp(eigenvalues);

    % Classification
    lambda1 = eigenvalues(1);
    lambda2 = eigenvalues(2);

    if isreal(lambda1) && isreal(lambda2)
        if lambda1*lambda2 < 0
            disp('Classification: Saddle Point (Unstable)');
        elseif lambda1 > 0 && lambda2 > 0
            disp('Classification: Unstable Node');
        elseif lambda1 < 0 && lambda2 < 0
            disp('Classification: Stable Node');
        end
    else
        if real(lambda1) == 0
            disp('Classification: Center / Marginally Stable');
        elseif real(lambda1) < 0
            disp('Classification: Stable Spiral');
        else
            disp('Classification: Unstable Spiral');
        end
    end

    disp('-------------------------------------');
end